#include <stdlib.h>

#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "hal.h"                /* HAL public API decls */





#define RINGBUFCAP 32 // incoming ring buffer capacitz in bytes



/* module information */
MODULE_AUTHOR("Boris Skegin");
MODULE_DESCRIPTION("Test source for serial OSC output of XIO NGIMU");
MODULE_LICENSE("LGPL");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/


/** This structure contains the runtime data for the component.
*/

static int count = 1;           /* number of instances */
RTAPI_MP_INT( count, "number of testsource instances");

static int debug = 0;
RTAPI_MP_INT( debug, "debug level");


static char* input = "\x23\x62\x75\x6E\x64\x6C\x65\x00\xBC\x5E\xCB\xEE\x45\x9A\x30\x00\x00\x00" \
                      "\x00\x24\x2F\x71\x75\x61\x74\x65\x72\x6E\x69\x6F\x6E\x00\x2C\x66\x66\x66\x66" \
					  "\x00\x00\x00\x3F\x6E\x9F\xAD\xBB\xB9\x44\x68\xBB\x88\xDF\x5D\x3E\xB6\xFB\x02\xC0" \
					  "\x23\x62\x75\x6E\x64\x6C\x65\x00\xBC\x5E\xCB\xEE\x45\x9A\x30\x00\x00\x00\x00\x1C" \
					  "\x2F\x65\x75\x6C\x65\x72\x00\x00\x2C\x66\x66\x66\x00\x00\x00\x00\x3E\xDF\x24\x93\x3F" \
					  "\x2D\x7F\x44\xC2\x28\x53\xC5\xC0\x23\x62\x75\x6E\x64\x6C\x65\x00\xBC\x5E\xCB\xEE\x45\x9A\x30\x00\x00\x00\x00\x1C\x2F\x65\x61\x72\x74\x68\x00\x00\x2C\x66\x66\x66\x00\x00\x00\x00\x39\xEC\xD2\xAE\xBA\x84\xDF\x2A\xBB\xF3\x4D\x24\xC0\x23\x62\x75\x6E\x64\x6C\x65\x00\xBC\x5E\xCB\xEF\x45\xAA\x68\x00\x00\x00\x00\x24\x2F\x71\x75\x61\x74\x65\x72\x6E\x69\x6F\x6E\x00\x2C\x66\x66\x66\x66\x00\x00\x00\x3F\x6E\x91\x64\xBB\xB7\x21\xF6\xBB\x87\x0F\x59\x3E\xB7\x45";

  
/*
static volatile struct {
    uint16_t     m_getIdx;
    uint16_t     m_putIdx;
    uint8_t      m_entry[ myQ_SIZE ];
} myQ;
*/

typedef struct {
	
   // Output pins
  /* hal_u32_t *misrx_buf_00;
   hal_u32_t *misrx_buf_01;
   hal_u32_t *misrx_buf_02;
   hal_u32_t *misrx_buf_03;
   hal_u32_t *misrx_buf_04;
   hal_u32_t *misrx_buf_05;
   hal_u32_t *misrx_buf_06;
   hal_u32_t *misrx_buf_07;
   
   hal_u32_t *misrx_ring_n;   
   hal_u32_t *misrx_ring_counter;
   hal_u32_t *misrx_ring_old_i; */
   
   hal_u32_t *misrx_ringbuf[RINGBUFCAP/4 + 3];


} hal_testsource_t;

static  hal_testsource_t* testsource_data;	

/* other globals */
static int comp_id;             /* component ID */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static int  testsource_export( hal_testsource_t* addr, const char* name);
static void update(void *arg, long period);

//static inline void *lcec_zalloc(size_t size);
//static inline float ieee_float(const uint32_t* uRep); 

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main( void)
{
    const char name[] = "testsource";

    if (debug) {
        rtapi_print_msg( RTAPI_MSG_ERR, "TESTSOURCE: INFO: testsource %d starting...\n", count);
    }
	
	/* connect to the HAL */
    comp_id = hal_init(name);
    if (comp_id < 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "TESTSOURCE: ERROR: hal_init() failed\n");
        return -1;
    }
	
	 /* allocate shared memory for madgwick data */
    xioparser_data = hal_malloc( sizeof( hal_xioparser_t));
    if (testsource_data == 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "TESTSOURCE: ERROR: hal_malloc(1) failed\n");
        hal_exit( comp_id);
        return -1;
    }
	

	// Initiliye circular buffer
	//CBUF_Init(myQ);
	
	/* export variables and functions */
    int retval = testsource_export( testsource_data, name);

    if (retval != 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "TESTSOURCE: ERROR: testsource var export failed\n");
        hal_exit( comp_id);
        return -1;
    }
    rtapi_print_msg( RTAPI_MSG_INFO, "TESTSOURCE: loaded TESTSOURCE fusion component\n");
    hal_ready( comp_id);
	
	
    return 0;
}


void rtapi_app_exit( void)
{
	hal_exit( comp_id);
}

static void update(void *arg, long period)
{
	static uint16_t k = 0 ;
	
	hal_testsource_t* m_data = arg;
	int counter = *(m_data->misrx_ringbuf[9]);
	int ii = 0;
	for ( ii = k ; ii < counter/4 ; ii++)
		memcpy( m_data->misrx_ringbuf[ii], &bytes[ii*4], 4);
	
	m_data->addr->misrx_ringbuf[10] = k;
	k = ii;
}



static int  testsource_export( hal_testsource_t* addr, const char* prefix)
{
	int retval;
	
	// Input pins:

   /* retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_00), comp_id, "%s.misrx_buf_00", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_01), comp_id, "%s.misrx_buf_01", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_02), comp_id, "%s.misrx_buf_02", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_03), comp_id, "%s.misrx_buf_03", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_04), comp_id, "%s.misrx_buf_04", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_05), comp_id, "%s.misrx_buf_05", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_06), comp_id, "%s.misrx_buf_06", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_buf_07), comp_id, "%s.misrx_buf_07", prefix);
    if (retval != 0) {
        return retval;
    } */
	
	int k;	
	for ( k = 0; k < RINGBUFCAP/4; k++)
	{		
		retval = hal_pin_u32_newf( HAL_OUT, &(addr->misrx_ringbuf[k]), comp_id, "%s.misrx_buf_%02d", prefix, k);
        if (retval != 0) {
            return retval;
        }
	}
	
	retval = hal_pin_u32_newf( HAL_OUT, &(addr->misrx_ringbuf[8]), comp_id, "%s.misrx_ring_n", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IO, &(addr->misrx_ringbuf[9]), comp_id, "%s.misrx_ring_counter", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_OUT, &(addr->misrx_ringbuf[10]), comp_id, "%s.misrx_ring_old_i", prefix);
    if (retval != 0) {
        return retval;
    }

    char bufExp[ HAL_NAME_LEN + 1];
    rtapi_snprintf( bufExp, sizeof( bufExp), "%s.update", prefix);
    retval = hal_export_funct( bufExp, update, addr, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "TESTSOURCE: ERROR: update function export failed\n");
        hal_exit( comp_id);
        return -1;
    }
		
    return 0;
}