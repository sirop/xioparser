#include <stdlib.h>

#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "hal.h"                /* HAL public API decls */


#include "Osc99.h"

//#include "CBUF.h"

#define RINGBUFCAP 32 // incoming ring buffer capacitz in bytes

//#define myQ_SIZE    512


/* module information */
MODULE_AUTHOR("Boris Skegin");
MODULE_DESCRIPTION("Parser for serail OSC output of XIO NGIMU");
MODULE_LICENSE("LGPL");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/


/** This structure contains the runtime data for the component.
*/

static int count = 1;           /* number of instances */
RTAPI_MP_INT( count, "number of xioparser instances");

static int debug = 0;
RTAPI_MP_INT( debug, "debug level");



static OscSlipDecoder oscSlipDecoder;
  
/*
static volatile struct {
    uint16_t     m_getIdx;
    uint16_t     m_putIdx;
    uint8_t      m_entry[ myQ_SIZE ];
} myQ;
*/

typedef struct {
	
   // Input pins
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

   // Output pins
   hal_float_t *quatW;
   hal_float_t *quatX;
   hal_float_t *quatY;
   hal_float_t *quatZ;
   
   // output from euler tag
   hal_float_t *roll;
   hal_float_t *pitch;
   hal_float_t *yaw;
   
   // output from earth tag
   hal_float_t *gX;
   hal_float_t *gY;
   hal_float_t *gZ;

} hal_xioparser_t;

static  hal_xioparser_t* xioparser_data;	

/* other globals */
static int comp_id;             /* component ID */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static int  xioparser_export( hal_xioparser_t* addr, const char* name);
static void update(void *arg, long period);

void ProcessPacket(OscPacket* const oscPacket);
void ProcessMessage(const OscTimeTag* const oscTimeTag, OscMessage* const oscMessage);
static OscError ProcessQuaternion(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessEuler(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessEuler(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);

//static inline void *lcec_zalloc(size_t size);
//static inline float ieee_float(const uint32_t* uRep); 

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main( void)
{
    const char name[] = "xioparser";

    if (debug) {
        rtapi_print_msg( RTAPI_MSG_ERR, "XIOPARSER: INFO: xioparser %d starting...\n", count);
    }
	
	/* connect to the HAL */
    comp_id = hal_init(name);
    if (comp_id < 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "XIOPARSER: ERROR: hal_init() failed\n");
        return -1;
    }
	
	 /* allocate shared memory for madgwick data */
    xiopaser_data = hal_malloc( sizeof( hal_xioparser_t));
    if (xiopaser_data == 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "XIOPARSER: ERROR: hal_malloc(1) failed\n");
        hal_exit( comp_id);
        return -1;
    }
	
	OscSlipDecoderInitialise(&oscSlipDecoder);
    oscSlipDecoder.processPacket = ProcessPacket; // assign callback function
	
	// Initiliye circular buffer
	//CBUF_Init(myQ);
	
	/* export variables and functions */
    int retval = xioparser_export( xioparser_data, name);

    if (retval != 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "XIOPARSER: ERROR: xioparser var export failed\n");
        hal_exit( comp_id);
        return -1;
    }
    rtapi_print_msg( RTAPI_MSG_INFO, "XIOPARSER: loaded XIOPARSER fusion component\n");
    hal_ready( comp_id);
	
	
    return 0;
}


void rtapi_app_exit( void)
{
   
	hal_exit( comp_id);
}

static void update(void *arg, long period)
{
	
	hal_xioparser_t* m_data = arg;
	int count = m_data->misrx_ring_counter;
	
	if ( counter > 0) {
	    unsigned char* bytes = (unsigned char*) (&m_data->misrx_ringbuf[0]);
	    int old_i = m_data->misrx_ring_old_i;
		int k;
		if (old_i + counter < RINGBUFCAP) {
			
			for ( k = old_i ; k <= old_i + counter; k++)
			    OscSlipDecoderProcessByte(&oscSlipDecoder, bytes[k]);			
		}
		esle {
			for ( k = old_i ; k < RINGBUFCAP; k++)
		        OscSlipDecoderProcessByte(&oscSlipDecoder, bytes[k]);
			// Example: ring cap 32, old_i 22, count 10, 
			for ( k = 0 ; k < (old_i+count) - RINGBUFCAP; k++)
		        OscSlipDecoderProcessByte(&oscSlipDecoder, bytes[k]);
	    }
		

	    /*
		for ( k = old_i ; k < RINGBUFCAP % ( old_i + 1 + count) ; k++)
		    OscSlipDecoderProcessByte(&oscSlipDecoder, bytes[k]);
			
		if ( RINGBUFCAP < ( old_i + 1 + count)) {
			int j = ( old_i + 1 + count) - RINGBUFCAP;
			for ( k = 0 ; k < j ; k++)
		        OscSlipDecoderProcessByte(&oscSlipDecoder, bytes[k]);
		}
		*/   
	
	
	}
}

// This function is called for each OSC packet received by the SLIP decoder
void ProcessPacket(OscPacket* const oscPacket) {
  oscPacket->processMessage = &ProcessMessage; // assign callback function
  OscPacketProcessMessages(oscPacket);
}

// This function is called for each OSC message found within the received OSC packet
void ProcessMessage(const OscTimeTag* const oscTimeTag, OscMessage* const oscMessage) {

  // If message address is "/quaternion" then send our hello message
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/quaternion") == true) {
       ProcessQuaternion(oscTimeTag, oscMessage)
       return;
    }

  // If message address is "/euler" then send our hello message
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/euler") == true) {
       ProcessEuler(oscTimeTag, oscMessage)
       return;
    }
	
	// If message address is "/earth" then send our hello message
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/earth") == true) {
       ProcessEarth(oscTimeTag, oscMessage)
       return;
    }

}

/**
 * @brief Process "/quaternion" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessQuaternion(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {

    // Get timestamp
    // NgimuQuaternion ngimuQuaternion;
    //  ngimuQuaternion.timestamp = *oscTimeTag;

    // Get W element
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->w);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get X element
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->x);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Y element
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->y);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Z element
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->z);
    if (oscError != OscErrorNone) {
        return oscError;
    }

   
    return OscErrorNone;
}

/**
 * @brief Process "/euler" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessEuler(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {


    // Get timestamp
    //NgimuEuler ngimuEuler;
   // ngimuEuler.timestamp = *oscTimeTag;

    // Get roll
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->roll);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get pitch
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->pitch);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yaw
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->yaw);
    if (oscError != OscErrorNone) {
        return oscError;
    }

   
    return OscErrorNone;
}

/**
 * @brief Process "/earth" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessEuler(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {


    // Get timestamp
    //NgimuEuler ngimuEuler;
   // ngimuEuler.timestamp = *oscTimeTag;

    // Get roll
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->gX);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get pitch
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->gY);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yaw
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &xioparser_data->gZ);
    if (oscError != OscErrorNone) {
        return oscError;
    }

   
    return OscErrorNone;
}






static int  xioparser_export( hal_xioparser_t* addr, const char* prefix)
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
		retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_ringbuf[k]), comp_id, "%s.misrx_buf_%02d", prefix, k);
        if (retval != 0) {
            return retval;
        }
	}
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_ringbuf[8]), comp_id, "%s.misrx_ring_n", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_ringbuf[9]), comp_id, "%s.misrx_ring_counter", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->misrx_ringbuf[10]), comp_id, "%s.misrx_ring_old_i", prefix);
    if (retval != 0) {
        return retval;
    }
	
	
	
	
	retval = hal_pin_float_newf( HAL_OUT, &(addr->quatW), comp_id, "%s.quatW", prefix);
    if (retval != 0) {
        return retval;
    }
	retval = hal_pin_float_newf( HAL_OUT, &(addr->quatX), comp_id, "%s.quatX", prefix);
    if (retval != 0) {
        return retval;
    }
	retval = hal_pin_float_newf( HAL_OUT, &(addr->quatY), comp_id, "%s.quatY", prefix);
    if (retval != 0) {
        return retval;
    }
	retval = hal_pin_float_newf( HAL_OUT, &(addr->quatZ), comp_id, "%s.quatZ", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_float_newf( HAL_OUT, &(addr->roll), comp_id, "%s.roll", prefix);
    if (retval != 0) {
        return retval;
    }
	retval = hal_pin_float_newf( HAL_OUT, &(addr->pitch), comp_id, "%s.pitch", prefix);
    if (retval != 0) {
        return retval;
    }
	retval = hal_pin_float_newf( HAL_OUT, &(addr->yaw), comp_id, "%s.yaw", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_float_newf( HAL_OUT, &(addr->gX), comp_id, "%s.gX", prefix);
    if (retval != 0) {
        return retval;
    }
	retval = hal_pin_float_newf( HAL_OUT, &(addr->gY), comp_id, "%s.gY", prefix);
    if (retval != 0) {
        return retval;
    }
	retval = hal_pin_float_newf( HAL_OUT, &(addr->gZ), comp_id, "%s.gZ", prefix);
    if (retval != 0) {
        return retval;
    }
	
	char bufExp[ HAL_NAME_LEN + 1];
    rtapi_snprintf( bufExp, sizeof( bufExp), "%s.update", prefix);
    retval = hal_export_funct( bufExp, update, addr, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "XIOPARSER: ERROR: update function export failed\n");
        hal_exit( comp_id);
        return -1;
    }
		
    return 0;
}