//CURVE CABLE CAM MODE 
/*

	 			GCS端  									copter端

$$set points:
	1.	ACTION_REQUEST(ACTION_REQUEST_SET)		==>			//wp data reset				
							 					<==			ACTION_RESPONSE

	2.	ACTIONSETTING_CCC    	  				==>			//add wp
								 				<==			ACTION_RESPONSE
					

	3.	ACTIONSETTING_CCC 						==>			//add wp
							 					<==			ACTION_RESPONSE
					

												.
												.
												.
	
	n. 	ACTIONSETTING_CCC 						==>			//add wp
							 					<==			ACTION_RESPONSE
					


$$check:
	 			GCS端  									copter端
	1.	ACTION_REQUEST(ACTION_REQUEST_CHECK) 	==>
							 
							    				<==			ACTIONSETTING_CCC//only send setting data when ACTION_RESPONSE is ok

	2.	ACTION_REQUEST(ACTION_REQUEST_CHECK) 	==>
							 
							    							ACTIONSETTING_CCC//only send setting data when ACTION_RESPONSE is ok
	
	3.	ACTION_REQUEST(ACTION_REQUEST_CHECK) 	==>
							 
							    							ACTIONSETTING_CCC//only send setting data when ACTION_RESPONSE is ok

												.
												.
												.
	n.	ACTION_REQUEST(ACTION_REQUEST_CHECK) 	==>
							 
							    				<==			ACTIONSETTING_CCC//only send setting data when ACTION_RESPONSE is ok
	

$$start
	n+1.ACTION_REQUEST(ACTION_REQUEST_START)	==>			//do mirror and start
							 
**********************************************************************************
$$get point
1.	ACTION_REQUEST(ACTION_REQUEST_GET) 			==>
							 
							    				<==			ACTIONSETTING_CCC//only send setting data when ACTION_RESPONSE is ok
**********************************************************************************
once action state changed ,send feedback
1.												<==			ACTION_FEEDBACK 
	ACTION_RESPONSE	    						==>			
*/

#define YPRC_DATA_LEN_MAX 128

typedef enum {
	YPRC_PACKET_TYPE_CHANNELDATA12,
	YPRC_PACKET_TYPE_CHANNELDATA24,
	YPRC_PACKET_TYPE_TELEMETRYDATA,
	YPRC_PACKET_TYPE_CHANNELDATA12_GPS,
	YPRC_PACKET_TYPE_BINDCMD,
	YPRC_PACKET_TYPE_ACTION = 0x14 //all action packet are taged by this 
} YPRC_PACKET_TYPE;

typedef enum {
	ACTION_TYPE_REQUEST, 
	ACTION_TYPE_RESPONSE,
	ACTION_TYPE_FEEDBACK,
	ACTION_TYPE_SETTING_CCC,
	ACTION_TYPE_NUMELEM
} ACTION_TYPE;

enum Mode_Type {
	AUTO_NONES,
	AUTO_JOUR,                      //journey mode
	AUTO_ROI,						//Hot piont mode
	AUTO_CURVECABLECAM,				//CCC mode
	AUTO_CRUISE,				    //curise mode
	AUTO_WAYPOINT 	 				//for wizard
}Mode_Type;

enum Action_Request {
	ACTION_REQUEST_START_AUTO,			
	ACTION_REQUEST_START_MANUAL ,
	ACTION_REQUEST_PAUSE ,
	ACTION_REQUEST_RESUME,					
	ACTION_REQUEST_EXIT,
	ACTION_REQUEST_GET,					//ask to return position to GCS
	ACTION_REQUEST_SET,					//sending wp data to MAV 
	ACTION_REQUEST_CHECK				//ask to return wp to GCS for check
}Action_Request;

enum Action_Result{
	ACTION_RESULT_OK,
	ACTION_RESULT_ERR_FAIL, 				// Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. 
	ACTION_RESULT_ERR_ACCESS_DENIED, 		// The system is refusing to accept this setting from this source / communication partner. 
	ACTION_RESULT_ERR_NOT_SUPPORTED, 		// setting is not supported, other setting would be accepted. 
	ACTION_RESULT_ERR_COORDINATE_VALID 		// The coordinate frame of this setting is not supported, or out of range. 
}Action_Result;

enum Action_State{
	ACTION_STATUS_ACQUIRED,				//means acquired the request is running now
	ACTION_STATUS_SUCCEED, 				//means the action request is finished 
	ACTION_STATUS_ABORTED, 				//means the action request is aborted for some reason 
}Action_State;

// Action Command for copter
//包括模式类型，动作类型，如果是ccc模式还需发送设置的数量（setting_count）

typedef struct {
	uint8_t  action_type;		//from enum Action_Type// the first byte is used to identify Action_Type
	uint8_t  action_request;	//from enom Action_Type
	uint8_t  mode_type;			//from enum Mode_Type
	uint8_t  setting_count;		//default:0 how many settingbags are followed
}ActionRequest;

typedef struct {
	uint8_t  action_type;		//from enum Action_Type// the first byte is used to identify Action_Type
	uint8_t  action_result;		//from enom Action_State
	uint8_t  mode_type;			//from enum Mode_Type
	uint8_t  setting_count;		//default:0 how many settingbags are followed
}ActionResponse;

typedef struct {
	uint8_t  action_type;		//from enum Action_Type// the first byte is used to identify Action_Type
	uint8_t  action_state;		//
	uint8_t  mode_type;
	uint8_t  degree_of_completion;
}ActionFeedback;



typedef struct {
	uint8_t  action_type;		//from enum Action_Type// the first byte is used to identify Action_Type
	uint8_t  action_result;	
	uint8_t  action_index;				// index of waypoint action 
	int32_t  lat;						// lattitude (degrees)	+/- 90 deg
	int32_t  lon;						// longitude (degrees)	+/- 180 deg
	int32_t  alt;						// 0.01m resolution, altitude (meters)
	int16_t  roll, pitch, yaw;			// 0.01 degree resolution, copter's angle
	int16_t  gimbal_pitch,gimbal_yaw;	// gimbal's angle
} ActionSettingCCC;



//Ack form copter, for ACTIONCOMMAND
// typedef struct {
// 	uint8_t  mode_type;
// 	uint8_t  ack_status;
// 	uint8_t  setting_count;//default: 0;
// } ActionAck;

// typedef struct {
// 	uint8_t  request_type;
// 	uint8_t  action_index;//default: 0;

// } RequestInfo;

// enum RequestType{
// 	CHECKING_REQUEST = 0,// check all the setting in copter
// 	ASKING_REQUEST = 1//ask the copter's status now
// };

// enum RequestType request_type;


// enum AckStatus{
// 	STATUS_OK = 0,
// 	STATUS_ERR_FAIL = 1, // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. 
// 	STATUS_ERR_ACCESS_DENIED = 2, // The system is refusing to accept this setting from this source / communication partner. 
// 	STATUS_ERR_NOT_SUPPORTED = 3, // setting is not supported, other setting would be accepted. 
// 	STATUS_ERR_COORDINATE_VALID = 4 // The coordinate frame of this setting is not supported, or out of range. 
// };


// enum YawMode{
// 	AUTO_YAW_NONE =             0,
//     AUTO_YAW_HOLD =             1,  // pilot controls the heading
//     AUTO_YAW_LOOK_AT_NEXT_WP =  2,  // point towards next waypoint (no pilot input accepted)
//     AUTO_YAW_ROI =              3,  // point towards a location held in roi_WP (no pilot input accepted)
//     AUTO_YAW_LOOK_AT_HEADING =  4,  // point towards a particular angle (not pilot input accepted)
//     AUTO_YAW_LOOK_AHEAD =       5,  // point in the direction the copter is moving
//     AUTO_YAW_RESETTOARMEDYAW =  6,  // point towards heading at time motors were armed
// 	AUTO_YAW_LOOK_AT_ME = 		7,	// point towards pilot
//     AUTO_YAW_LOOK_REPLAY =      8,  // point reply last record 
// }

// typedef struct 
// {
// 	uint8_t yaw_mode;
// }YAWCONTROL;

typedef struct {
	uint16_t t;					// packet counter or clock
	int32_t	lat;				// lattitude (degrees)	+/- 90 deg
	int32_t	lon;				// longitude (degrees)	+/- 180 deg
	int32_t	alt;				// 0.01m resolution, altitude (meters)
	int16_t	vx, vy, vz; 		// velocity 0.01m res, +/-320.00 North-East-Down
	uint8_t	nsat;				// number of satellites; 7th bit notify GPS is used
	uint8_t	voltage; 			// 25.4V	voltage = 5 + 255*0.1 = 30.5V, min=5V
	uint8_t	current; 			// 0.5A resolution
	int16_t	roll, pitch, yaw;	// 0.01 degree resolution
	uint8_t	motorStatus;		// 1 bit per motor for status 1=good, 0= fail
	uint8_t	imuStatus;			// inertial measurement unit status
	uint8_t	baroMagGpsStatus;	// baro/mag/gps status
	uint8_t flightMode;         // vehicle flight mode
	uint8_t vehicleType;   		// vehicle type and additional warning flags
	uint8_t errorFlags;   		// warning flags
	uint8_t gpsAccH;       		// gps accuracy in 0.05m resolution
	
	//int16_t gimbal_pitch,gimbal_yaw;
	//uint8_t mode_type;
	uint8_t avoidence_open;
	//uint8_t yaw_mode;
} TelemetryData;


