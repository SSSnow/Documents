
typedef enum {
	ACTION_TYPE_MAP_WAYPOINT 		= 8,
	ACTION_TYPE_FENCE 				= 9,
	ACTION_TYPE_LANDING 			= 10,
	ACTION_TYPE_NUMELEM	
}ActionType;

typedef enum {
	ACTION_REQUEST_START    	 	= 0,
	ACTION_REQUEST_PAUSE         	= 1,
	ACTION_REQUEST_RESUME        	= 2,					
	ACTION_REQUEST_EXIT          	= 3,
	ACTION_REQUEST_GET           	= 4,		//ask to return position to GCS
	ACTION_REQUEST_SET           	= 5,		//sending wp data to MAV, in a list way
	ACTION_REQUEST_CONFIG        	= 6,		//Configuration
}ActionRequest;

typedef enum { 
	ACTION_TYPE_RESPONSE 			= 1,
	ACTION_TYPE_FEEDBACK 			= 2,
}ActionResponseType;

typedef struct {
	ActionResponseType action_response_type = ACTION_TYPE_RESPONSE;
	ActionType action_type;
	ActionRequest action_request;
	ActionResult action_result;
}Response;


typedef enum {
	AIRLINE_TYPE_STRAIGHT 			= 0,
	AIRLINE_TYPE_CURVE 				= 1
}ArilineType;

typedef enum {
	GIMBAL_ORIENTATION_FOLLOW_WAYPOINT 	= 0,
	GIMBAL_ORIENTATION_FORWARD_AIRLINE 	= 1,
	GIMBAL_ORIENTATION_FREEDOM		   	= 2
}GimbalOrientation;

typedef enum {
	AIRLINE_ENDING_RETURN_START 		= 0,
	AIRLINE_ENDING_LANDING				= 1,
	AIRLINE_ENDING_SUSPEND				= 2,
	AIRLINE_ENDING_CYCLE				= 3,
}AirlineEndingAction;

typedef struct {
	uint_8 action_type 		= ACTION_TYPE_MAP_WAYPOINT; 
	uint_8 action_request 	= ACTION_REQUEST_CONFIG;
	uint_8 waypointCount;
	ArilineType ariline_type;
	GimbalOrientation gimbal_orientation;
	AirlineEndingAction airline_ending_action;
}MapConfigRequest;


typedef enum {
	ALTITUDE_TYPE_RELATIVE 				= 0,
	ALTITUDE_TYPE_ABSOLUTE 				= 1
}AltitudeType;

typedef enum {
	WAYPOINT_ACTION_NONE				= 0,
	WAYPOINT_ACTION_PHOTO				= 1,
	WAYPOINT_ACTION_THROW_OUT			= 2,
	WAYPOINT_ACTION_ORBIT				= 3,
}WaypointAction;

typedef struct {
	uint16_t radius                    //0.1m resolution, the radius of the orbit cycle. 
}WaypointActionParamOrbit

typedef struct {
	uint8_t  waypointIndex;				// index of waypoint 
	int32_t  lat;						// lattitude (degrees)	+/- 90 deg
	int32_t  lon;						// longitude (degrees)	+/- 180 deg
	int32_t  alt;						// 0.01m resolution, altitude (meters)
	int16_t  gimbal_pitch,gimbal_yaw;	// gimbal's angle
	uint8_t  altitudeType               // the altitude type: absolute or relative 
	uint8_t  speed                      // 0.1m resolution, the copter speed on the waypoint.
	uint8_t  waitTime                   // 1 second resolution, wait time after the copter arrived the waypoint.
	uint8_t  waypointAction 			// the action when arrived the waypoint.
	uint8_t  waypointActionParam[8]     // the parameters of action
} MapWaypoint;

typedef struct {
	uint_8 action_type 		= ACTION_TYPE_MAP_WAYPOINT;
	uint_8 action_request 	= ACTION_REQUEST_SET;
	MapWaypoint map_waypoint;
} MapSetWaypointRequest;

typedef struct {
	Response response;
	uint_8 waypointIndex;    			// the waypoint index of receive. 
} MapSetWaypointResponse;

typedef struct {
	uint_8 action_type 		= ACTION_TYPE_MAP_WAYPOINT;
	ActionRequest action_request;
} MapWaypointRequest;


typedef struct {
	uint_8 action_type 		= ACTION_TYPE_FENCE;
	uint_8 action_request 	= ACTION_REQUEST_CONFIG;
	uint_8 waypointCount;
} FencConfigRequest;


typedef struct {
	uint8_t  waypointIndex;				// index of waypoint 
	int32_t  lat;						// lattitude (degrees)	+/- 90 deg
	int32_t  lon;						// longitude (degrees)	+/- 180 deg
	int32_t  alt;						// 0.01m resolution, altitude (meters)
	uint8_t  altitudeType               // the altitude type: absolute or relative 
} FenceWaypoint;

typedef struct {
	uint_8 action_type 		= ACTION_TYPE_FENCE;
	uint_8 action_request 	= ACTION_REQUEST_SET;
	FenceWaypoint fence_waypoint;
} FenceSetWaypointRequest;

typedef struct {
	Response response;
	uint_8 waypointIndex;				// the waypoint index of receive. 
} FenceSetWaypointResponse;


typedef struct {
	uint_8 action_type 		= ACTION_TYPE_FENCE;
	ActionRequest action_request;
} FenceRequest;


typedef struct {
	uint_8 action_type 		= ACTION_TYPE_LANDING;
	uint_8 action_request 	= ACTION_REQUEST_START;
	int32_t  lat;						// lattitude (degrees)	+/- 90 deg
	int32_t  lon;						// longitude (degrees)	+/- 180 deg
} LandingRequest;



/**

map waypoint setup:
		GCS																Aircraft
1, 	<MapConfigRequest>		  									===>	check and reply
							  									<===	<Response>


2, <MapSetWaypointRequest>    									===>    check and reply
							  									<===    <MapSetWaypointResponse>


							  									....

n, <MapWaypointRequest>(action_request=ACTION_REQUEST_START)    ===> 	check and reply (start mission)
																<===    <Response>


fence setup :
		GCS																Aircraft
1,   <FencConfigRequest>										===>    check and reply
																<===    <Response>


2, <FenceSetWaypointRequest>    								===>    check and reply
							  									<===    <FenceSetWaypointResponse>


							  									....

n, <FenceRequest>(action_request=ACTION_REQUEST_START)    		===> 	check and reply (start mission)
																<===    <Response>

landing setup :

		GCS																Aircraft
1,   <LandingRequest>											===>    check and reply
																<===    <Response>


Note:  Aircraft will check the request parameter and the situation of itself to determine if will goto next step,
if aircraft terminate the setup process, it need send the response to GCS with the reason.

if GCS want to terminate the process, it would to send the request with "ActionRequest=ACTION_REQUEST_EXIT".
Aircraft will terminate the process and clear all settings about the process immediatly after receiving the request.

*/
