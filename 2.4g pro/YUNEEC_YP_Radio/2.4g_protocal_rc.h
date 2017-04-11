#ifndef __YPRC_H__
#define __YPRC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <YUNEEC_Mission_WayPoint.h>

#define MAX_YPRC_PORT_NUM	2

#define YPRC_DATA_LEN_MAX	128
#define YPRC_LENGTH_VALUE	130
#define YPRC_STX1			0x55
#define YPRC_STX2			0x55

/* define range mapping here, -+100% -> 1000..2000 */
#define YPRC_RANGE_MIN 0.0f
#define YPRC_RANGE_MAX 4096.0f

#define YPRC_TARGET_MIN 1000.0f
#define YPRC_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define YPRC_SCALE_FACTOR ((YPRC_TARGET_MAX - YPRC_TARGET_MIN) / (YPRC_RANGE_MAX - YPRC_RANGE_MIN))
#define YPRC_SCALE_OFFSET (int)(YPRC_TARGET_MIN - (YPRC_SCALE_FACTOR * YPRC_RANGE_MIN + 0.5f))

#define _YP_PACKET_DATA(msg) ( (const char *)( &((msg)->yprc_data[0]) ) )
#define _YP_PACKET_DATA_NON_CONST(msg) ( (char *)( &((msg).yprc_data[0]) ) )
#define _YP_PACKET_DATA_LOCATE(msg, index) ( (char *)( &(msg.yprc_data[0]) + index) )

typedef enum {
	YPRC_DECODE_STATE_UNSYNCED = 0,
	YPRC_DECODE_STATE_GOT_STX1,
	YPRC_DECODE_STATE_GOT_STX2,
	YPRC_DECODE_STATE_GOT_LEN,
	YPRC_DECODE_STATE_GOT_TYPE,
	YPRC_DECODE_STATE_GOT_DATA
} YPRC_DECODE_STATE;

typedef	enum {
	SYNC_SUCCESS = 0,
	SYNC_RUNNING = 1,
	SYNC_UNSYNC = 2,
	SYNC_CRC_ERR = 3
} YP_Sync_Status;

static YPRC_DECODE_STATE _decode_state = YPRC_DECODE_STATE_UNSYNCED;
static unsigned _rxlen;

typedef enum {
	FMODE_THR_MODE = 0,					// not available now
	FMODE_THR_MODE_GPS_OFF = 1,			// not available now
	FMODE_THR_MODE_NO_GPS = 2,			// not available now
	FMODE_ANGLE_MODE = 3,				// angle mode with GPS 3D locked
	FMODE_ANGLE_MODE_GPS_OFF = 4,		// gps off mode(althold mode in fact), always send this value if user switch off GPS
	FMODE_ANGLE_MODE_NO_GPS = 5,		// angle mode without GPS 3D locked
	FMODE_SMART_MODE = 6,				// smart mode with GPS 3D locked
	FMODE_SMART_MODE_NO_GPS = 7,		// smart mode without GPS 3D locked
	FMODE_ARMING_MOTORS = 8,			// indicate we are arming the motors
	FMODE_IMU_TEMP_CALIBRATION = 9,		// not available now - imu temperature calibration
	FMODE_BARO_CALIBRATION = 10,		// not available now - barometer calibration
	FMODE_IMU_ABIAS_CALIBRATION = 11,	// imu accel bias calibration
	FMODE_EMERGENCY_SHUTDOWN = 12,		// indicate we crashed
	FMODE_RETURN_TO_HOME = 13,			// return to home
	FMODE_RTH_FINAL_LANDING = 14,		// rth final landing
	FMODE_BINDING = 15,					// we are binding
	FMODE_READY_TO_FLY = 16,			// all sensors are healthy, we are ready to fly
	FMODE_WAITING_FOR_RC = 17,			// no use
	FMODE_MAG_CALIBRATION = 18,			// magnetometer calibration
	FMODE_UNKNOWN = 19,					// unknown flight mode
	FMODE_ACRO_MODE = 20,				// acrobat mode
	FMODE_FOLLOW_MODE = 21,				// follow me mode with both copter and rc GPS 3D locked
	FMODE_FOLLOW_MODE_NO_RC_GPS = 22,	// follow me mode without rc GPS 3D locked
	FMODE_WATCHME_MODE = 23,			// camera tracking mode
	FMODE_WATCHME_MODE_NO_RC_GPS = 24,	// camera tracking mode without rc GPS 3D locked
	FMODE_GUIDE_MODE = 25,				// guide mode
	FMODE_CCC_MODE = 26,				// CCC mode
	FMODE_JOURNEY_MODE = 27,			// Journey mode
	FMODE_POI_MODE = 28,				// POI mode
	FMODE_ORBIT_MODE = 29,				// Orbit mode
	FMODE_3D_FOLLOW_WIZARD = 30,		// Wizard 3D follow
	FMODE_3D_WATCHME_WIZARD = 31,		// Wizard 3D watch me
	FMODE_ANGLE_MODE_IPS_ONLY = 32,		// angle mode with IPS only
	FMODE_WAYPOINT = 33,				// waypoint mode
} YPRC_FMODE_TYPE;

typedef enum {
	VEHICLE_NONE 		= 0,
	VEHICLE_TORNADO 	= 1,
	VEHICLE_Q500 		= 2,
	VEHICLE_350QX 		= 3,
	VEHICLE_CHROMA 		= 4,
	VEHICLE_TYPHOON_H	= 5,
	VEHICLE_TORNADO_W   = 6,
} YPRC_VEHICLE_TYPE;

#pragma pack(push,1)

typedef struct {
	uint8_t	header1;			///< 0x55 for a valid packet
	uint8_t	header2;			///< 0x55 for a valid packet
	uint8_t	length;				///< length includes type, data, and crc = sizeof(type)+sizeof(data[payload_len])+sizeof(crc8)
	uint8_t	type;				///< from enum YPRC_PACKET_TYPE
	uint8_t	yprc_data[YPRC_DATA_LEN_MAX];
	uint8_t	crc8;				///< crc8 checksum, calculated by yprc_crc8 and including fields length, type and yprc_data
} ReceiverFcPacket;


typedef enum {
	YPRC_PACKET_TYPE_CHANNELDATA12 = 0,
	YPRC_PACKET_TYPE_CHANNELDATA24,
	YPRC_PACKET_TYPE_TELEMETRYDATA,
	YPRC_PACKET_TYPE_CHANNELDATA12_GPS,
	YPRC_PACKET_TYPE_BINDCMD,
	YPRC_PACKET_TYPE_OBSTACLE_HEARTBEAT			= 0x0F,
	YPRC_PACKET_TYPE_OBSTACLE_DATA_INPUT		= 0x10,
	YPRC_PACKET_TYPE_OBSTACLE_DATA_OUTPUT		= 0x11,
	YPRC_PACKET_TYPE_OPTFLOW_DATA_INPUT			= 0x12,
	YPRC_PACKET_TYPE_OBSTACLE_DISTANCE_INFO		= 0x13,
	YPRC_PACKET_TYPE_ACTION 					= 0x14, // all action packet share this ID
	YPRC_PACKET_TYPE_OBSTACLE_DISTANCE_360		= 0x15,
	YPRC_PACKET_TYPE_CHT_COMMAND				= 0x16,
} YPRC_PACKET_TYPE;

/**
 * RC Channel data (12 channels).
 *
 * This is incoming from the YPRC
 *
 * Caution: YPRC receiver is different from DSM or other RC
 */
typedef struct {
	uint16_t t;				///< packet counter or clock
	uint8_t	rssi;			///< signal strength
	uint8_t	lost_count;		///< how much time passed if lost RF frame, lost_time = lost_count * 20ms
	uint8_t	channel[18];	///< channel data, 12 channels (12 bit numbers)
} ChannelData12;

/**
 * RC Channel data (24 channels).
 *
 */
typedef struct {
	uint16_t t;				///< packet counter or clock
	uint8_t	rssi;			///< signal strength
	uint8_t	lost_count;		///< how much time passed if lost RF frame, lost_time = lost_count * 20ms
	uint8_t	channel[36];	///< channel data, 24 channels (12 bit numbers)
} ChannelData24;

/**
 * Telemetry packet
 *
 * This is outgoing to the YPRC
 *
 * imuStatus:
 * 0 mpu6050 in FC0 init OK
 * 1 mpu6050 in IMU init OK
 * 2 accelerometer in IMU init OK
 * 3 mpu6050 in FC0 warning
 * 4 mpu6050 in IMU warning
 * 5 accelerometer in IMU warning
 * 6 FC0 and IMU data mismatch warning flag
 * 7 unused
 *
 * For Cyclone/Q500/380QX
 * imuStatus:
 * 0 mpu6050 in FC0 init OK
 * 1 mpu6050 in IMU init OK
 * 2 accelerometer in IMU init OK
 * 3 mpu6050 in FC0 warning
 * 4 mpu6050 in IMU warning
 * 5 set to 1 if GPS is enabled, 0 if GPS is switched off
 * 6 set to 1 if trying to use RC GPS data, 0 if not
 * 7 set to 1 if RC GPS data is good and ready to use, 0 if not
 *
 * baroMagGpsStatus bits
 * 0 pressure in FC0 OK
 * 1 pressure in IMU OK
 * 2 compass1 in FC0 OK
 * 3 compass2 in IMU OK
 * 4 GPS1 in FC0 OK
 * 5 GPS2 in IMU OK/ REALSENSE OK
 * 6 SONAR OK
 * 7 OPTFLOW OK
 *
 * errorFlags bits
 * 0 voltage warning 1
 * 1 voltage warning 2
 * 2 bit for motor failsafe mode
 * 3 complete motor/esc failure
 * 4 high temperature warning (in FC0 or ESC)/ IMU Pre-heating warning
 * 5 compass calibration warning
 * 6 flyaway checker warning
 * 7 NFZ warning
 *
 * fMode byte
 * 8th bit for GPS position switch
 * first 7 to define the flight modes
 *
 * vehicleType bits
 * 0-3 bits for vehicle type (0-15)
 * 4 unused (warning/status)
 * 5 unused (warning/status)
 * 6 unused (warning/status)
 * 7 unused (warning/status)
 *
 */
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
} TelemetryData;

typedef struct {
	uint16_t feedback_software_version;
	uint16_t feedback_hardware_version;
	uint16_t feedback_software_time_year;
	uint8_t	 feedback_software_time_month;
	uint8_t	 feedback_software_time_day;
	uint8_t  feedback_software_type;
	char	 feedback_product_name[16];
} feedback_request_info_struct;

/**
 * RC Channel data (12 channels) with GPS data input
 *
 * This is incoming from the YPRC
 *
 */
typedef struct {
	int32_t  lat;				// need to be divided by 10^7 to get the actual latitude
	int32_t  lon;  				// need to be divided by 10^7 to get the actual longitude
	float 	 alt;				// absolute altitude in mm
	uint16_t hacc;				// horizontal accuracy in mm
	int16_t  speed; 			// ground speed in mm/s
	int16_t  course;			// ground course in -18000 ~ 18000 centi-degrees
	uint8_t  fixmode_satellite; // The lowest 5 bits of the fixmode_satellite is for number of the satellite(GPS in the YPRC), and the highest 3 bits are reserved for fix mode.
} StGpsDataType;

typedef struct {
	uint16_t t;          		// packet counter or clock
	uint8_t rssi;				// signal strength
	uint8_t lost_count;			// how much time passed if lost RF frame, lost_time = lost_count * 20ms
	uint8_t channel[18]; 		// 12 channel data (compressed)
	StGpsDataType gps_data;		// RC GPS data
} ChannelData12_Gps;

/**
 * StBindCmd packet
 *
 * This is a command to set YPRC in BIND mode. After receiving the bind command,
 * the satellite will go to bind mode, and it will quit bind mode automatically
 * if it is bind successfully.
 *
 */
typedef struct {
	uint16_t t;			// packet counter or clock
	uint8_t cmd[4];		// command is "BIND"
} StBindCmd;

/**************************************************************************
 * ACTION packet definition
 **************************************************************************/
typedef enum {
	ACTION_TYPE_REQUEST = 0, 
	ACTION_TYPE_RESPONSE = 1,
	ACTION_TYPE_FEEDBACK = 2,
	ACTION_TYPE_SETTING_CCC = 3,
	ACTION_TYPE_SETTING_ROI = 4,
	ACTION_TYPE_CA_CONFIG = 5,
	ACTION_TYPE_ONEKEY_TAKEOFF = 6,
	ACTION_TYPE_SETTING_JOUR = 7,
	ACTION_TYPE_FRONT_DISTANCE = 8,
	ACTION_TYPE_LED_CONFIG = 9,
	ACTION_TYPE_ASK_FOR_CONFIG = 10,
	ACTION_TYPE_GOHOME_CONFIG = 11,
	ACTION_TYPE_MAP_WAYPOINT = 12,
	ACTION_TYPE_FENCE = 13,
	ACTION_TYPE_LANDING = 14,
	ACTION_TYPE_TAKEOFF	= 15,
	ACTION_TYPE_NONE = 255,
	ACTION_TYPE_NUMELEM
} ActionType_e;

typedef enum {
	AUTO_NONES          = 0,
	AUTO_JOUR  			= 1,                      // Journey mode
	AUTO_POI  			= 2,						// Point Of Interest mode
	AUTO_CURVECABLECAM	= 3,				// CCC mode
	AUTO_ORBIT 			= 4,				    	// Orbit mode
	AUTO_WAYPOINT 		= 5, 	 				// for wizard
	AUTO_MAP_WAYPOINT 	= 6,				// for gcs waypoint from map
	MAP_FENCE 			= 7,						// for fence from map
} ModeType_e;

typedef enum {
	ACTION_REQUEST_START_AUTO    = 0,			
	ACTION_REQUEST_START_MANUAL  = 1,
	ACTION_REQUEST_PAUSE         = 2,
	ACTION_REQUEST_RESUME        = 3,					
	ACTION_REQUEST_EXIT          = 4,
	ACTION_REQUEST_GET           = 5,		// ask to return position to GCS
	ACTION_REQUEST_SET           = 6,		// sending wp data to MAV, in a list way
	ACTION_REQUEST_CHECK		 = 7, 		// ask to return wp to GCS for check
	ACTION_REQUEST_CONFIG        = 8,		// Configuration
} ActionRequest_e;

typedef enum {
	ACTION_REQUEST_START_PHOTO   	= 0,
	ACTION_REQUEST_START_SHOOTING	= 1,
} CameraActionRequest;

typedef enum {
	ACTION_RESULT_OK = 0,
	ACTION_RESULT_ERR_FAIL, 				// Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. 
	ACTION_RESULT_ERR_ACCESS_OUTRANGE, 		// restricted area
	ACTION_RESULT_ERR_COORDINATE_TOOCLOSE, 	// aircraft is too close
	ACTION_RESULT_ERR_COORDINATE_DANGER, 	// Dangerous path
	ACTION_RESULT_ERR_NO_GIMBAL,
	ACTION_RESULT_ERR_NOFLYZONE,
	ACTION_RESULT_ERR_LOWBATTERY
} ActionResult_e;

typedef enum {
	map_waypoint_set_type_none = 0,
	map_waypoint_set_type_yp,
	map_waypoint_set_type_wifi
} Map_waypoint_set_type;

typedef enum {
	ACTION_STATUS_IDLE = 0,
	ACTION_STATUS_SETTING,		// the copter is in setting process(e.g roi or ccc)
	ACTION_STATUS_RUNNING,		// the action is running
	ACTION_STATUS_STOPPED, 		// the action request is stopped by request or other reason
	ACTION_STATUS_EXIT 			// the action request is finished or be ordered to quit by some reason
} Action_State;

typedef struct {
	uint8_t  actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t  actionRequest;	    // from enum ActionType_e
	uint8_t  modeType;			// from enum ModeType_e
	uint8_t  settingCount;		// default:0 how many settingbags are followed, if in jour mode, this is for distance unit m
} ActionRequest;

typedef struct {
	uint8_t  actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t  actionResult;		// from enom Action_State
	uint8_t  modeType;			// from enum ModeType_e, if no mode(e.g sonar), the default is 0
	uint8_t  settingCount;		// default:0 how many settingbags are followed
} ActionResponse;

typedef struct {
	uint8_t  actionType;				// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t  actionResult;
	uint8_t  actionIndex;				// index of waypoint action 
	int32_t  lat;						// lattitude (degrees)	+/- 90 deg
	int32_t  lon;						// longitude (degrees)	+/- 180 deg
	int32_t  alt;						// 0.01m resolution, altitude (meters)
	int16_t  roll, pitch, yaw;			// 0.01 degree resolution, copter's angle
	int16_t  gimbal_pitch, gimbal_yaw;	// gimbal's angle
} ActionSettingCCC;

typedef struct {
	uint8_t  actionType;				// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t  actionResult;
	int32_t  lat;						// lattitude (degrees)	+/- 90 deg
	int32_t  lon;						// longitude (degrees)	+/- 180 deg
	int32_t  alt;						// 0.01m resolution, altitude (meters)
	uint8_t  radius;
	uint8_t  speed;						// default: 0, if there is no speed config ,do in default way
} ActionSettingROI;

typedef struct {
	uint8_t actionType;
	uint8_t actionResult;
	uint8_t maxSpeed;                      	// 2-10m/s default :7m 
	uint8_t maxDistance;					// 20-100m default 30m
} ActionSettingJOUR;

typedef struct {
	uint8_t  actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t  action_state;
	uint8_t  mode_type;
	uint8_t  degree_of_completion;
} ActionFeedback;

typedef struct {
	uint8_t actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t takeoffSpeed;
	uint8_t takeoffHeight;
} ActionTakeOff;

typedef struct {
	uint8_t  actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t  CA_on;				// 0 for off, 1 for on
} ActionCAConfig;

typedef struct {
	uint8_t  actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t  ledStatus;			// Motor LED status, start from the head left arm, CCW. 1: On, 0: Off
} ActionLEDConfig;

typedef struct {
	uint8_t actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint8_t whichConfig;		// send back which configuration, this value should be the actionType for this configuratoin.
} ActionAskForConfig;

typedef struct {
	uint8_t  actionType;		// from enum ActionType_e, the first byte is used to identify ActionType_e
	uint16_t goHomeHeight;		// config the default minimum height for GoHome mode; unit: cm; range from 200cm to 12200cm
	uint16_t goHomeConfig;		// reserved for other configuration for GoHome mode, this's going to be bit-wise
} ActionGoHomeConfig;

/**
 * Distance of obstacles in front of the camera. Units in 10cm, starting -30 deg from the left, in 10 deg increments to the right.
 * Expressed in camera coordinates. It is the slice of zero elevation Angle seen from the camera. A value of 0 means no obstacles.
 */
#define REALSENSE_OBSTACLE_DISTANCE_INFO_SIZE	6

typedef struct {
	uint8_t actionType;
	uint8_t front_distance[REALSENSE_OBSTACLE_DISTANCE_INFO_SIZE];
} ActionFrontDistance; 	// send from the autopilot to the ground station with 5Hz.


/**************************************************************************
 * RealSense packet definition
 **************************************************************************/
// DataType Definition
union __attribute__((packed)) QUATERNION
{
       struct { float q1, q2, q3, q4; };
       struct { float w, x, y, z; };
       float elem[4];
};
typedef union QUATERNION realsense_quaternion;
 
union __attribute__((packed)) VECT3F{
       struct { float x,y,z; };
       float elem[3];
};
typedef union VECT3F vector3f;

// Heartbeat Package
// Version Informartion
// If code fixing is necessary (“API breakage”) count major version up
// If only recompiling of code is necessary (“ABI breakage”) count minor version up.
#define FC_CHT_COMM_VERSION_MAJOR			1	// Increase for API breakage, i.e. code needs to be rewritten
#define FC_CHT_COMM_VERSION_MINOR			0	// Increase for ABI breakage, i.e. code needs to be recompiled

typedef struct __attribute__((packed))
{
  uint8_t 	versionMajor;
  uint8_t 	versionMinor;
  uint32_t 	componentType;
} HEARTBEAT;

enum ComponentType {
	FLIGHT_CONTROLLER	= 0x01,
	COLLISION_AVOIDANCE	= 0x02,
	OPTICAL_FLOW 		= 0x03,
	SYSTEM_MONITOR		= 0x04,
};

enum ObstacleAvoidanceOutputFlags {
	CAMERA_RUNNING 	= 0x01,	///< Camera started and producing frames
	ACTIVE 			= 0x04,	///< Obstacle avoidance algorithm running
	QUALITY_0		= 0x08,
	QUALITY_1		= 0x10,
	devel_FRONTSONAR_ALWAYS_ON = 0x80000000, ///< Asks to turn the front sonar always on. For development only.
};

typedef struct __attribute((packed))
{
	vector3f desiredSpeed;			// m/s
	float desiredHeading;			// radians
	realsense_quaternion attitude; 	// wxyz format
	vector3f position; 				// position in relative coordinate system in m
	vector3f velocity;				// m/s
	vector3f angularVelocity; 		// angular velocity
	vector3f linearAcceleration; 	// accelerations wrt world exp in body
	uint16_t sonarFrontMm; 			// front-sonar measurement in millimeters.
	uint32_t timestamp; 			// in ms
	uint32_t flags;
} TYPHOON_H_OBSTACLE_DATA_INPUT; //send from the autopilot to the CHT board with 100Hz. Packagetype 0x10
 
typedef struct __attribute((packed))
{
	vector3f obstacleAvoidanceSpeed; 	// m/s in earth frame
	float obstacleAvoidanceYawSpeed; 	// radians/s
	uint32_t timestamp; 				// in ms
	uint32_t flags; 					// 0x01=camera OK
} TYPHOON_H_OBSTACLE_DATA_OUTPUT; //send from the CHT board to the autopilot with 60Hz. Packagetype 0x11

enum ObstacleAvoidanceInputFlags {
	OK_TO_FLY 	= 0x01,	///< Set if GPS Has lock and state estimation provides reliable data.
	RESET_CA	= 0x02,	///< Signal to reset CA
	KEEP_MAP	= 0x04,
	USE_SMALL_VEHICLE_SIZE	= 0x08, ///< Set a small vehicle size for smooth manual flight
	devel_DESIRED_VELOCITY_PASSTHROUGH = 0x80000000	///< For development only. If set, OA will just output the desired velocity
};

typedef struct __attribute((packed))
{
	uint64_t   shutter_time;			// time elapse between the current optical flow calculation and message send out
	uint32_t   integration_time_us;		// time elapse between two message sending time
	float      integration_x;			// flow in radians around X axis (RH rotation definition)
	float      integration_y;			// flow in radians around Y axis (RH ratation definition)
	float      gyro_x;					// RH rotation around X axis in rad
	float      gyro_y;					// RH rotation around Y axis in rad
	float      gyro_z;					// RH rotation around Z axis in rad
	uint8_t    quality;					// optical flow quality
	uint32_t   time_delta_distance_us;	// time elapse between the distance measurement and message send out
	float      distance;				// distance to the center of the flow field in meters
} TYPHOON_H_OPTFLOW_DATA_INPUT;	// send from the CHT board to the autopilot. Packgetype 0x12

/**
 * Distance of obstacles in front of the camera. Units in 10cm, starting -30 deg from the left, in 5 deg increments to the right.
 * Expressed in camera coordinates. It is the slice of zero elevation Angle seen from the camera. A value of 0 means no obstacles.
 */
#define REALSENSE_FRONT_DISTANCE_SCALAR		10 	// 10cm/LSB

typedef struct __attribute((packed))
{
	uint8_t 	front_distance[12];		// distance in front
} TYPHOON_H_OBSTACLE_DISTANCE_INFO;	// send from the CHT board to the autopilot with 5Hz. Packgetype 0x13

#define REALSENSE_DISTANCE_SIZE		72
typedef struct __attribute__((packed))
{
	/// Timestamp of the CHT board in [ms]
	uint32_t timestampMs;
	/**
	* Distance between vehicle and obstacles 360 deg around the vehicle. Azimuth is discretized to 18 segments with 20 degrees each, containing the minimal distance of an obstacle
	* within this sector. Elevation is discretized into 3 segments with 15.56 degrees each. Coordinates are expressed in the world coordinate system. Zero azimuth is north, increasing clockwise.
	* One unit is 10 cm.
	* Data layout: starting from the bottom layer, increasing azimuth cw, then middle layer, then top layer.
	*/
	uint8_t distances[REALSENSE_DISTANCE_SIZE];
} TYPHOON_H_OBSTACLE_DISTANCE_360; // 360 deg distance info sent from the CHT to the flight controller. Packet ID 0x15

enum CommandType {
	CMD_SHUTDOWN = 0x01,
};

typedef struct __attribute((packed))
{
	uint32_t command;	// command type
} TYPHOON_H_CHT_COMMAND;	// send from the autopilot to the CHT with 20Hz. Packgetype 0x16


/**************************************************************************
 * Waypoint mission packet definition
 **************************************************************************/
typedef enum {
	ActionResponseType_RESPONSE = 1,
	ActionResponseType_FEEDBACK = 3,
} ActionResponseType;

typedef enum{
	MissionState_Stop		= 0,
	MissionState_Running	= 1,
	MissionState_Pause		= 2,
} MissionState;

typedef struct {
	uint8_t action_response_type;// = ACTION_TYPE_RESPONSE;
	uint8_t action_type;
	uint8_t action_request;
	uint8_t action_result;
} Response;


/*******************************************************
 * new mapwaypoint_protocal
 * *****************************************************/

// new protocol waypoint 30/8/2016
typedef enum {
	Pan_FollowRoute		= 0,    //yaw follow route
	Pan_Manual			= 1,	//manual control
	Pan_Custom			= 2,	//custom,towards point of interest or program yaw angle
} PanMode;

typedef enum {
	Tilt_Poi			= 0,	//pitch towards point of interest
	Tilt_Manual			= 1,	//manual control
	Tilt_Custom			= 2,	//custom,program pitch angle
} TiltMode;

//config waypoint
typedef struct {
	uint8_t action_type;		// == ACTION_TYPE_MAP_WAYPOINT = 12;
	uint8_t action_request;		// == ACTION_REQUEST_CONFIG   = 8,
	uint8_t waypointCount;
	AirlineType airline_type;
	AirlineEndingAction airline_ending_action;
	uint8_t altitude_type;		// the altitude type: absolute or relative
	PanMode pan_mode;
	TiltMode tilt_mode;
	uint8_t airline_default_speed;
} MapConfigRequest;

//set MapWaypoint
typedef struct {
	uint8_t waypointIndex;		// index of waypoint
	int32_t lat;				// lattitude (degrees)	+/- 90 deg
	int32_t lon;			  	// longitude (degrees)	+/- 180 deg
	int32_t alt;				// 0.01m resolution, altitude (meters)
	uint8_t speed;              // uint Km/H
	int16_t pan_angle;				//yaw angle
	int16_t tilt_angle;				//pitch angle
	TiltMode tilt_mode;
	uint8_t doCmdCount;			//do_cmd count in this point
} MapWaypoint;

//set DoCmd
typedef enum {
	DoCmdType_Wait = 0,
	DoCmdType_Camera,		//all cmd about camera use this type,all the parameter is in the DoCmdParam
	DoCmdType_PanRot360,
	DoCmdType_FullView,
	DOCmdType_GimbalSet,
	//other do cmd type can added here
} DoCmdType;

typedef struct {
	uint16_t GimbalParam_yaw_angle;			//if GimbalParam_yaw_orientation == Gimbal_Orientation_Programed
	uint16_t GimbalParam_pitch_angle;			//if GimbalParam_pitch_orientation == Gimbal_Orientation_Programed
} GimbalSet;

typedef struct {
	uint8_t Photo;
	uint8_t Shooting;
	uint8_t ShootTime;
} CameraParam;

typedef struct {

} PanRot360;

typedef struct {

} FullView;

typedef struct {
	uint16_t time;
} DelayTime;

typedef  union {
	//here can add different do cmd ,but most bytes of each cmd parameter cannot over 12 bytes
	GimbalSet	gimbalset;
	PanRot360   panrot360;
	FullView    fullview;
	CameraParam camera;
	DelayTime   delaytime;
	//most bytes limits
	uint8_t bytes[12];
} DoCmdParam;

/*--------------*set request*----------------*/
typedef enum {
	MapSetWaypointType_Waypoint = 0,
	MapSetWaypointType_Docmd    = 1,
} MapWaypointSetType;

typedef struct {
	uint8_t		waypointIndex;
	uint8_t		doCmdIndex;
	uint8_t		doCmdType;					//enum DoCmdType
	uint16_t	delayTime;
	DoCmdParam	doCmdParam;
} MapWaypointDoCmd;


typedef union {
	MapWaypoint 		waypoint;
	MapWaypointDoCmd	waypointDoCmd;
} MapWaypointSetting;


//other do cmd edit here

typedef struct {
	ActionResponseType 	action_response_type;// = ACTION_TYPE_RESPONSE;
	uint8_t 			action_type;
	uint8_t		 		drone_state;
	MissionState		mission_state;
	uint8_t				value;   //
	uint16_t 			yaw_angle;   //0-360   precision 1 degree
	uint8_t				pitch_angle;
//	uint8_t     		camera;    //0 for photo(1);1 for shooting(1); 2-7 for shooting time;
} MissionFeedback;

#define DRONE_STATE_LAND_COMPLETE_BIT		0	//BIT 0:	1 mean land_complete
#define DRONE_STATE_MOTOR_ARMED_BIT			1	//BIT1:  1 mean motor_armed


typedef struct {
	Response response;
	uint8_t waypointIndex;    			// the waypoint index of receive.
} MapWaypointSetResponse;

typedef struct {
	uint8_t action_type;   				 // == ACTION_TYPE_MAP_WAYPOINT = 12;
	uint8_t waypointIndex;    			// the waypoint index of receive.
	uint8_t docmdIndex;    			// the waypoint index of receive.
	CameraActionRequest actionRequest;
	uint8_t shooting_time;            //photo:0; shooting:1~60;
} MapWaypointCameraRequest;

typedef struct {
	uint8_t action_type;		// ActionType_MAP_WAYPOINT == 12;
	uint8_t action_request;
} MapWaypointRequest;

typedef struct {
	uint8_t action_type;		// == ACTION_TYPE_MAP_WAYPOINT = 12;
	uint8_t action_request;		// == ACTION_REQUEST_SET   = 6,
	MapWaypointSetType	set_type;
	MapWaypointSetting	waypointSetting;
} MapWaypointSetRequest;

typedef struct {
	uint8_t action_type;			// ActionType_FENCE == 13;
	uint8_t action_request;			// ActionRequest_CONFIG == 8 ;
	uint8_t gpsPointCount;
	uint8_t fencetype;
	int32_t alt;						// 0.01m resolution, altitude (meters)
	uint8_t altitudeType;				// the altitude type: absolute or relative
} FenceConfigRequest;


typedef struct {
	uint8_t  waypointIndex;				// index of waypoint
	int32_t  lat;						// lattitude (degrees)	+/- 90 deg
	int32_t  lon;						// longitude (degrees)	+/- 180 deg
} FenceWaypoint;

typedef struct {
	uint8_t action_type;			// ActionType_FENCE == 13;
	uint8_t action_request;			// ActionRequest_SET == 6
	FenceWaypoint fence_waypoint;
} FenceSetWaypointRequest;

typedef struct {
	Response response;
	uint8_t waypointIndex;				// the waypoint index of receive.
} FenceSetWaypointResponse;


typedef struct {
	uint8_t action_type;			// ActionType_FENCE == 13;
	uint8_t action_request;
} FenceRequest;

//onekey takeoff
typedef struct {
	uint8_t action_type;			// ACTION_TYPE_TAKEOFF == 15;
	uint8_t action_request;			// ActionRequest_CONFIG == 8 ;
	uint8_t takeoffSpeed;
	uint8_t takeoffHeight;
} ActionTakeOffStart;

typedef struct {
	uint8_t action_type;			// ACTION_TYPE_TAKEOFF == 14;
	uint8_t action_request;			// ActionRequest_START == 0;
	int32_t  lat;					// lattitude (degrees)	+/- 90 deg
	int32_t  lon;					// longitude (degrees)	+/- 180 deg
} LandingRequest;

typedef struct {
	uint8_t action_type;			// ACTION_TYPE_TAKEOFF == 14;
	uint8_t action_request;			// ActionRequest_START == 0;
} LandingExit;

typedef struct {
	int32_t  lat;					// lattitude (degrees)	+/- 90 deg
	int32_t  lon;					// longitude (degrees)	+/- 180 deg
	int32_t  alt;					// alt
} LandingWayPoint;

/**

map waypoint setup:
		GCS																Aircraft
1, 	<MapConfigRequest>		  									===>	check and reply
							  									<===	<Response>


2, <MapWaypointSetRequest>    									===>    check and reply
							  									<===    <MapWaypointSetResponse>


							  									....

n, <MapWaypointRequest>(action_request=ActionRequest_START)    ===> 	check and reply (start mission)
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

if GCS want to terminate the process, it would to send the request with "Action_Request=ACTION_REQUEST_EXIT".
Aircraft will terminate the process and clear all settings about the process immediatly after receiving the request.

*/
//
// Add for the waypoint mission with remote control.

#pragma pack(pop)

/*
 * Channel packet length
 */
#define PACKET_LENGTH_CHANNELDATA12	        			sizeof(ChannelData12)
#define PACKET_LENGTH_CHANNELDATA24	        			sizeof(ChannelData24)
#define PACKET_LENGTH_CHANNELDATA12_GPS	       		 	sizeof(ChannelData12_Gps)
#define PACKET_LENGTH_TELEMETRYDATA	        			sizeof(TelemetryData)
#define PACKET_LENGTH_STBINDCMD	               		 	sizeof(StBindCmd)

/*
 * Action packet length
 */
#define PACKET_LENGTH_ACTIONREQUEST             		sizeof(ActionRequest)
#define PACKET_LENGTH_ACTIONRESPONSE	        		sizeof(ActionResponse)
#define PACKET_LENGTH_ACTIONFEEDBACK	        		sizeof(ActionFeedback)
#define PACKET_LENGTH_ACTIONSETTING_CCC	        		sizeof(ActionSettingCCC)
#define PACKET_LENGTH_ACTIONSETTING_ROI					sizeof(ActionSettingROI)
#define PACKET_LENGTH_ACTIONSETTING_JOUR                sizeof(ActionSettingJOUR)
#define PACKET_LENGTH_CACONFIG							sizeof(ActionCAConfig)
#define PACKET_LENGTH_LEDCONFIG							sizeof(ActionLEDConfig)
#define PACKET_LENGTH_ACTIONTAKEOFF         			sizeof(ActionTakeOff)
#define PACKET_LENGTH_FRONTDISTANCE						sizeof(ActionFrontDistance)
#define PACKET_LENGTH_ASKFORCONFIG         				sizeof(ActionAskForConfig)
#define PACKET_LENGTH_GOHOMECONFIG						sizeof(ActionGoHomeConfig)

/*
 * RealSense packet length
 */
#define PACKET_LENGTH_HEARTBEAT						 	sizeof(HEARTBEAT)
#define PACKET_LENGTH_TYPHOON_H_OBSTACLE_DATA_INPUT 	sizeof(TYPHOON_H_OBSTACLE_DATA_INPUT)
#define PACKET_LENGTH_TYPHOON_H_OBSTACLE_DATA_OUTPUT 	sizeof(TYPHOON_H_OBSTACLE_DATA_OUTPUT)
#define PACKET_LENGTH_TYPHOON_H_OPTFLOW				 	sizeof(TYPHOON_H_OPTFLOW_DATA_INPUT)
#define PACKET_LENGTH_TYPHOON_H_OBSTACLE_DISTANCE_INFO	sizeof(TYPHOON_H_OBSTACLE_DISTANCE_INFO)
#define PACKET_LENGTH_TYPHOON_H_OBSTACLE_DISTANCE_360	sizeof(TYPHOON_H_OBSTACLE_DISTANCE_360)
#define PACKET_LENGTH_TYPHOON_H_CHT_COMMAND				sizeof(TYPHOON_H_CHT_COMMAND)

/*
 * Waypoint packet length
 */
#define PACKET_LENGTH_RESPONSE							sizeof(Response)
#define PACKET_LENGTH_MAPCONFIGREQUEST					sizeof(MapConfigRequest)
#define PACKET_LENGTH_WAYPOINTACTIONPARAMORBIT 			sizeof(WaypointActionParamOrbit)
#define PACKET_LENGTH_MAPWAYPOINT 						sizeof(MapWaypoint)
#define PACKET_LENGTH_MAPSETWAYPOINTREQUEST				sizeof(MapWaypointSetRequest)
#define PACKET_LENGTH_MAPSETWAYPOINTRESPONSE			sizeof(MapWaypointSetResponse)
#define PACKET_LENGTH_MAPWAYPOINTREQUEST 				sizeof(MapWaypointRequest)
#define PCKKET_LENGTH_MISSIONFEEDBACK					sizeof(MissionFeedback)
#define PACKET_LENGTH_FENCCONFIGREQUEST 				sizeof(FenceConfigRequest)
#define PACKET_LENGTH_FENCEWAYPOINT 					sizeof(FenceWaypoint)
#define PACKET_LENGTH_FENCESETWAYPOINTREQUEST 			sizeof(FenceSetWaypointRequest)
#define PACKET_LENGTH_FENCESETWAYPOINTRESPONSE 			sizeof(FenceSetWaypointResponse)
#define PACKET_LENGTH_FENCEREQUEST 						sizeof(FenceRequest)
#define PACKET_LENGTH_LANDINGREQUEST 					sizeof(LandingRequest)
#define PACKET_LENGTH_ACTIONTAKEOFFSTART				sizeof(ActionTakeOffStart)
#define PACKET_LENGTH_ACTIONCAMERAREQUEST				sizeof(MapWaypointCameraRequest)


#define YPRC_USE_CRC8_TABLE 1

/**
 * CRC8 implementation for YPRC protocol
 *
 * @param prt Pointer to the data to CRC
 * @param len number of bytes to accumulate in the checksum
 * @return the checksum of these bytes over len
 */
#if YPRC_USE_CRC8_TABLE
uint8_t yprc_crc8(uint8_t *ptr, uint8_t len);
#else
uint8_t yprc_crc8(uint8_t *ptr, uint8_t len);
#endif

/**
 * Decoder for YPRC protocol
 *
 * @param byte current char to read
 * @param rssi pointer to a byte where the RSSI value is written back to
 * @param rx_count pointer to a byte where the receive count of packets since last wireless frame is written back to
 * @param channels pointer to a data structure of size max_chan_count where channel values (12 bit) are written back to
 * @param max_chan_count maximum channels to decode - if more channels are decoded, the last n are skipped and success (0) is returned
 * @return 0 for success (a decoded packet), 1 for no packet yet (accumulating), 2 for unknown packet, 3 for out of sync, 4 for checksum error
 */
ReceiverFcPacket* yprc_encode_bind(StBindCmd *bindCmd);


/********************handle yuneec protocol*******************
* frame sync for YP
* param in byte : new char received
* param out returnPacket :synced packet
* return 0:success 1:syncing 3:unsync 4:CRC err
*/
int _YP_Frame_Sync(uint8_t index, uint8_t byte, ReceiverFcPacket* returnPacket);

#ifdef __cplusplus
} // extern "C"
#endif




//CURVE CABLE CAM MODE 
/*

	 			GCS  									copter

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
	 			GCS 									copter
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
	n+1.ACTION_REQUEST(ACTION_REQUEST_START)	==>			
												<==         ACTION_RESPONSE()
							 
**********************************************************************************
$$get point
1.	ACTION_REQUEST(ACTION_REQUEST_GET) 			==>
							 
							    				<==			ACTIONSETTING_CCC//only send setting data when ACTION_RESPONSE is ok
**********************************************************************************
once action state changed ,send feedback
1.												<==			ACTION_FEEDBACK 
	ACTION_RESPONSE	    						==>			
*/

#endif
