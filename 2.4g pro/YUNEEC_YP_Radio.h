/*
 * This is the driver for ESCBUS electrical speed controller from YUNEEC
 */
#ifndef __YUNEEC_YP_RADIO_H__
#define __YUNEEC_YP_RADIO_H__

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <AP_Param.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_Mission.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>
#include <AC_Circle.h>
#include <AP_AHRS.h>
#include <AP_InertialSensor.h>
#include <AP_BattMonitor.h>
#include <AP_Notify.h>
#include <YUNEEC_ESCBUS_Motors.h>
#include "RCInput.h"
#include <utility/yuneec_protocal_rc.h>
#include <utility/pinmap_typedef.h>
#include <vectorN.h>
#include <AP_Math.h>
#include <YUNEEC_GimbalCamera.h>
#include "../../ArtIntCopter/defines.h"


// YP radio GPS offset compensate for difference between radio and copter
#define LATITUDE_OFFSET					(int32_t)0
#define LONGITUDE_OFFSET				(int32_t)0

#define LOCATION_SCALING_FACTOR_INV 	89.83204953368922f

// YP radio power enable pin definition
#define YP_RADIO_POWER_PIN					PC3
// YP radio channel info
#define YP_RC_INPUT_CHANNEL					12
#define YP_CHANNEL_UPDATE_TIME				(uint32_t)1000 // ms
#define RADIO_CHANNEL_UPDATE_TIME			(uint32_t)200 // ms
#define WIFI_CHANNEL_UPDATE_TIME			(uint32_t)500 // ms

// YP radio GPS info
#define YP_GPS_HACC_MIN						(float)1.0f 			// m
#define YP_GPS_TIMEOUT						(uint32_t)3000			// ms
#define YP_GPS_PROCESS_NOISE				(float)3.5f				// m/s/s 	Tesla Model S: 9.3 m/s/s
#define YP_GPS_POS_MES_NOISE				(float)1.8f				// m 		position accuracy
#define YP_GPS_POS_MES_GATE					(float)10.0f
#define YP_GPS_VEL_MES_NOISE				(float)0.3f				// m/s
#define YP_GPS_VEL_MES_GATE					(float)10.0f

// YP radio key function channel definition
#define YP_KEY_FUNCTION_CHANNEL				(uint8_t)10
#define YP_SCREEN_BUTTON_FUNCTION_CHANNEL	(uint8_t)9

#define YPRC_PORT_NUM	0

// data rate definition
#define YPRC_DATARATE_TELEMETRY			20	// ms
#define YPRC_DATARATE_ACION_FEEDBACK	333	// ms
#define YPRC_DATARATE_FRONT_DISTANCE	250	// ms
#define YPRC_DATARATE_MISSION_FEEDBACK  455 // ms

class YP_Radio {
	friend class YUNEEC::YUNEECRCInputYP;
	friend class GimbalCamera;
public:
	typedef float ftype;
	typedef ftype Vector2[2];
    typedef ftype Matrix2[2][2];

	// constructor
	YP_Radio(AP_AHRS &ahrs, 
		     AP_BattMonitor &battery, 
		     Compass &compass,
		     AP_Mission &mission,
			 AC_Fence &fence,
		     AC_WPNav &wp_nav, 
		     GimbalCamera &gimbalcamera,
		     AC_Circle &ac_circle, 
		     struct Location &cur_loc);

	// initialize uart port for YPRC
	void init(AP_HAL::UARTDriver* port);

	// check YP input
    void check_input(void);

	// get GPS update timestamp
	uint32_t gps_last_update_time() {
		return _last_gps_update_time;
	}

	// return true if gps position ok
	bool position_ok();

	// get number of satellites
	uint8_t get_num_sats() {
		return (_gps_data.fixmode_satellite & (uint8_t)0x1f);
	}

	float get_altitude_cm(void) {
		return (_gps_data.alt*0.1f);
	}

	// get raw GPS location from RC
	const struct Location &get_location() {
		_position.lat = _gps_data.lat;
		_position.lng = _gps_data.lon;
		_position.alt = (int32_t)(_gps_data.alt*0.1f);
		return _position;
	}

	// get raw GPS velocity North/East in m/s
	const Vector2f &get_velocity_NE() {
		float ground_speed = get_speed();
		float ground_course = get_course() * DEG_TO_RAD;
		_velocity_NE.x = ground_speed * cosf(ground_course);
		_velocity_NE.y = ground_speed * sinf(ground_course);
		return _velocity_NE;
	}

   void vehicle_kalman_filter_pos_vel(const struct Location& vehicle_location, const Vector2f& vehicle_speed, bool new_gps_data, bool vehicle_position_ok);

	bool get_vehicle_kf_location(struct Location& vehicle_loc) {
		if (_vehicle_kf_initialised) {
			vehicle_loc = _vehicle_kf_position;
			return true;
		}
		return false;
	}

	bool get_vehicle_kf_velocity(Vector2f& vehicle_speed) {
		if (_vehicle_kf_initialised) {
			vehicle_speed.x = _vehicle_latX[1];
			vehicle_speed.y = _vehicle_lngX[1];
			return true;
		}
		return false;
	}

	// update kalman filter for GPS position
    void kalman_filter_pos_vel(void);

	// get kalman filter location
	const struct Location &get_kf_location() {
		if (_kf_initialised) {
			_kf_position.alt = (int32_t)(_gps_data.alt*0.1f);
			return _kf_position;
		} else {
			return get_location();
		}
	}

	// get kalman filter velocity
	const Vector2f &get_kf_velocity_NE() {
		if (_kf_initialised) {
			_kf_velocity_NE.x = latX[1];
			_kf_velocity_NE.y = lngX[1];
		} else {
			float ground_speed = get_speed();
			float ground_course = get_course() * DEG_TO_RAD;
			float vel_N = ground_speed * cosf(ground_course);
			float vel_E = ground_speed * sinf(ground_course);
			_kf_velocity_NE.x = vel_N;
			_kf_velocity_NE.y = vel_E;
		}

		return _kf_velocity_NE;
	}

	// get GPS accuracy from RC
	float get_accuracy() {
		return (float)_gps_data.hacc * 0.001f;	// cm
	}

	// get GPS ground speed of RC
	float get_speed() {
		return (float)_gps_data.speed * 0.001f;  // m/s
	}

	// get GPS heading of RC in degrees
	float get_course() {
		float course = (float)_gps_data.course * 0.01f;
		if (_gps_data.course < 0) {
			course += 360.0f;
		}
		return course;
	}

	const TelemetryData& get_telem_data(void) const {
		return _telem_data;
	}

	int16_t get_raw_course() {
		return _gps_data.course;
	}

	uint16_t get_raw_channel(uint8_t channel_num) {
		if (channel_num >= YP_RC_INPUT_CHANNEL) return 0;
		return _channels_raw[channel_num];
	}

	void get_channels_raw(volatile uint16_t channels_to_get[]) {
		for (uint8_t i = 0; i < YP_RC_INPUT_CHANNEL; i++)
			channels_to_get[i] = _channels_raw[i];
	}	

	void get_gps_data(StGpsDataType *gps_data) {
		memcpy(gps_data, (const StGpsDataType *)&_gps_data, sizeof(_gps_data));
	}

	uint16_t get_radio_raw_channel(uint8_t channel_num) {
		if (channel_num >= YP_RC_INPUT_CHANNEL) return 0;
		return _radio_channels_raw[channel_num];
	}

	void get_radio_gps_data(StGpsDataType *gps_data) {
		memcpy(gps_data, (const StGpsDataType *)&_radio_gps_data, sizeof(_gps_data));
	}

    // send telemetry data to RC
	void update_radio(void);

	// check user' command for binding
	void check_if_need_binding();
	void set_sending_bind_WIFI_flags(bool flags) {	sending_bind_WIFI_flags = flags;}
	bool get_sending_bind_WIFI_flags() { return sending_bind_WIFI_flags;}
	bool get_bind_ack_flags() { return bind_ack_flags; }
	void set_bind_ack_flags(bool flags) { bind_ack_flags = flags; }

	// set motor failure mask to show which motors are failed
	void set_motor_failure_mask(uint8_t motor_failure_mask) {
		_motor_failure_mask = motor_failure_mask;
	}

	// switch mode
	enum Switch_Mode {
		Switch_Mode_None = 0,
		Switch_Mode_GPSDisable,			// disable GPS
		Switch_Mode_GPSEnable,			// enable GPS
		Switch_Mode_GPSCalibrate,		// calibrate GPS offset between radio and copter
		Switch_Mode_IMUCalibrate,		// calibrate IMU
		Switch_Mode_MAGCalibrate,		// calibrate MAG
		Switch_Mode_DisableFollowMe, 	// disable follow me(normal smart mode)
		Switch_Mode_DisableWatchMe,		// disable watch me
		Switch_Mode_EnableWatchMe,		// enable watch me
		Switch_Mode_LandingGear,		// retract/deploy landing gear
		Switch_Mode_EnableCA,			// enable CA mode
		Switch_Mode_DisableCA,			// disable CA mode
		Switch_Mode_3DFollow,			// 3D follow on/off
		Switch_Mode_IPSOnly,			// IPS only flight mode
	};

	// check which mode we are switching in
	enum Switch_Mode get_switch_mode() {
		return _switch_mode;
	}

	struct Key_Function {
		uint8_t landing_gear_retract	: 1;	// retract landing gear
		uint8_t watchme_on				: 1;	// watch me function on/off
		uint8_t ca_on					: 1;	// collision avoidance mode on/off
	} key_function;

	enum Flight_Mode {
		Flight_Mode_THR = 0,			// not available now - angle mode with directly throttle control(this mode will be triggered if baro is bad)
		Flight_Mode_THR_GPS_OFF,		// not available now - stabilize mode
		Flight_Mode_THR_NO_GPS,			// not available now - stabilize mode
		Flight_Mode_ANGLE,				// angle mode with GPS 3D locked
		Flight_Mode_ANGLE_IPS_ONLY,		// angle mode with IPS only
		Flight_Mode_ANGLE_GPS_OFF,		// gps off mode(althold mode in fact), always send this value if user switch off GPS
		Flight_Mode_ANGLE_NO_GPS,		// angle mode without GPS 3D locked
		Flight_Mode_SMART,				// smart mode with GPS 3D locked
		Flight_Mode_SMART_NO_GPS,		// smart mode
		Flight_Mode_ACRO,				// not available now - acro mode
		Flight_Mode_FOLLOW,				// follow me mode with both copter and rc GPS 3D locked
		Flight_Mode_FOLLOW_NO_RC_GPS,	// follow me mode without rc GPS 3D locked
		Flight_Mode_WATCHME,			// camera tracking mode
		Flight_Mode_WATCHME_NO_RC_GPS,	// camera tracking mode without rc GPS 3D locked
		Flight_Mode_RETURN_TO_HOME,		// return to home
		Flight_Mode_LAND,				// land
		Flight_Mode_3D_FOLLOW,			// 3D follow mode
		Flight_Mode_3D_WATCHME,			// 3D watch me mode
	};

	enum Pre_Flight_Status {
		Pre_Flight_Status_Not_Ready = 0,
		Pre_Flight_Status_Ready,
		Pre_Flight_Status_Arming,
		Pre_Flight_Status_Armed
	};

	enum Calibration_Type {
		Calibration_Type_None = 0,
		Calibration_Type_IMU,
		Calibration_Type_MAG
	};

	struct vehicle_state_type {
		uint16_t pre_flight_status  	: 2;
		uint16_t flight_mode			: 8;
		uint16_t emergency_shutdown 	: 1;
		uint16_t calibration_type   	: 2;
		uint16_t imu_preheating			: 1;
	};

	static struct vehicle_state_type vehicle_state;

	enum rc_type {
		RC_TYPE_ST = 0,
		RC_TYPE_WIZARD,
	};

	enum rc_type get_rc_type() {
		if (_gps_data.fixmode_satellite & 0x80) return RC_TYPE_WIZARD;
		else 									return RC_TYPE_ST;
	}

	enum fix_type {
		FIX_TYPE_NONE = 0,
		FIX_TYPE_UNLOCK,
		FIX_TYPE_2DLOCK,
		FIX_TYPE_3DLOCK,
	};

	enum fix_type get_fix_type() {
		return (enum fix_type)((_gps_data.fixmode_satellite & 0x60) >> 5);
	}

//	typedef enum {
//		map_waypoint_set_type_none = 0,
//		map_waypoint_set_type_yp,
//		map_waypoint_set_type_wifi
//	} Map_waypoint_set_type;

    ModeType_e get_modetype(void) { return _mode_type; }

	void send_jour_cmd(void);
    void send_action_request_set_ccc(void);
    void send_action_setting_ccc(void);
    void send_action_feedback(void);
    void send_response(ActionResult_e result, uint8_t count);
    void send_ccc_start_cmd(void);
    void send_action_request_get_ccc(void);
    void send_action_setting_roi(void);
    void send_bind(void);
    void send_circleme_start_cmd(void);
    void set_photo_mode_clear(void);
    void exit_photo_mode(void);
    void set_photo_state_resume(void);
    void set_photo_state_stop(void) { _action_state = ACTION_STATUS_STOPPED; }
    bool get_new_gps(void) { return _new_gps_data; }

    // set front distance data buffer pointer
    void set_front_distance(const uint8_t* front_distance_p) {
    	memcpy(_front_distance, front_distance_p, REALSENSE_OBSTACLE_DISTANCE_INFO_SIZE);
    	_have_distance_data = true;
    }

    // set/reset high cpass_velocity_variance
    void set_compass_velocity_high_variance(bool high_variance) {
    	_high_compass_velocity_variance = high_variance;
    }

    // current channel type
    typedef enum {
    	CHANNEL_TYPE_NONE = 0,
    	CHANNEL_TYPE_RADIO = 1,
    	CHANNEL_TYPE_WIFI = 2,
    } channel_type_e;

    channel_type_e current_channel_type(void) { return _current_channel_type; }

    // waypoint navigation
	void send_map_waypoint_response(ActionType_e type, ActionRequest_e request, ActionResult_e result, uint8_t index, Map_waypoint_set_type set_type);
	void send_takeoff_landing_response(ActionType_e type, ActionRequest_e request, ActionResult_e result);
	void send_map_fence_response(ActionType_e type, ActionRequest_e request, ActionResult_e result, uint8_t index);
	void send_action_mission_feedback(ActionType_e type, uint8_t drone_state, uint8_t index, MissionState mission_state);
	void send_action_current_mapwapoint(void);
	void send_map_waypoint_camera(CameraActionRequest request, uint8_t waypoint_index, uint8_t docmd_index, uint8_t shooting_time);

    static const struct AP_Param::GroupInfo var_info[];

private:
    /* which uart is used as YP communication port */
    AP_HAL::UARTDriver* _port;
    bool _initialised;

    /* YP data */
    uint8_t _rssi;
	uint8_t _rx_count;
	uint16_t _channel_count;
	uint16_t _channels[YP_RC_INPUT_CHANNEL];
	uint16_t _channels_raw[YP_RC_INPUT_CHANNEL];
	StGpsDataType _gps_data;
	StGpsDataType _last_gps_data;
	uint16_t _radio_channels[YP_RC_INPUT_CHANNEL];
	uint16_t _radio_channels_raw[YP_RC_INPUT_CHANNEL];
	StGpsDataType _radio_gps_data;
	StGpsDataType _radio_last_gps_data;
	uint32_t _last_channel_update_time;
	uint32_t _last_radio_channel_update_time;
	uint32_t _last_wifi_channel_update_time;
	uint32_t _last_gps_update_time;
	bool _new_gps_data;

	channel_type_e _current_channel_type;

	struct Location _position;
	Vector2f _velocity_NE;

	bool _high_compass_velocity_variance;

	/*
	 * Obstacle in front distance, unit 10cm/LSB
	 */
	uint8_t _front_distance[REALSENSE_OBSTACLE_DISTANCE_INFO_SIZE];
	bool	_have_distance_data;

	/*
	 * Timestamp for feedback
	 */
	uint32_t _timestamp_telemetry;
	uint32_t _timestamp_action_feedback;
	uint32_t _timestamp_front_distance;
	uint32_t _timestamp_mission_feedback;

    void _check_switch_mode(void);
    void _update_key_function(void);

    // send telemetry for rc
    void _send_telemetry();
    void _send_front_distance(void);
    void _send_gohome_config(uint16_t goHomeHeight, uint16_t goHomeConfig);
    void _pack_and_send(YPRC_PACKET_TYPE packet_type, uint8_t send_count);

    void _send_bind(void);
    void _YP_Handle_Packet(ReceiverFcPacket* packet_in);
    void _handle_packet_channeldata12(ChannelData12* packet);
    void _handle_packet_channeldata12_GPS(ChannelData12_Gps* packet);
    void _handle_packet_channeldata24(ChannelData24* packet);
    void _handle_packet_action(ReceiverFcPacket* packet_in);

    FUNCTOR_TYPEDEF(set_mode_fn, bool, uint8_t);
    FUNCTOR_TYPEDEF(ccc_start_fn, bool);
    FUNCTOR_TYPEDEF(poi_orbit_start_fn, bool);
    FUNCTOR_TYPEDEF(journey_start_fn, bool);
    FUNCTOR_TYPEDEF(reset_control_switch_fn, void);
    FUNCTOR_TYPEDEF(journey_stop_fn,void);
    FUNCTOR_TYPEDEF(journey_resume_fn,void);
    FUNCTOR_TYPEDEF(roi_reset_fn, void);
    FUNCTOR_TYPEDEF(roi_config_fn, bool, const Vector3ul&, const float&);
    FUNCTOR_TYPEDEF(circle_me_config_fn, void);
    FUNCTOR_TYPEDEF(motors_auto_arming_fn, bool, uint8_t, uint8_t);
    FUNCTOR_TYPEDEF(jour_config_fn, bool, uint8_t, uint8_t);

    void _handle_start_auto(ccc_start_fn ccc_start, poi_orbit_start_fn poi_orbit_start, journey_start_fn journey_start, circle_me_config_fn circle_me_config);
    void _handle_action_request(ActionRequest* packet_in);
    void _handle_action_response(ActionResponse* packet_in);
    void _handle_action_settingccc(ActionSettingCCC* packet_in);
    void _handle_action_settingroi(ActionSettingROI* packet_in, roi_config_fn roi_config);
    void _handle_action_takeoff(ActionTakeOff* packet_in, motors_auto_arming_fn motors_auto_arming);
    void _handle_action_settingjour(ActionSettingJOUR* packet_in, jour_config_fn jour_config);

    void _handle_exit(uint8_t actiontype, reset_control_switch_fn reset_control_switch, Map_waypoint_set_type set_type);
	void _handle_pause(journey_stop_fn journey_stopped);
    void _handle_resume(journey_resume_fn journey_resume);
    void _handle_set(roi_reset_fn roi_reset);
    
	void _handle_waypoint_start(uint8_t land_complete,bool motor_armed, set_mode_fn set_mode);
	void _handle_waypoint_config(Map_waypoint_set_type set_type);
    void _handle_MapSetWaypointRequest(MapWaypointSetRequest *packet_in, Map_waypoint_set_type set_type);
	void _handle_MapWaypointRequest(MapWaypointRequest *packet_in, Map_waypoint_set_type set_type);
	void _handle_MapConfigRequest(MapConfigRequest *packet_in, Map_waypoint_set_type set_type);
	void _handle_fenceConfigRequest(FenceConfigRequest *packet_in);
	void _handle_FenceSetWaypointRequest(FenceSetWaypointRequest *packet_in);
	void _handle_FenceRequest(FenceRequest *packet_in);
	void _handle_LandingRequest(LandingRequest *packet_in);
	bool _handle_landingMission(LandingWayPoint *landpoint);
	void _handle_wp_mission_feedback_send(void);

    struct Location &_cur_loc;
    AP_AHRS &_ahrs;
    AP_BattMonitor &_battery;
    Compass &_compass;
   	AP_Mission &_mission;
   	AC_Fence &_fence;
   	AC_WPNav &_wp_nav;
    GimbalCamera &_gimbalcamera; 
    AC_Circle &_ac_circle;
    const AP_GPS &_gps;
    const AP_InertialSensor &_ins;
    const AP_Baro &_baro;

	/*
	*	temp variable for map waypoint config
	*/
	AirlineType _airline_type;
	AirlineEndingAction _airline_ending_action;
	uint8_t	_airline_default_speed;
	uint8_t	_airline_altitude_type;
	PanMode _pan_mode;
	TiltMode _tilt_mode;

    //data stream for sending and receiving
	ReceiverFcPacket _txpacket;
	ReceiverFcPacket _rxpacket;

	TelemetryData _telem_data;
	
	StBindCmd _bind_cmd;
	ActionResponse _action_response;
	ActionSettingCCC _action_settingccc;
	ActionFeedback _action_feedback;
	uint8_t _last_index;
	uint8_t _docmd_last_index;

	//enum status for photo mode
	ModeType_e _mode_type;
	ActionType_e _running_action_type;
	ActionRequest_e _action_request;
	Action_State _action_state;
	uint8_t _setting_count; // record the num of the point
	uint32_t _last_send_response_timestamp;
	uint8_t _docmd_setting_count;	//record the num of do cmd in a point


	MissionState	mission_state;
	Map_waypoint_set_type _map_waypoint_set_type;

	uint8_t bind_step;
	uint8_t bind_step_WIFI;
	uint32_t bind_time;
	bool sending_bind_WIFI_flags;       // flag for check if now switch single/dual remote control.
	bool bind_ack_flags;

	enum Switch_Mode _switch_mode;

	uint8_t _motor_failure_mask;

	// kalman filter for GPS location
	void _kalman_filter_init(float vel_N, float vel_E);

	AP_Float _process_noise;
	AP_Float _pos_measure_noise;
	AP_Float _vel_measure_noise;
	AP_Float _forward_pos_scalar;

	uint32_t _last_kalman_update_time;
	float _variance;
	bool _kf_initialised;
	struct Location _origin_loc;
	struct Location _kf_position;
	Vector2f _kf_velocity_NE;
	Vector2 latX, latY;
	Matrix2 latP, latnextP, latQ, latS, latinvS, latK, latKHP;
	Vector2 lngX, lngY;
	Matrix2 lngP, lngnextP, lngQ, lngS, lnginvS, lngK, lngKHP;

	void _vehicle_kalman_filter_init(float vel_N, float vel_E);

	uint32_t _vehicle_last_kalman_update_time;
	float _vehicle_variance;
	bool _vehicle_kf_initialised;
	struct Location _vehicle_origin_loc;
	struct Location _vehicle_kf_position;
	Vector2f _vehicle_kf_velocity_NE;
	Vector2 _vehicle_latX, _vehicle_latY;
	Matrix2 _vehicle_latP, _vehicle_latnextP, _vehicle_latQ, _vehicle_latS, _vehicle_latinvS, _vehicle_latK, _vehicle_latKHP;
	Vector2 _vehicle_lngX, _vehicle_lngY;
	Matrix2 _vehicle_lngP, _vehicle_lngnextP, _vehicle_lngQ, _vehicle_lngS, _vehicle_lnginvS, _vehicle_lngK, _vehicle_lngKHP;

};

#endif //__YUNEEC_YP_RADIO_H__
