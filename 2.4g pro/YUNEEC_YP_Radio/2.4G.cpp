#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <AP_Math.h>
#include "RCInput.h"
#include "YUNEEC_YP_Radio.h"
#include <utility/yuneec_protocal_rc.h>
#include <utility/pinmap_typedef.h>
#include <stdlib.h>
#include "../../ArtIntCopter/Copter.h"


#define DEBUG_PRINTF(x)  //hal.console->printf(x)
#define DEBUG_UARTG_PRINTF1(x)    //hal.uartG->printf(x)
#define DEBUG_UARTG_PRINTF2(x,y)  //hal.uartG->printf(x,y)

extern const AP_HAL::HAL& hal;

struct YP_Radio::vehicle_state_type YP_Radio::vehicle_state;

#define YUNEEC_TELEM_BIND_TIME_LIMIT	5000 	// ms
#define YUNEEC_SWITCH_MODE_TIMEOUT	 	1000	// ms
#define YUNEEC_MAX_GPS_OFFSET_DIS		2000	// cm
#define MAX_COMPASS_OFFSETS          	600

#define USE_STICK_SWITCH				1
#define USE_THROTTLE_YAW_STICK_SWITCH	1
#define USE_ROLL_PITCH_STICK_SWITCH		0

const AP_Param::GroupInfo YP_Radio::var_info[] PROGMEM = {

    // @Param: PNOISE
    // @DisplayName:
    // @Description:
    // @Units:
    // @Range:
    // @Increment:
    // @User: Advanced
    AP_GROUPINFO("PNOISE", 1, YP_Radio, _process_noise, YP_GPS_PROCESS_NOISE),

    // @Param: POS_NOISE
    // @DisplayName:
    // @Description:
    // @Units:
    // @Range:
    // @Increment:
    // @User: Advanced
    AP_GROUPINFO("POS_NOISE", 2, YP_Radio, _pos_measure_noise, YP_GPS_POS_MES_NOISE),

    // @Param: VEL_NOISE
    // @DisplayName:
    // @Description:
    // @Units:
    // @Range:
    // @Increment:
    // @User: Advanced
    AP_GROUPINFO("VEL_NOISE", 3, YP_Radio, _vel_measure_noise, YP_GPS_VEL_MES_NOISE),

    // @Param: FWD_SCALAR
    // @DisplayName:
    // @Description:
    // @Units:
    // @Range:
    // @Increment:
    // @User: Advanced
    AP_GROUPINFO("FWDSCALAR", 4, YP_Radio, _forward_pos_scalar, 0.225f),

    AP_GROUPEND
};

// constructor
YP_Radio::YP_Radio(AP_AHRS &ahrs, 
				   AP_BattMonitor &battery, 
				   Compass &compass, 
				   AP_Mission &mission, 
				   AC_Fence &fence,
				   AC_WPNav &wp_nav,
				   GimbalCamera &gimbalcamera, 
				   AC_Circle &ac_circle,
				   struct Location& cur_loc) :
	_initialised(false),
	sending_bind_WIFI_flags(false),
	_cur_loc(cur_loc),
	_ahrs(ahrs),
	_battery(battery),
	_compass(compass),
	_mission(mission),
	_fence(fence),
	_wp_nav(wp_nav),
	_gimbalcamera(gimbalcamera),
	_ac_circle(ac_circle),
	_gps(ahrs.get_gps()),
	_ins(ahrs.get_ins()),
	_baro(ahrs.get_baro())
{
    AP_Param::setup_object_defaults(this, var_info);

    // initiate variables
    _rssi = 0;
	_rx_count = 0;
	_channel_count = 0;
	memset((char *)&_channels, 0, sizeof(_channels));
	memset((char *)&_channels_raw, 0, sizeof(_channels_raw));
	memset((char *)&_gps_data, 0, sizeof(_gps_data));
	memset((char *)&_last_gps_data, 0, sizeof(_last_gps_data));
	memset((char *)&_radio_channels, 0, sizeof(_radio_channels));
	memset((char *)&_radio_channels_raw, 0, sizeof(_radio_channels_raw));
	memset((char *)&_radio_gps_data, 0, sizeof(_radio_gps_data));
	memset((char *)&_radio_last_gps_data, 0, sizeof(_radio_last_gps_data));
	_last_channel_update_time = 0;
	_last_radio_channel_update_time = 0;
	_last_wifi_channel_update_time = 0;
	_last_gps_update_time = 0;
	_new_gps_data = false;
	_current_channel_type = CHANNEL_TYPE_NONE;
	memset((char *)&_position, 0, sizeof(_position));
	_velocity_NE.zero();
	_high_compass_velocity_variance = false;
	memset((char *)&_front_distance, 0, sizeof(_front_distance));
	_have_distance_data = true;
	_timestamp_telemetry = 0;
	_timestamp_action_feedback = 0;
	_timestamp_front_distance = 0;
    _airline_type = AIRLINE_TYPE_STRAIGHT;
    _airline_ending_action = AIRLINE_ENDING_SUSPEND;
    _airline_default_speed = 0;
	_airline_altitude_type = 0;
	_pan_mode = Pan_FollowRoute;
	_tilt_mode = Tilt_Manual;
	memset((char *)&_txpacket, 0, sizeof(_txpacket));
	memset((char *)&_rxpacket, 0, sizeof(_rxpacket));
	memset((char *)&_telem_data, 0, sizeof(_telem_data));
	memset((char *)&_bind_cmd, 0, sizeof(_bind_cmd));
	memset((char *)&_action_response, 0, sizeof(_action_response));
	memset((char *)&_action_settingccc, 0, sizeof(_action_settingccc));
	memset((char *)&_action_feedback, 0, sizeof(_action_feedback));
	_last_index = 0;
   	_mode_type = AUTO_NONES;
	_action_request = ACTION_REQUEST_START_MANUAL;
	_action_state = ACTION_STATUS_IDLE;
	_running_action_type = ACTION_TYPE_NONE;
	_setting_count = 0;
	_last_send_response_timestamp = 0;
	mission_state = MissionState_Stop;
	_map_waypoint_set_type = map_waypoint_set_type_none;
	bind_step = 0;
	bind_step_WIFI = 0;
	bind_time = 0;
	sending_bind_WIFI_flags = false;
	bind_ack_flags = false;
	_switch_mode = Switch_Mode_None;
	_motor_failure_mask = 0;
	_last_kalman_update_time = 0;
	_variance = 0.0f;
	_kf_initialised = false;
	memset((char *)&_origin_loc, 0, sizeof(_origin_loc));
	memset((char *)&_kf_position, 0, sizeof(_kf_position));
	_kf_velocity_NE.zero();
	_vehicle_last_kalman_update_time = 0;
	_vehicle_variance = 0.0f;
	_vehicle_kf_initialised = false;
	memset((char *)&_vehicle_origin_loc, 0, sizeof(_vehicle_origin_loc));
	memset((char *)&_vehicle_kf_position, 0, sizeof(_vehicle_kf_position));
	_vehicle_kf_velocity_NE.zero();
}

void YP_Radio::init(AP_HAL::UARTDriver *port)
{
#if DRIVER_LEVEL_CHECK == 1
	if (port == NULL) {
		hal.scheduler->panic("No UART port for YP radio, Please specify one!");
		return ; // never reach
	}
#endif
	_port = port;
	_port->begin(115200, 512, 512);
	_port->set_blocking_writes(false);

	/* power on YP radio */
	hal.gpio->pinMode(YP_RADIO_POWER_PIN, HAL_GPIO_OUTPUT);
	hal.gpio->write(YP_RADIO_POWER_PIN, 0);

	/* radio init successfully */
	_initialised = true;
}

void YP_Radio::check_input(void)
{
	// check if uart port is available
	int16_t nbytes = _port->available();
	for (int i = 0; i < nbytes; i++) {
		uint8_t byte = _port->read();
		// decode the packet
		uint8_t ret = _YP_Frame_Sync(YPRC_PORT_NUM, byte, &_rxpacket);
		// if success, handle packet actiontype
		if (ret == SYNC_SUCCESS) _YP_Handle_Packet(&_rxpacket);
	}

	// check if radio channel timeout
	uint32_t tnow = hal.scheduler->millis();
	uint32_t wifi_channel_update_time = _gimbalcamera.last_wifi_channel_update_time();
	if (tnow - _last_radio_channel_update_time < RADIO_CHANNEL_UPDATE_TIME) {
		if (_last_channel_update_time != _last_radio_channel_update_time) {
			// update channel value
			for (uint8_t chan_index = 0; chan_index < YP_RC_INPUT_CHANNEL; chan_index++) {
				// copy channel's raw value.
				_channels_raw[chan_index] = _radio_channels_raw[chan_index];
				/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
				_channels[chan_index] = (uint16_t)(_channels_raw[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
			}

			// get GPS info
			_gps_data = _radio_gps_data;

			// check if GPS info updated
			if (_last_gps_data.lat != _gps_data.lat || _last_gps_data.lon != _gps_data.lon || _last_gps_data.alt != _gps_data.alt || _last_gps_data.course != _gps_data.course || _last_gps_data.speed != _gps_data.speed) {
				// tell the simple kf filter, we get new rc gps data to use
				_new_gps_data = true;
				_last_gps_update_time = tnow;
				memcpy((char *)&_last_gps_data, (const char *)(&_gps_data), sizeof(StGpsDataType));
			}

			// set channel update time to radio channel update time
			_last_channel_update_time = _last_radio_channel_update_time;
			// set current channel type
			_current_channel_type = CHANNEL_TYPE_RADIO;
		}
	// check if WIFI channel is available
	} else if (tnow - wifi_channel_update_time < WIFI_CHANNEL_UPDATE_TIME) {
		if (_last_channel_update_time != wifi_channel_update_time) {
			// update channel value
			for (uint8_t chan_index = 0; chan_index < YP_RC_INPUT_CHANNEL; chan_index++) {
				// copy channel's raw value.
				_channels_raw[chan_index] = _gimbalcamera.get_raw_channel(chan_index);
				/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
				_channels[chan_index] = (uint16_t)(_channels_raw[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
			}

			// get GPS info
			_gimbalcamera.get_gps_data(&_gps_data);

			// check if GPS info updated
			if (_last_gps_data.lat != _gps_data.lat || _last_gps_data.lon != _gps_data.lon || _last_gps_data.alt != _gps_data.alt || _last_gps_data.course != _gps_data.course || _last_gps_data.speed != _gps_data.speed) {
				// tell the simple kf filter, we get new rc gps data to use
				_new_gps_data = true;
				_last_gps_update_time = tnow;
				memcpy((char *)&_last_gps_data, (const char *)(&_gps_data), sizeof(StGpsDataType));
			}

			// set channel update time to radio channel update time
			_last_channel_update_time = wifi_channel_update_time;
			// set current channel type
			_current_channel_type = CHANNEL_TYPE_WIFI;
		}
	} else {
		// set current channel type
		_current_channel_type = CHANNEL_TYPE_NONE;
	}
}

bool YP_Radio::position_ok() {
	uint32_t tnow = hal.scheduler->millis();
	uint32_t update_time_elapse = tnow - _last_gps_update_time;
#if 0
	if (_current_channel_type == CHANNEL_TYPE_RADIO) {
		hal.console->printf("Radio Connected --- ");
	} else if (_current_channel_type == CHANNEL_TYPE_WIFI) {
		hal.console->printf("Wifi Connected --- ");
	}
	hal.console->printf("rc_type: %d period: %u sats: %d fix_type: %d hacc: %.2f speed: %.2f course: %.2f lat: %u lon: %u\n", get_rc_type(), update_time_elapse, get_num_sats(), get_fix_type(), get_accuracy(), get_speed(), get_course(), _gps_data.lat, _gps_data.lon);
#endif
	// if GPS is 3D fixed, return true
	uint32_t rc_gps_timeout_ms = 2000;
	if (_current_channel_type == CHANNEL_TYPE_WIFI) {
		rc_gps_timeout_ms = 5000;
	}
	if (((update_time_elapse < (uint32_t)rc_gps_timeout_ms)) && (get_num_sats() >= 10) && (get_accuracy() <= 100.0f) && !(_gps_data.lat == 0 && _gps_data.lon == 0)) {
		return true;
	// if RC GPS is lost, return false
	} else {
		return false;
	}
}

/*
 * send telemetry frames. Should be called at 50Hz.
 */
void YP_Radio::update_radio(void)
{
    if (!_initialised)	return;

    uint32_t tnow = hal.scheduler->millis();

    // send telemetry data - 50Hz
    if ((tnow - _timestamp_telemetry) >= YPRC_DATARATE_TELEMETRY) {
    	_timestamp_telemetry = tnow;
        _send_telemetry();
    }

    // send action feedback - 3Hz
    if ((tnow - _timestamp_action_feedback) >= YPRC_DATARATE_ACION_FEEDBACK) {
    	_timestamp_action_feedback = tnow;
    	send_action_feedback();
    }

    // send sonar distance - 4Hz
    if ((tnow - _timestamp_front_distance) >= YPRC_DATARATE_FRONT_DISTANCE) {
    	_timestamp_front_distance = tnow;
    	_send_front_distance();
    }

	// send wp mission feedback -2Hz
	if(tnow - _timestamp_mission_feedback >= YPRC_DATARATE_MISSION_FEEDBACK) {
		_timestamp_mission_feedback = tnow;
		_handle_wp_mission_feedback_send();
	}

    // check switch mode
    _check_switch_mode();

    // update key function
    _update_key_function();
}

// check copter's attitude send "BIND" command to radio
void YP_Radio::check_if_need_binding()
{	
	if (hal.rcin->new_input() && !sending_bind_WIFI_flags) {
		return;
	} else {
		uint32_t tnow = hal.scheduler->millis();

		if (sending_bind_WIFI_flags == 0){
			bind_step_WIFI = 0;
			switch (bind_step) {
			case 0:
				if (_ahrs.pitch_sensor < -2500) {
					bind_step = 1;
					bind_time = tnow;
				}
				break;
			case 1:
				if (_ahrs.pitch_sensor > -1000)
					bind_step = 2;
				// timeout check
				if ((tnow - bind_time) > YUNEEC_TELEM_BIND_TIME_LIMIT)
					bind_step = 0;
				break;
			case 2:
				if (_ahrs.pitch_sensor < -2500)
					bind_step = 3;
				// timeout check
				if ((tnow - bind_time) > YUNEEC_TELEM_BIND_TIME_LIMIT)
					bind_step = 0;
				break;
			case 3:
				if (_ahrs.pitch_sensor > -1000)
					bind_step = 4;
				// timeout check
				if ((tnow - bind_time) > YUNEEC_TELEM_BIND_TIME_LIMIT) {
					bind_step = 0;
					bind_time = tnow;
				}
				break;
			case 4:
				// power off YP radio first
				hal.gpio->write(YP_RADIO_POWER_PIN, 1);
				bind_step = 5;
				break;
			case 5:
				// power on YP radio
				hal.gpio->write(YP_RADIO_POWER_PIN, 0);
				bind_step = 6;
				break;
			case 6:
			case 7:
			case 8:
			case 9:
				// wait a moment
				bind_step++;
				break;
			case 10:
				// notify user that we are in bind mode now
				AP_Notify::startup_flags.in_bind_mode = true;
				// send bindcmd 5 times
				_send_bind();
				bind_step = 0;
				break;
			default:
				bind_step = 0;
				break;
			}
		} else if (sending_bind_WIFI_flags == 1) 		// Notify that this flag for switch between single or dual remote control, enter the BIND mode again.
		{
			switch (bind_step_WIFI) {
			case 0:
				// power off YP radio first
				hal.gpio->write(YP_RADIO_POWER_PIN, 1);
				bind_step_WIFI = 1;
				break;
			case 1:
				// power on YP radio
				hal.gpio->write(YP_RADIO_POWER_PIN, 0);
				bind_step_WIFI = 2;
				break;
			case 2:
			case 3:
			case 4:
			case 5:
				// wait a moment
				bind_step_WIFI++;
				break;
			case 6:{
				// notify user that we are in bind mode now
				AP_Notify::startup_flags.in_bind_mode = true;
				// send bindcmd 5s
				_send_bind();
				bind_step_WIFI++;
				break;}
			case 7 ... 17:
				bind_step_WIFI++;
				break;
			default:
				if (tnow - _last_radio_channel_update_time < RADIO_CHANNEL_UPDATE_TIME) {
					bind_step_WIFI = 0;
					bind_ack_flags = true;
					sending_bind_WIFI_flags = 0;
				}
				break;
			}
		}
	}
}

// check which mode we are switching in
void YP_Radio::_check_switch_mode(void)
{
	static uint32_t mode_change_timestamp = 0;

	uint32_t tnow = hal.scheduler->millis();
	if (tnow - mode_change_timestamp > 100) {
		// get the corresponding switch mode
		enum Switch_Mode switch_mode = Switch_Mode_None;

#if USE_STICK_SWITCH == 1
		static uint8_t switch_state = 0;
		static uint8_t flip_times = 0;
		static uint32_t timestamp = 0;
#if USE_THROTTLE_YAW_STICK_SWITCH == 1
		// get throttle/yaw input channel value
		uint16_t throttle_in = hal.rcin->read(0);
		uint16_t yaw_in = hal.rcin->read(3);
		if (throttle_in < 1350 && yaw_in < 1350)		switch_mode = Switch_Mode_None;		// right downside 	- TBD
		else if (throttle_in > 1650 && yaw_in < 1350)	switch_mode = Switch_Mode_None;		// right upside 	- TBD
		else if (throttle_in > 1650 && yaw_in > 1650)	switch_mode = Switch_Mode_None;		// left upside		- TBD
		else if (throttle_in < 1350 && yaw_in > 1650)	switch_mode = Switch_Mode_IPSOnly;	// left downside 	- IPS only flight mode
#endif
#if USE_ROLL_PITCH_STICK_SWITCH == 1
		// get roll/pitch input channel value
		uint16_t roll_in  = hal.rcin->read(1);
		uint16_t pitch_in = hal.rcin->read(2);
		if (roll_in < 1200 && pitch_in < 1200)		switch_mode = Switch_Mode_GPSDisable;	// right downside 	- gps disable
		else if (roll_in < 1200 && pitch_in > 1800)	switch_mode = Switch_Mode_GPSCalibrate;	// right upside 	- gps calibrate
		else if (roll_in > 1800 && pitch_in > 1800)	switch_mode = Switch_Mode_IMUCalibrate;	// left upside 		- imu calibrate
		else if (roll_in > 1800 && pitch_in < 1200)	switch_mode = Switch_Mode_MAGCalibrate;	// left downside 	- mag calibrate
#endif

		// ready to change to Attitude mode if high compass and velocity variance for emergency
		if (_high_compass_velocity_variance) {
			switch_mode = Switch_Mode_GPSDisable;
		}

		// check switch state
		if (switch_mode != Switch_Mode_None) {
			uint16_t switch_in = hal.rcin->read(4);
			switch (switch_state) {
			case 0:
				// switch reach down side
				if (switch_in < 1200) {
					timestamp = tnow;
					switch_state = 2;
				}
				break;
			case 1:
				// switch reach down side
				if (switch_in < 1200) {
					if (tnow - timestamp <= YUNEEC_SWITCH_MODE_TIMEOUT) {
						switch_state = 2;
					} else {
						flip_times = 0;
						switch_state = 0;
					}
				}
				break;
			case 2:
				// switch reach up side
				if (switch_in > 1800) {
					if (tnow - timestamp <= YUNEEC_SWITCH_MODE_TIMEOUT) {
						flip_times++;
						switch_state = 1;
					} else {
						flip_times = 0;
						switch_state = 0;
					}
				}
				break;
			}
			// switch fliped 3 times
			if (flip_times >= 3) {
				switch_state = 0;
				flip_times = 0;
			} else {
				switch_mode = Switch_Mode_None;
			}

		} else {
			switch_state = 0;
			flip_times = 0;
		}
#endif

		// read switch mode from channel 10
		uint16_t button_value = _channels_raw[YP_SCREEN_BUTTON_FUNCTION_CHANNEL];
		switch (button_value) {
		case 4:
			switch_mode = Switch_Mode_GPSDisable;
			break;
		case 8:
			switch_mode = Switch_Mode_GPSEnable;
			break;
		case 12:
			switch_mode = Switch_Mode_MAGCalibrate;
			break;
		case 16:
			switch_mode = Switch_Mode_IMUCalibrate;
			break;
		case 20:
			switch_mode = Switch_Mode_LandingGear;
			break;
		case 24:
			// reserved for gimbal calibration
			break;
		case 28:
			switch_mode = Switch_Mode_DisableFollowMe;
			break;
		case 32:
			switch_mode = Switch_Mode_DisableWatchMe;
			break;
		case 36:
			switch_mode = Switch_Mode_EnableWatchMe;
			break;
		case 40:
			switch_mode = Switch_Mode_EnableCA;
			break;
		case 44:
			switch_mode = Switch_Mode_DisableCA;
			break;
		case 48: // For Camera: take photo
		case 42: // For Camera: Video on/off
			break;
		case 56:
			switch_mode = Switch_Mode_3DFollow;
			break;
		default:
			break;
		}

		// update switch mode
		if (switch_mode != Switch_Mode_None) {
			mode_change_timestamp = tnow;
		}

		_switch_mode = switch_mode;
	}
}

// update key function
// for landing gear
void YP_Radio::_update_key_function(void) {
	uint16_t key_value = hal.rcin->read(YP_KEY_FUNCTION_CHANNEL);

	if (key_value > (uint16_t)1800) {
		key_function.landing_gear_retract = false;
	} else if (key_value < (uint16_t)1200) {
		key_function.landing_gear_retract = true;
	}

	switch (_switch_mode) {
	case Switch_Mode_EnableCA:
		key_function.ca_on = true;
		break;
	case Switch_Mode_DisableCA:
		key_function.ca_on = false;
		break;
	default:
		break;
	}
}

void YP_Radio::_send_bind(void) {
	/* Send bind command 5 times */
	_bind_cmd = {0, {'B', 'I', 'N', 'D'}};
	_pack_and_send(YPRC_PACKET_TYPE_BINDCMD, 5);
}

void YP_Radio::_pack_and_send(YPRC_PACKET_TYPE packet_type, uint8_t send_count)
{
	switch(packet_type){
	case YPRC_PACKET_TYPE_BINDCMD:
	{
		StBindCmd bind_cmd = {0, {'B', 'I', 'N', 'D'}};
		_txpacket.header1 = YPRC_STX1;
		_txpacket.header2 = YPRC_STX2;
		_txpacket.length = PACKET_LENGTH_STBINDCMD + 2;
		_txpacket.type = packet_type;
		memcpy(_txpacket.yprc_data, (const uint8_t *)&_bind_cmd, PACKET_LENGTH_STBINDCMD);
		_txpacket.yprc_data[PACKET_LENGTH_STBINDCMD] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
		for (int i = 0; i < send_count; i++) {
			_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
		}
		break;
	}
	case YPRC_PACKET_TYPE_TELEMETRYDATA:
	{
		// format data
		_txpacket.header1 = YPRC_STX1;
		_txpacket.header2 = YPRC_STX2;
		_txpacket.length = PACKET_LENGTH_TELEMETRYDATA + 2;
		_txpacket.type = packet_type;
		memcpy(_txpacket.yprc_data, (const uint8_t *)&_telem_data, PACKET_LENGTH_TELEMETRYDATA);
		_txpacket.yprc_data[PACKET_LENGTH_TELEMETRYDATA] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
		_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
		break;
	}
	default:
		break;
	}
}

void  YP_Radio::_send_telemetry()
{
	static uint32_t start_time = 0;
	static bool startup_delay = true;

	uint32_t tnow = hal.scheduler->millis();

	// record the time start to send telemetry
	if (start_time == 0) start_time = tnow;

	// delay 5s to report health info of all sensors to prevent fake unhealthy during startup
	if (startup_delay) {
		if ((tnow - start_time) > (uint32_t)60000) startup_delay = false;
	}

    /////////////////////////////////////////////////////////////////////
	// GPS info
	// if position is okay, then send GPS information to RC
    Vector3f velocity = _gps.velocity() * 100; // convert from m/s to cm/s
	_telem_data.lat = _cur_loc.lat;	// latitude
	_telem_data.lon = _cur_loc.lng;	// longitude
	_telem_data.alt = _cur_loc.alt;
	_telem_data.vx 	= velocity.x;	// north velocity
	_telem_data.vy 	= velocity.y;	// east velocity
	_telem_data.vz 	= velocity.z;	// down velocity
   	_telem_data.nsat = _gps.num_sats();// number of satellite
    if (AP_Notify::flying_flags.ready_to_fly && vehicle_state.flight_mode != Flight_Mode_ANGLE_GPS_OFF) {
		_telem_data.nsat |= 0x80;
    }

    /////////////////////////////////////////////////////////////////////
    // Battery info
    _telem_data.voltage = (_battery.voltage() - 5.0f) * 10.0f;
    _telem_data.current = 0;	// not available now

    /////////////////////////////////////////////////////////////////////
    // Attitude info
    _telem_data.roll  = (int16_t)_ahrs.roll_sensor;
    _telem_data.pitch = (int16_t)_ahrs.pitch_sensor;
    uint32_t yaw = _ahrs.yaw_sensor;
    if (yaw > 18000) yaw -= 36000;
	_telem_data.yaw = (int16_t)(9000-yaw);

    /////////////////////////////////////////////////////////////////////
	// Motor info
	uint8_t motorStatus = 0;
	if (_motor_failure_mask & 0x04) motorStatus |= 0x01;
	if (_motor_failure_mask & 0x02) motorStatus |= 0x02;
	if (_motor_failure_mask & 0x20) motorStatus |= 0x04;
	if (_motor_failure_mask & 0x08) motorStatus |= 0x08;
	if (_motor_failure_mask & 0x01) motorStatus |= 0x10;
	if (_motor_failure_mask & 0x10) motorStatus |= 0x20;

	_telem_data.motorStatus = motorStatus ^ 0xFF;

    /////////////////////////////////////////////////////////////////////
	// IMU health info
	uint8_t status = 0;
	if (_ins.healthy())
		status |= 0x01;
	if (_gimbalcamera.get_camera_record_status())
		status |= 0x04;
    // set GPS used flag to 1 if GPS is available and user didn't switch it off
    if (vehicle_state.flight_mode != Flight_Mode_ANGLE_GPS_OFF) {
    	status |= 0x20;
        // set using RC GPS flag to 1 if both available
    	if (AP_Notify::flying_flags.ready_to_fly && position_ok()) {
    	    status |= 0x80;
    	}
    }
	// sonar mode feedback
	if (AP_Notify::flying_flags.ca_mode > 0)
		status |= 0x10;
	_telem_data.imuStatus = status | 0x40;

    /////////////////////////////////////////////////////////////////////
	// Compass, barometer and GPS health info
	status = 0;
	if (_baro.healthy())
		status |= 0x01;
	if (_compass.healthy())
		status |= 0x04;
	// GPS info will show later
	if (startup_delay)			status |= 0x10;
	else if (_gps.status() > 0)	status |= 0x10;
#if defined(YUNEEC_PRODUCT_TYPHOONH) || defined(YUNEEC_PRODUCT_TYPHOONH_PLUS)
	// RealSense health flag
	if (AP_Notify::startup_flags.realsense_ok)
		status |= 0x20;
	// Sonar health flag
	if (AP_Notify::startup_flags.sonar_ca_ok)
		status |= 0x40;
	// OpticalFlow health flag
	if (AP_Notify::startup_flags.optflow_ok)
		status |= 0x80;
#elif defined(YUNEEC_PRODUCT_TORNADO)
	status |= 0x20;
	status |= 0x40;
	status |= 0x80;
#else
#error Please specify YUNEEC_PRODUCT_XXX
#endif
	_telem_data.baroMagGpsStatus = status;

    /////////////////////////////////////////////////////////////////////
	// Error flag
	status = 0;
	if (_battery.voltage() <= _battery.warning_level1_voltage() && (_battery.voltage() > _battery.warning_level2_voltage()))
		status |= 0x01;
	else if ((_battery.voltage() <= _battery.warning_level2_voltage()))
		status |= 0x02;
	if (vehicle_state.imu_preheating)
		status |= 0x10;
    if (AP_Notify::diagnose_flags.mag_require_cal)
    	status |= 0x20;
	if (AP_Notify::startup_flags.no_fly_zone)
		status |= 0x80;
	_telem_data.errorFlags = status;

    /////////////////////////////////////////////////////////////////////
	// Vehicle state flag
	if (vehicle_state.emergency_shutdown) {
		_telem_data.flightMode = FMODE_EMERGENCY_SHUTDOWN;
	} else {
		switch (vehicle_state.pre_flight_status) {
		case Pre_Flight_Status_Not_Ready:
			// TODO: show motor/sensor failure
			break;
		case Pre_Flight_Status_Ready:
			switch (vehicle_state.calibration_type) {
			case Calibration_Type_None:
				_telem_data.flightMode = FMODE_READY_TO_FLY;
				break;
			case Calibration_Type_IMU:
				_telem_data.flightMode = FMODE_IMU_ABIAS_CALIBRATION;
				break;
			case Calibration_Type_MAG:
				_telem_data.flightMode = FMODE_MAG_CALIBRATION;
				break;
			}
			break;
		case Pre_Flight_Status_Arming:
			_telem_data.flightMode = FMODE_ARMING_MOTORS;
			break;
		case Pre_Flight_Status_Armed:
			switch (vehicle_state.flight_mode) {
			case Flight_Mode_THR:
				_telem_data.flightMode = FMODE_THR_MODE;
				break;
			case Flight_Mode_THR_GPS_OFF:
				_telem_data.flightMode = FMODE_THR_MODE_GPS_OFF;
				break;
			case Flight_Mode_THR_NO_GPS:
				_telem_data.flightMode = FMODE_THR_MODE_NO_GPS;
				break;
			case Flight_Mode_ANGLE:
				_telem_data.flightMode = FMODE_ANGLE_MODE;
				break;
			case Flight_Mode_ANGLE_IPS_ONLY:
				_telem_data.flightMode = FMODE_ANGLE_MODE_IPS_ONLY;
				break;
			case Flight_Mode_ANGLE_GPS_OFF:
				_telem_data.flightMode = FMODE_ANGLE_MODE_GPS_OFF;
				break;
			case Flight_Mode_ANGLE_NO_GPS:
				_telem_data.flightMode = FMODE_ANGLE_MODE_NO_GPS;
				break;
			case Flight_Mode_SMART:
				_telem_data.flightMode = FMODE_SMART_MODE;
				break;
			case Flight_Mode_SMART_NO_GPS:
				_telem_data.flightMode = FMODE_SMART_MODE_NO_GPS;
				break;
			case Flight_Mode_ACRO:
				_telem_data.flightMode = FMODE_ACRO_MODE;
				break;
			case Flight_Mode_FOLLOW:
				_telem_data.flightMode = FMODE_FOLLOW_MODE;
				break;
			case Flight_Mode_FOLLOW_NO_RC_GPS:
				_telem_data.flightMode = FMODE_SMART_MODE;
				break;
			case Flight_Mode_WATCHME:
				_telem_data.flightMode = FMODE_WATCHME_MODE;
				break;
			case Flight_Mode_WATCHME_NO_RC_GPS:
				_telem_data.flightMode = FMODE_WATCHME_MODE_NO_RC_GPS;
				break;
			case Flight_Mode_RETURN_TO_HOME:
				_telem_data.flightMode = FMODE_RETURN_TO_HOME;
				break;
			case Flight_Mode_LAND:
				_telem_data.flightMode = FMODE_RTH_FINAL_LANDING;
				break;
			case Flight_Mode_3D_FOLLOW:
				_telem_data.flightMode = FMODE_3D_FOLLOW_WIZARD;
				break;
			case Flight_Mode_3D_WATCHME:
				_telem_data.flightMode = FMODE_3D_WATCHME_WIZARD;
				break;
			}
			switch (_mode_type) {
				case AUTO_NONES: break;
				case AUTO_JOUR: 
					_telem_data.flightMode = FMODE_JOURNEY_MODE;
					break;
				case AUTO_POI:
					_telem_data.flightMode = FMODE_POI_MODE;
					break;
				case AUTO_ORBIT:
					_telem_data.flightMode = FMODE_ORBIT_MODE;
					break;
				case AUTO_CURVECABLECAM:
					_telem_data.flightMode = FMODE_CCC_MODE;
					break;
				case AUTO_WAYPOINT:
				case AUTO_MAP_WAYPOINT:
					_telem_data.flightMode = FMODE_WAYPOINT;
					break;
				default:
					break;
			}
			break;
		}
	}

    /////////////////////////////////////////////////////////////////////
	// Vehicle type indicator
	_telem_data.vehicleType = YP_RADIO_VEHICLE_TYPE;

    /////////////////////////////////////////////////////////////////////
	// GPS horizontal accuracy in 0.05m resolution
	float horizontal_acc;
	if (_gps.horizontal_accuracy(horizontal_acc))
		_telem_data.gpsAccH = horizontal_acc * 20;
	else
		_telem_data.gpsAccH = 0;

	// add packet count and send this package
	_telem_data.t++;
	// send YPRC_PACKET_TYPE_TELEMETRYDATA packet 1 time
	_pack_and_send(YPRC_PACKET_TYPE_TELEMETRYDATA, 1);

#if 0
	hal.uartA->printf("YPTelemetry:\n lat:%ld lon:%ld alt:%ld vx:%u vy:%d vz:%d nsat:%u vol:%u cur:%u roll:%d pitch:%d yaw:%d mS:%u iS%u bmgS:%u fM:%u vT:%u eF:%u gpsH:%u\n",
		    _telem_data.lat,// lat,
		    _telem_data.lon,// lon,
		    _telem_data.alt,// alt,
		    _telem_data.vx,// vx,
		    _telem_data.vy,// vy,
		    _telem_data.vz,// vz,
		    _telem_data.nsat,// nsat,
		    _telem_data.voltage,// voltage,
		    _telem_data.current,// current,
		    _telem_data.roll,// roll,
		    _telem_data.pitch,// pitch,
		    _telem_data.yaw,// yaw,
		    _telem_data.motorStatus,// motorStatus,
		    _telem_data.imuStatus,// imuStatus,
		    _telem_data.baroMagGpsStatus,// baroMagGpsStatus,
		    _telem_data.flightMode,// flightMode,
		    _telem_data.vehicleType,// vehicleType,
		    _telem_data.errorFlags,// errorFlags,
		    _telem_data.gpsAccH);// gpsAccH);
#endif
}

void YP_Radio::_handle_action_response(ActionResponse* packet_in)
{
	if (packet_in->actionResult == ACTION_RESULT_OK){
		DEBUG_PRINTF("OK for feedback response");
	}
}

void YP_Radio::_handle_action_settingccc(ActionSettingCCC* packet_in)
{
	uint8_t index = 0;
	uint8_t total_index = 0;

	// check if the type state, if wrong then return
	if (_mode_type != AUTO_CURVECABLECAM || _action_state != ACTION_STATUS_SETTING) {
		send_response(ACTION_RESULT_ERR_FAIL, 0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		DEBUG_PRINTF("no good for receive");
		return;
	}

	DEBUG_PRINTF("receive ccc point");

	index = packet_in->actionIndex; 

	if (index > _setting_count) {
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		send_response(ACTION_RESULT_ERR_FAIL,0);
		return;
	}

	total_index = _mission.get_current_cable_cam_point_total();

	// when total_index is 2, we got 2 points, now we get from rc index is 2, so we enter in adding process
	if ( total_index == index ) {
		AP_Mission::cableCamPoint cable_cam_point;
		cable_cam_point.location.lat = packet_in->lat;
		cable_cam_point.location.lng = packet_in->lon;
		cable_cam_point.location.alt = packet_in->alt;

		// convert from <-18000,+18000> to <0,36000>
	    int32_t yaw = (int32_t)packet_in->yaw;
	    if (yaw < 0) yaw += 36000;
		cable_cam_point.copter_yaw   = yaw;
		cable_cam_point.gimbal_yaw   = yaw;
		cable_cam_point.gimbal_pitch = packet_in->gimbal_pitch;	

		// add current cable cam point
		if (_mission.add_cable_cam_point(cable_cam_point)) {
			// notify user add one point successfully
			AP_Notify::flying_flags.ccc_add_point = true;
			DEBUG_PRINTF("CCC STATE: add one point\n");
			send_response(ACTION_RESULT_OK, index);	
			// when we got adding process is good
			_last_index = index;	
		} else {
			_action_state = ACTION_STATUS_IDLE;
			_mode_type = AUTO_NONES;
			send_response(ACTION_RESULT_ERR_FAIL,0);
		}
	// To ensure second handshake is ok , when we lost one response for ok
	} else if (_last_index == index) {
		send_response(ACTION_RESULT_OK, index);	
	} else {
		send_response(ACTION_RESULT_ERR_FAIL,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		DEBUG_PRINTF("fail the index check!");
	}
}

void YP_Radio::_handle_action_settingroi(ActionSettingROI* packet_in, roi_config_fn roi_config)
{
	Vector3ul center;
	float radius;	// unit cm
	
	// check if the type state ,if wrong return
	if ( _mode_type != AUTO_POI || _action_state != ACTION_STATUS_SETTING) {
		send_response(ACTION_RESULT_ERR_FAIL,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		return;
	}

	radius   = packet_in->radius;
	center.x = packet_in->lat;
	center.y = packet_in->lon;
	center.z = (int32_t)(packet_in->alt/10.0f);

	//if the config of roi is ok, keep the center point in memory
	if (roi_config(center, radius)) {
		DEBUG_PRINTF("GOT CENTER GOOD");
		send_response(ACTION_RESULT_OK, 1);
	} else {
		send_response(ACTION_RESULT_ERR_FAIL,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
	}
}

void YP_Radio::_handle_action_settingjour(ActionSettingJOUR* packet_in, jour_config_fn jour_config)
{
	if (_mode_type != AUTO_JOUR || _action_state != ACTION_STATUS_SETTING) {
		send_response(ACTION_RESULT_ERR_FAIL,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		return;
	}
	uint8_t distance = packet_in->maxDistance;
	uint8_t speed = packet_in->maxSpeed;

	if (jour_config(distance, speed)) {
		// count is 1, indicate that we got a good point
		send_response(ACTION_RESULT_OK, 1);
	} else {
		send_response(ACTION_RESULT_ERR_FAIL,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
	}
}

void YP_Radio::_handle_action_takeoff(ActionTakeOff* packet_in, motors_auto_arming_fn motors_auto_arming)
{
	uint8_t speed, height;
	// unit is m/s
	speed = packet_in->takeoffSpeed;
	// unit is m
	height = packet_in->takeoffHeight;
	// if need manual armed before takeoff
	bool manual_armed_before_takeoff = true;

	// auto arm succeed, reponse ok to controller
	if (manual_armed_before_takeoff && motors_auto_arming(speed, height)) {
		if (packet_in->actionType == ACTION_TYPE_TAKEOFF) {
			send_takeoff_landing_response(ACTION_TYPE_TAKEOFF, ACTION_REQUEST_CONFIG, ACTION_RESULT_OK);
		} else {
			send_response(ACTION_RESULT_OK, 0);
		}
	} else {
		if (packet_in->actionType == ACTION_TYPE_TAKEOFF) {
			send_takeoff_landing_response(ACTION_TYPE_TAKEOFF, ACTION_REQUEST_CONFIG, ACTION_RESULT_ERR_FAIL);
		} else {
			send_response(ACTION_RESULT_ERR_FAIL,0);
		}
	}
	return;
}

void YP_Radio::send_action_feedback(void)
{
	static uint32_t _last_action_time = 0;

	uint32_t tnow = hal.scheduler->millis();

	if (_mode_type == AUTO_NONES && (tnow - _last_action_time > 8000)) {
		return;
	} else {
		_last_action_time = tnow;
	}

	ActionFeedback tx;
	tx.actionType = ACTION_TYPE_FEEDBACK;
	tx.action_state = _action_state;
	tx.mode_type = _mode_type;

	// to follow the order of controller display
	if ((_action_state == ACTION_STATUS_RUNNING || _action_state == ACTION_STATUS_STOPPED)) {
		// start from 1 to n, for the point-rule of receiver
		uint16_t total_index = _mission.get_current_cable_cam_point_total();
		tx.degree_of_completion = _mission.get_current_nav_index() - 1;
	} else {
		tx.degree_of_completion = 0;
	}
	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_ACTIONFEEDBACK + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONFEEDBACK);
	_txpacket.yprc_data[PACKET_LENGTH_ACTIONFEEDBACK] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::_send_front_distance(void)
{
	ActionFrontDistance tx;

	tx.actionType = ACTION_TYPE_FRONT_DISTANCE;
	if (_have_distance_data) {
		_have_distance_data = false;
		memcpy(tx.front_distance, (const uint8_t *)&_front_distance, REALSENSE_OBSTACLE_DISTANCE_INFO_SIZE);
	} else {
		return;
	}

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_FRONTDISTANCE + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_FRONTDISTANCE);
	_txpacket.yprc_data[PACKET_LENGTH_FRONTDISTANCE] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::_send_gohome_config(uint16_t goHomeHeight, uint16_t goHomeConfig)
{
	ActionGoHomeConfig tx;

	tx.actionType = ACTION_TYPE_GOHOME_CONFIG;
	tx.goHomeHeight = goHomeHeight;
	tx.goHomeConfig = goHomeConfig;

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_GOHOMECONFIG + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_GOHOMECONFIG);
	_txpacket.yprc_data[PACKET_LENGTH_GOHOMECONFIG] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_bind(void)
{
	DEBUG_PRINTF("bind process");
	_send_bind();
}

void YP_Radio::send_jour_cmd(void)
{
	ActionRequest tx;
	// packet struct for action_request journey
	tx.actionType = ACTION_TYPE_REQUEST;
	tx.actionRequest = ACTION_REQUEST_START_AUTO;
	tx.modeType = AUTO_JOUR;
	tx.settingCount = 0;	

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_ACTIONREQUEST + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONREQUEST);
	_txpacket.yprc_data[PACKET_LENGTH_ACTIONREQUEST] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_circleme_start_cmd(void)
{
	ActionRequest tx;
	tx.actionType = ACTION_TYPE_REQUEST;
	tx.actionRequest = ACTION_REQUEST_START_AUTO;
	tx.modeType = AUTO_ORBIT;
	tx.settingCount = 0;	

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_ACTIONREQUEST + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONREQUEST);
	_txpacket.yprc_data[PACKET_LENGTH_ACTIONREQUEST] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_ccc_start_cmd(void)
{
	ActionRequest tx;
	// packet struct for action_request ccc
	tx.actionType = ACTION_TYPE_REQUEST;
	tx.actionRequest = ACTION_REQUEST_START_AUTO;
	tx.modeType = AUTO_CURVECABLECAM;
	tx.settingCount = 0;

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_ACTIONREQUEST + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONREQUEST);
	_txpacket.yprc_data[PACKET_LENGTH_ACTIONREQUEST] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_action_request_set_ccc(void)
{
	ActionRequest tx;
	tx.actionType = ACTION_TYPE_REQUEST;
	tx.actionRequest = ACTION_REQUEST_SET;
	tx.modeType = AUTO_CURVECABLECAM;
	//settingCount depends on how many points need to be sent
	tx.settingCount = 1;

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_ACTIONREQUEST + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONREQUEST);
	_txpacket.yprc_data[PACKET_LENGTH_ACTIONREQUEST] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}
/********* testing code ************/

void YP_Radio::send_action_setting_roi(void)
{
	ActionSettingROI tx;
	tx.actionType = ACTION_TYPE_SETTING_ROI;
	if (AP_Notify::flying_flags.ready_to_fly) {
		tx.actionResult = ACTION_RESULT_OK;
		tx.lat = _cur_loc.lat;
		tx.lon = _cur_loc.lng;
		tx.alt = _cur_loc.alt;
		tx.radius = 0;
		tx.speed = 0;
		_txpacket.header1 = YPRC_STX1;
		_txpacket.header2 = YPRC_STX2;
		_txpacket.length = PACKET_LENGTH_ACTIONSETTING_ROI + 2;
		_txpacket.type = YPRC_PACKET_TYPE_ACTION;
		memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONSETTING_ROI);
		_txpacket.yprc_data[PACKET_LENGTH_ACTIONSETTING_ROI] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
		_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
	} else {
		send_response(ACTION_RESULT_ERR_FAIL,0);
	}
}

void YP_Radio::send_action_setting_ccc(void)
{
	ActionSettingCCC tx;
	tx.actionType = ACTION_TYPE_SETTING_CCC;
	if (AP_Notify::flying_flags.ready_to_fly) {
		tx.actionResult = ACTION_RESULT_OK;
		tx.actionIndex = 0;
		tx.lat = _cur_loc.lat;
		tx.lon = _cur_loc.lng;
		tx.alt = _cur_loc.alt;
		tx.roll = 0;
		tx.pitch = 0;
		// convert from <0,36000> to <-18000,+18000>
	    int32_t yaw = _ahrs.yaw_sensor;
	    if (yaw > 18000) yaw -= 36000;
		tx.yaw = yaw;
		tx.gimbal_pitch = int16_t(_gimbalcamera.get_pitch_angle() * 100);
		tx.gimbal_yaw = tx.yaw;

		_txpacket.header1 = YPRC_STX1;
		_txpacket.header2 = YPRC_STX2;
		_txpacket.length = PACKET_LENGTH_ACTIONSETTING_CCC + 2;
		_txpacket.type = YPRC_PACKET_TYPE_ACTION;
		memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONSETTING_CCC);
		_txpacket.yprc_data[PACKET_LENGTH_ACTIONSETTING_CCC] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
		_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
	}else{
		send_response(ACTION_RESULT_ERR_FAIL,0);
	}
}

void YP_Radio::send_action_request_get_ccc(void)
{
	ActionRequest tx;
	tx.actionType = ACTION_TYPE_REQUEST;
	tx.actionRequest = ACTION_REQUEST_GET;
	tx.modeType = AUTO_CURVECABLECAM;
	tx.settingCount = 0;

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_ACTIONREQUEST + 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_ACTIONREQUEST);
	_txpacket.yprc_data[PACKET_LENGTH_ACTIONREQUEST] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_response(ActionResult_e result, uint8_t count)
{
	// limit response send rate to 20Hz
	uint32_t tnow = hal.scheduler->millis();
	if (tnow - _last_send_response_timestamp > 50) {
		_last_send_response_timestamp = tnow;
		ActionResponse res;
		res.actionType = ACTION_TYPE_RESPONSE;
		res.actionResult = result;
		res.modeType = _mode_type;
		res.settingCount = count;

		_txpacket.header1 = YPRC_STX1;
		_txpacket.header2 = YPRC_STX2;
		_txpacket.length = PACKET_LENGTH_ACTIONRESPONSE + 2;
		_txpacket.type = YPRC_PACKET_TYPE_ACTION;
		memcpy(_txpacket.yprc_data, (const uint8_t *)&res, PACKET_LENGTH_ACTIONRESPONSE);
		_txpacket.yprc_data[PACKET_LENGTH_ACTIONRESPONSE] = yprc_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
		DEBUG_PRINTF("SEND_RESPONSE");
		_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
	}
}

void YP_Radio::exit_photo_mode(void)
{
	DEBUG_PRINTF("exit all mode");

	// notify user mode reset
	AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;

	send_response(ACTION_RESULT_OK, 0);
	mission_state = MissionState_Stop;
	_running_action_type = ACTION_TYPE_NONE;

	_mode_type = AUTO_NONES;
	_action_state = ACTION_STATUS_IDLE;
	_setting_count = 0;

}

void YP_Radio::set_photo_mode_clear()
{
	AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;

	_map_waypoint_set_type = map_waypoint_set_type_none;
	mission_state = MissionState_Stop;
	_running_action_type = ACTION_TYPE_NONE;

	_mode_type = AUTO_NONES;
	_action_state = ACTION_STATUS_IDLE;
	_setting_count =0;
}

void YP_Radio::_handle_start_auto(ccc_start_fn ccc_start, poi_orbit_start_fn poi_orbit_start, journey_start_fn journey_start, circle_me_config_fn circle_me_config)
{	
	uint8_t index = 0;

	// not allow start Task Mode if on the ground
	if (AP_Notify::flying_flags.land_complete) {
		send_response(ACTION_RESULT_ERR_FAIL,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
		return;
	}

	// if gimbal not connected, return immediatiely
	if (!_gimbalcamera.got_gimbal_ymavlink_connected()) {
		send_response(ACTION_RESULT_ERR_NO_GIMBAL,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
		return;
	}

	// if we are in rth flightmode, return immediatiely
	if (_telem_data.flightMode ==  FMODE_RETURN_TO_HOME) {
		send_response(ACTION_RESULT_ERR_FAIL, 0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
		return;		
	}
	
	// do start in different mode
	// if in the level 1 battery warning
	if (_battery.voltage() <= _battery.warning_level1_voltage()) {
		send_response(ACTION_RESULT_ERR_LOWBATTERY,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
		return;		
	}

	if (AP_Notify::startup_flags.no_fly_zone && !AP_Notify::startup_flags.restrict_flight) {
		send_response(ACTION_RESULT_ERR_NOFLYZONE,0);
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
		return;
	}
	
	switch(_mode_type) {
		case AUTO_JOUR:
		{
			// To ensure third handshake is ok, when the response for ok is lost to rc due to some reason
			if (_action_state == ACTION_STATUS_RUNNING) {
				send_response(ACTION_RESULT_OK,0);
				return;
			}

			DEBUG_PRINTF("try to start AUTO_JOUR");
			if (_action_state != ACTION_STATUS_IDLE) {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				return;
			}
			if (journey_start()) {
				// change state to RUNNING
				_action_state = ACTION_STATUS_RUNNING;
				// notify user photo mode is running
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
				send_response(ACTION_RESULT_OK, 0);
			} else {
				//send_response(ACTION_RESULT_ERR_FAIL, 0);
				_mode_type = AUTO_NONES;
				_action_state = ACTION_STATUS_IDLE;
				// notify user we are in idle 
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
			}
			break;
		}
		case AUTO_POI:
		{
			DEBUG_PRINTF("try to start AUTO_POI");

			// To ensure third handshake is ok, when response for ok is lost to rc
			if (_action_state == ACTION_STATUS_RUNNING) {
				send_response(ACTION_RESULT_OK,0);
				return;
			}

			if (_action_state != ACTION_STATUS_SETTING) {
				send_response(ACTION_RESULT_ERR_FAIL,0);
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				return;
			}

			if (poi_orbit_start()) {
				_action_state = ACTION_STATUS_RUNNING;
				// notify user photo mode is running
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;

				DEBUG_PRINTF("roi STATE: start\n");

				send_response(ACTION_RESULT_OK, 0);
			} else {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
			}
			break;
		}
		case AUTO_CURVECABLECAM:
		{
			// To ensure third handshake is ok, when response for ok is lost to rc
			if (_action_state == ACTION_STATUS_RUNNING) {
				send_response(ACTION_RESULT_OK,0);
				return;
			}

			if (_action_state != ACTION_STATUS_SETTING) {
				send_response(ACTION_RESULT_ERR_FAIL,0);
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				return;
			}

			//if index is 0, return
			index = _mission.get_current_cable_cam_point_total();

			DEBUG_PRINTF("try to start AUTO_CURVECABLECAM");

			// at least two points is required
			if (index < 2){
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				return;
			}

			AP_Mission::cableCamPoint cable_cam_point;
			cable_cam_point.location = _cur_loc;
			cable_cam_point.copter_yaw = _ahrs.yaw_sensor;
			cable_cam_point.gimbal_yaw = _ahrs.yaw_sensor;
			cable_cam_point.gimbal_pitch = _gimbalcamera.get_pitch_channel();
			_mission.set_start_cable_cam_point(cable_cam_point);

			if (ccc_start()) {
				_action_state = ACTION_STATUS_RUNNING;
				// notify user photo mode is running
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
				send_response(ACTION_RESULT_OK, index);
			} else {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
				_mode_type = AUTO_NONES;
				_action_state = ACTION_STATUS_IDLE;
			}
			break;				
		}
		case AUTO_ORBIT:
		{	
			// To ensure third handshake is ok, when response for ok is lost to rc
			if (_action_state == ACTION_STATUS_RUNNING) {
				send_response(ACTION_RESULT_OK,0);
				return;
			}
			// this is to insure the sequence of photo mode
			if (_action_state != ACTION_STATUS_IDLE) {
				send_response(ACTION_RESULT_ERR_FAIL,0);
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				return;
			}
			// configure ROI center as RC position
			circle_me_config();
			// set mode ROI
			if (poi_orbit_start()) {
				_action_state = ACTION_STATUS_RUNNING;
				// notify user photo mode is running
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
				send_response(ACTION_RESULT_OK, index);
			} else {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
				_mode_type = AUTO_NONES;
				_action_state = ACTION_STATUS_IDLE;
			}
			break;			
		}
		default:
			break;
	}
}

void YP_Radio::_handle_waypoint_start(uint8_t land_complete,bool motor_armed,set_mode_fn set_mode)
{
	uint8_t index = 0;
	// To ensure third handshake is ok, when response for ok is lost to rc
	if (_action_state == ACTION_STATUS_RUNNING) {
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_START_AUTO, ACTION_RESULT_OK, 0,_map_waypoint_set_type);
		return;
	}

	if (_action_state != ACTION_STATUS_SETTING) {
		_action_state = ACTION_STATUS_IDLE;
		_mode_type    = AUTO_NONES;
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_START_AUTO, ACTION_RESULT_OK, 0,_map_waypoint_set_type);
		return;
	}

	// total points of map_waypoint mission.
	index = _mission.get_current_map_waypoint_total();
	// at least one points.
	if (index < 1 || AP_Notify::diagnose_flags.low_battery_warning >= 1) {
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
		_action_state = ACTION_STATUS_IDLE;
		_mode_type    = AUTO_NONES;
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_START_AUTO, ACTION_RESULT_ERR_FAIL, 0,_map_waypoint_set_type);
		return;
	} else if(index == 1) {
		// total points of map waypoint is one cannot let mission cycle
		if(_mission.get_map_waypoint_endingaction() == AIRLINE_ENDING_CYCLE)
		{
			_mission.set_map_waypoint_endingAction(AIRLINE_ENDING_SUSPEND);
		}
	}

	AP_Mission::MapWaypoint map_waypoint;
	map_waypoint.waypointIndex = _setting_count;
	map_waypoint.lat = _cur_loc.lat;
	map_waypoint.lon = _cur_loc.lng;
	map_waypoint.alt = max(_cur_loc.alt,200);
	map_waypoint.waitTime = 0;
	map_waypoint.do_cmd = NULL;
	map_waypoint.speed = 15;     // take off speed
	map_waypoint.pan_angle = 0;
	map_waypoint.tilt_angle = 0;
	map_waypoint.tilt_mode = Tilt_Manual;
	map_waypoint.doCmdCount = 0;

	_mission.set_start_map_waypoint(map_waypoint);
	//if on the land ,copter will auto takeoff first
	if (land_complete) {
		//motor must armed
		if (motor_armed) {
			_mission.set_map_waypoint_auto_takeoff_enable(true);
		} else {
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
			_action_state = ACTION_STATUS_IDLE;
			_mode_type    = AUTO_NONES;
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_START_AUTO, ACTION_RESULT_ERR_FAIL, 0,_map_waypoint_set_type);
			return ;
		}
	} else {
		_mission.set_map_waypoint_auto_takeoff_enable(false);
	}

	if (set_mode(AUTO)) {
		_action_state = ACTION_STATUS_RUNNING;
		//Notify user photo mode is running
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
		_running_action_type = ACTION_TYPE_MAP_WAYPOINT;
		mission_state = MissionState_Running;
		_mode_type = AUTO_MAP_WAYPOINT;
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_START_AUTO, ACTION_RESULT_OK, 0,_map_waypoint_set_type);
	} else {
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
		_mode_type = AUTO_NONES;
		_action_state = ACTION_STATUS_IDLE;
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_START_AUTO, ACTION_RESULT_ERR_FAIL, 0,_map_waypoint_set_type);
	}
}

void YP_Radio::_handle_exit(uint8_t actiontype, reset_control_switch_fn reset_control_switch, Map_waypoint_set_type set_type)
{
	DEBUG_PRINTF("exit all mode");
	// reset mode according to current switch position
	if (AP_Notify::flying_flags.photo_mode != AP_Notify::Photo_Mode_NONE) {
		reset_control_switch();
	}

	// notify user mode reset
	AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;

	switch (actiontype) {
		case ACTION_TYPE_MAP_WAYPOINT:
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_EXIT, ACTION_RESULT_OK, 0, set_type);
			break;
		case ACTION_TYPE_FENCE:
			send_map_waypoint_response(ACTION_TYPE_FENCE, ACTION_REQUEST_EXIT, ACTION_RESULT_OK, 0, map_waypoint_set_type_yp);
			break;
		case ACTION_TYPE_LANDING:
			send_map_waypoint_response(ACTION_TYPE_LANDING, ACTION_REQUEST_EXIT, ACTION_RESULT_OK, 0, map_waypoint_set_type_yp);
			break;
		default:
			send_response(ACTION_RESULT_OK, 0);
			break;
	}

	_map_waypoint_set_type = map_waypoint_set_type_none;
	mission_state = MissionState_Stop;
	_running_action_type = ACTION_TYPE_NONE;

	_mode_type = AUTO_NONES;
	_action_state = ACTION_STATUS_IDLE;
	_setting_count =0;
}

void YP_Radio::_handle_pause(journey_stop_fn journey_stop)
{
	switch(_mode_type){
		case AUTO_CURVECABLECAM:
		{
			// notify user photo mode stopped
			// AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_STOPPED;
			// stop CCC controller
			_wp_nav.set_CCC_stop(true);
			_action_state = ACTION_STATUS_STOPPED;
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case AUTO_JOUR:
		{
			//AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_STOPPED;
			journey_stop();
			_action_state = ACTION_STATUS_STOPPED;
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case AUTO_POI:
		{
			//AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_STOPPED;
			_action_state = ACTION_STATUS_STOPPED;
			_ac_circle.set_circle_stop(true);
			break;
		}
		case AUTO_ORBIT:
		{	
			//AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_STOPPED;
			_action_state = ACTION_STATUS_STOPPED;
			_ac_circle.set_circle_stop(true);
			break;
		}
		default:
			break;
	}
}

void YP_Radio::_handle_resume(journey_resume_fn journey_resume)
{	
	switch(_mode_type){
		case AUTO_CURVECABLECAM:
		{
			// notify user photo mode is running
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
			// restart CCC controller
			_wp_nav.set_CCC_stop(false);
			_action_state = ACTION_STATUS_RUNNING;
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case AUTO_JOUR:
		{
			// notify user photo mode is running
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
			// resume journey
			journey_resume();
			_action_state = ACTION_STATUS_RUNNING;
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case AUTO_POI:
		{	
			// notify user photo mode is running
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
			_ac_circle.set_circle_stop(false);
			_action_state = ACTION_STATUS_RUNNING;
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case AUTO_ORBIT:
		{
			// notify user photo mode is running
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
			_ac_circle.set_circle_stop(false);
			_action_state = ACTION_STATUS_RUNNING;
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		default:
			break;
	}
}

void YP_Radio::_handle_set(roi_reset_fn roi_reset)
{
	switch(_mode_type){
		case AUTO_CURVECABLECAM:
		{
			// we are entering setting mode, ACTIONSETTING_CCC packet will be the next packet
			// init cable_cam mode
			if (_mission.init_cable_cam() && _setting_count > 0) {
				// notify user entered setting
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_SETTING;
				_last_index = 0;
				send_response(ACTION_RESULT_OK, 0);
				DEBUG_PRINTF("init_cable_cam successfully");
			} else {
				// setting_count should be reset to zero to insure next time it will enter setting process
				_setting_count = 0;
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				_mode_type = AUTO_NONES;
				_action_state = ACTION_STATUS_IDLE;
				DEBUG_PRINTF("ACTION_REQUEST_SET:ACTION_RESULT_ERR_FAIL");
			}
			break;
		}
		//add more setting here if needed (roi or other)
		case AUTO_POI:
		{
			// notify user entered setting
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_SETTING;

			// initiate ROI mode
			roi_reset();
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case AUTO_JOUR:
		{
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_SETTING;
 			send_response(ACTION_RESULT_OK, 0);
 			break;
		}
		default:
		{
			send_response(ACTION_RESULT_ERR_FAIL, 0);
			DEBUG_PRINTF("ACTION_REQUEST_SET:ACTION_RESULT_ERR_FAIL");
			break;
		}
	}
}

void YP_Radio::_handle_waypoint_config(Map_waypoint_set_type set_type)
{
	// we are entering setting mode, MapSetWaypointRequest packet will be the next packet
	AP_Mission::MapWayPointConfig mapWayPointConfig;
	mapWayPointConfig.waypointCount = _setting_count;
	mapWayPointConfig.airline_type = _airline_type;
	mapWayPointConfig.airline_ending_action  = _airline_ending_action;
	mapWayPointConfig.altitude_type = _airline_altitude_type;
	mapWayPointConfig.airline_default_speed = _airline_default_speed;
	mapWayPointConfig.pan_mode = _pan_mode;
	mapWayPointConfig.tilt_mode = _tilt_mode;

	// init the map_waypoint mode.
	if (_mission.init_map_waypoint(&mapWayPointConfig) && _setting_count > 0) {
		// notify user entered setting
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_SETTING;
		_mission.set_yaw_looking_cmd(AP_Mission::LOOK_AT_NEXT_WP);
		_last_index = 0;
		_docmd_last_index = 0;
		_docmd_setting_count = 0;
		_map_waypoint_set_type = set_type;
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_CONFIG, ACTION_RESULT_OK, 0,set_type);
		DEBUG_UARTG_PRINTF1("init map waypoint success\n");
	} else {
		// setting_count should be reset to zero to insure next time it will enter setting process.
		_setting_count = 0;
		_mode_type = AUTO_NONES;
		_action_state = ACTION_STATUS_IDLE;
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_CONFIG, ACTION_RESULT_ERR_FAIL, 0,set_type);
		DEBUG_UARTG_PRINTF1("init map waypoint fail\n");
	}
}

void YP_Radio::_handle_packet_channeldata12(ChannelData12* packet)
{
	uint32_t tnow = hal.scheduler->millis();

	_rssi = packet->rssi;
	_rx_count = packet->lost_count;

	_channel_count = (YP_RC_INPUT_CHANNEL < 12) ? YP_RC_INPUT_CHANNEL : 12;

	unsigned stride_count = (_channel_count * 3) / 2;
	unsigned chan_index = 0;
	for (unsigned i = 0; i < stride_count; i += 3) {
		_radio_channels[chan_index] = ((uint16_t)packet->channel[i] << 4);
		_radio_channels[chan_index] |= ((uint16_t)(0xF0 & packet->channel[i + 1]) >> 4);

		//add to save the 12bits channel's raw value.
		_radio_channels_raw[chan_index] = _radio_channels[chan_index];

		// convert values to 1000-2000 ppm encoding in a not too sloppy fashion 
		_radio_channels[chan_index] = (uint16_t)(_radio_channels[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
		chan_index++;

		_radio_channels[chan_index] = ((uint16_t)packet->channel[i + 2]);
		_radio_channels[chan_index] |= (((uint16_t)(0x0F & packet->channel[i + 1])) << 8);

		//add to save the 12bits channel's raw value.
		_radio_channels_raw[chan_index] = _radio_channels[chan_index];
		// convert values to 1000-2000 ppm encoding in a not too sloppy fashion 
		_radio_channels[chan_index] = (uint16_t)(_radio_channels[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
		chan_index++;
	}

	// check if we have lost some packet
	// if yes, do not update measure time for channel and GPS
	if (_rx_count > 0)
		return;

	_last_radio_channel_update_time = tnow;

}

void YP_Radio::_handle_packet_channeldata12_GPS(ChannelData12_Gps* packet)
{
	uint32_t tnow = hal.scheduler->millis();

	_rssi = packet->rssi;
	_rx_count = packet->lost_count;

	/* this can lead to rounding of the strides */
	_channel_count = (YP_RC_INPUT_CHANNEL < 12) ? YP_RC_INPUT_CHANNEL : 12;

	unsigned stride_count = (_channel_count * 3) / 2;
	unsigned chan_index = 0;

	for (unsigned i = 0; i < stride_count; i += 3) {
		_radio_channels[chan_index] = ((uint16_t)packet->channel[i] << 4);
		_radio_channels[chan_index] |= ((uint16_t)(0xF0 & packet->channel[i + 1]) >> 4);
		//add to save the 12bits channel's raw value.
		_radio_channels_raw[chan_index] = _radio_channels[chan_index];
		/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
		_radio_channels[chan_index] = (uint16_t)(_radio_channels[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
		chan_index++;

		_radio_channels[chan_index] = ((uint16_t)packet->channel[i + 2]);
		_radio_channels[chan_index] |= (((uint16_t)(0x0F & packet->channel[i + 1])) << 8);
		//add to save the 12bits channel's raw value.
		_radio_channels_raw[chan_index] = _radio_channels[chan_index];
		/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
		_radio_channels[chan_index] = (uint16_t)(_radio_channels[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
		chan_index++;
	}

	/* copy gps data from rc */
	memcpy((char *)&_radio_gps_data, (const char *)(&(packet->gps_data)), sizeof(StGpsDataType));

	//check if we have lost some packet
	//if yes, do not update measure time for channel and gps
	if (_rx_count > 0) {
		return;
	}

	_last_radio_channel_update_time = tnow;

	if (_radio_last_gps_data.lat != _radio_gps_data.lat || _radio_last_gps_data.lon != _radio_gps_data.lon || _radio_last_gps_data.alt != _radio_gps_data.alt || _radio_last_gps_data.course != _radio_gps_data.course || _radio_last_gps_data.speed != _radio_gps_data.speed) {
		// tell the simple kf filter, we get new rc gps data to use
		_new_gps_data = true;
		_last_gps_update_time = tnow;
		memcpy((char *)&_radio_last_gps_data, (const char *)(&_radio_gps_data), sizeof(StGpsDataType));
	}
}


void YP_Radio::_handle_packet_channeldata24(ChannelData24* packet)
{
	uint32_t tnow = hal.scheduler->millis();

	_rssi = packet->rssi;
	_rx_count = packet->lost_count;

	_channel_count = (YP_RC_INPUT_CHANNEL < 24) ? YP_RC_INPUT_CHANNEL : 24;

	unsigned stride_count = (_channel_count * 3) / 2;
	unsigned chan_index = 0;
	for (unsigned i = 0; i < stride_count; i += 3) {
		_radio_channels[chan_index] = ((uint16_t)packet->channel[i] << 4);
		_radio_channels[chan_index] |= ((uint16_t)(0xF0 & packet->channel[i + 1]) >> 4);
		//add to save the 12bits channel's raw value.
		_radio_channels_raw[chan_index] = _radio_channels[chan_index];
		// convert values to 1000-2000 ppm encoding in a not too sloppy fashion
		_radio_channels[chan_index] = (uint16_t)(_radio_channels[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
		chan_index++;

		_radio_channels[chan_index] = ((uint16_t)packet->channel[i + 2]);
		_radio_channels[chan_index] |= (((uint16_t)(0x0F & packet->channel[i + 1])) << 8);
		//add to save the 12bits channel's raw value.
		_radio_channels_raw[chan_index] = _radio_channels[chan_index];
		// convert values to 1000-2000 ppm encoding in a not too sloppy fashion
		_radio_channels[chan_index] = (uint16_t)(_radio_channels[chan_index] * YPRC_SCALE_FACTOR + .5f) + YPRC_SCALE_OFFSET;
		chan_index++;
	}

	//check if we have lost some packet
	//if yes, do not update measure time for channel and gps
	if (_rx_count > 0)
		return;

	_last_radio_channel_update_time = tnow;

}

/////////////code add for the remote controls waypoint misssion////////////////
/////
void YP_Radio::_handle_MapSetWaypointRequest(MapWaypointSetRequest *packet_in,Map_waypoint_set_type set_type)
{
	uint8_t index = 0;
	uint8_t total_index = 0;
	//if data chain changed ,return immediate
	if(_map_waypoint_set_type != set_type) {
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
		return ;
	}


	// check if the type state, if wrong then return
	if (_mode_type != AUTO_MAP_WAYPOINT || _action_state != ACTION_STATUS_SETTING || packet_in->action_request != ACTION_REQUEST_SET) {
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
		return;
	}

	switch(packet_in->set_type) {
		case MapSetWaypointType_Waypoint:
			index = packet_in->waypointSetting.waypoint.waypointIndex;

			if (index > _setting_count)
			{
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
				return;
			}

			total_index = _mission.get_current_map_waypoint_total();

			// when total index ,if it is 2, means it got 2 points, and if now we get from rc index is 2, so we enter in adding point process.
			if (total_index == index) {
				AP_Mission::MapWaypoint map_waypoint;

				map_waypoint.waypointIndex = packet_in->waypointSetting.waypoint.waypointIndex;
				map_waypoint.lat = packet_in->waypointSetting.waypoint.lat;
				map_waypoint.lon = packet_in->waypointSetting.waypoint.lon;
				map_waypoint.alt = packet_in->waypointSetting.waypoint.alt;
				map_waypoint.waitTime = 0;
				map_waypoint.doCmdCount = packet_in->waypointSetting.waypoint.doCmdCount;
				map_waypoint.do_cmd = NULL;
				map_waypoint.speed = packet_in->waypointSetting.waypoint.speed;
				map_waypoint.pan_angle = packet_in->waypointSetting.waypoint.pan_angle;
				map_waypoint.tilt_angle = packet_in->waypointSetting.waypoint.tilt_angle;
				map_waypoint.tilt_mode = packet_in->waypointSetting.waypoint.tilt_mode;
				_docmd_setting_count = packet_in->waypointSetting.waypoint.doCmdCount;
				DEBUG_UARTG_PRINTF2("way_point Speed:%d\n",map_waypoint.speed);
				if (map_waypoint.alt <= 0) {
					_action_state = ACTION_STATUS_IDLE;
					_mode_type = AUTO_NONES;
					send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
					return;
				}

				// add current waypoint into the map waypoint mission.
				if(_mission.add_map_waypoint_point(map_waypoint)) {
					// notify gcs that add one point successfully
					AP_Notify::flying_flags.map_wapoint_add_point = true;
					_last_index = index;
					send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_OK, index,set_type);
				} else {
					_action_state = ACTION_STATUS_IDLE;
					_mode_type = AUTO_NONES;
					send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
				}
			// To ensure second handshake is ok , when we lost one response for ok
			} else if (_last_index == index) {
				send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_OK, index,set_type);
			} else {
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
			}
			break;
		case MapSetWaypointType_Docmd:
			index = packet_in->waypointSetting.waypointDoCmd.doCmdIndex;

			if (index > _docmd_setting_count)
			{
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
				return;
			}

			total_index = _mission.get_current_map_waypoint_docmd_total();

			if(	index == total_index )
			{
				AP_Mission::MapWaypoint_doCmd Waypoint_docmd;
				// add current waypoint into the map waypoint mission.
				Waypoint_docmd.waypointIndex = packet_in->waypointSetting.waypointDoCmd.waypointIndex;
				Waypoint_docmd.doCmdIndex = packet_in->waypointSetting.waypointDoCmd.doCmdIndex;
				Waypoint_docmd.doCmdType = packet_in->waypointSetting.waypointDoCmd.doCmdType;
				Waypoint_docmd.delayTime = packet_in->waypointSetting.waypointDoCmd.delayTime;
				Waypoint_docmd.Content = packet_in->waypointSetting.waypointDoCmd.doCmdParam;

				if(_mission.add_map_waypoint_do_cmd(Waypoint_docmd))
				{
					// notify gcs that add one point successfully
					AP_Notify::flying_flags.map_wapoint_add_point = true;
					_docmd_last_index = index;
					send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_OK, index,set_type);
				} else {
					_action_state = ACTION_STATUS_IDLE;
					_mode_type = AUTO_NONES;
					send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
				}
			// To ensure second handshake is ok , when we lost one response for ok
			} else if (_docmd_last_index == index) {
				send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_OK, index,set_type);
			} else {
				_action_state = ACTION_STATUS_IDLE;
				_mode_type = AUTO_NONES;
				send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
			}
			break;
		default:
			_action_state = ACTION_STATUS_IDLE;
			_mode_type = AUTO_NONES;
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
			return;
		break;
	}

}



void YP_Radio::_handle_FenceSetWaypointRequest(FenceSetWaypointRequest *packet_in)
{

	uint8_t index = 0;
	uint8_t total_index = 0;

	// check if the type state, if wrong then return
	if (_mode_type != MAP_FENCE|| _action_state != ACTION_STATUS_SETTING || packet_in->action_request != ACTION_REQUEST_SET)
	{
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0);
		return;
	}

	index = packet_in->fence_waypoint.waypointIndex;
	if (index > _setting_count)
	{
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0);
		return;
	}

	total_index = _fence.get_current_fence_point_totol();
	if (total_index == index)
	{
		AC_Fence::Y_FencePoints fencePoint;
		fencePoint.lat = packet_in->fence_waypoint.lat;
		fencePoint.lon = packet_in->fence_waypoint.lon;
		//fencePoint.alt = packet_in->fence_waypoint.alt;
		fencePoint.pointIndex = packet_in->fence_waypoint.waypointIndex;
		//fencePoint.altitudeType = packet_in->fence_waypoint.altitudeType;
		#if 0
		if (fencePoint.alt <= 0)
		{
			_action_state = ACTION_STATUS_IDLE;
			_mode_type = AUTO_NONES;
			send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0);
			return;
		}
		#endif

		// add current waypoint into the map waypoint mission.
		if(_fence.add_fence_point(fencePoint))
		{
			// notify gcs that add one point successfully
			AP_Notify::flying_flags.map_fence_add_point= true;
			_last_index = index;
			send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_OK, index);
		} else {
			_action_state = ACTION_STATUS_IDLE;
			_mode_type = AUTO_NONES;
			send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0);
		}
	}
	else if (_last_index == index) {
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_OK, index);
	} else {
		_action_state = ACTION_STATUS_IDLE;
		_mode_type = AUTO_NONES;
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0);
	}

}

void YP_Radio::send_map_waypoint_response(ActionType_e type, ActionRequest_e request, ActionResult_e result, uint8_t index, Map_waypoint_set_type set_type)
{
	if (set_type == map_waypoint_set_type_wifi) {
//		Wifi_MapWayPointResponse(request,result,index);
		return;
	} else {
		MapWaypointSetResponse tx;
		tx.response.action_response_type = ActionResponseType_RESPONSE;
		tx.response.action_type = type;
		tx.response.action_request = request;
		tx.response.action_result = result;
		tx.waypointIndex = index;

		_txpacket.header1 = YPRC_STX1;
		_txpacket.header2 = YPRC_STX2;
		_txpacket.length = PACKET_LENGTH_MAPSETWAYPOINTRESPONSE + 2;
		memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_MAPSETWAYPOINTRESPONSE);
		_txpacket.type = YPRC_PACKET_TYPE_ACTION;
		_txpacket.yprc_data[PACKET_LENGTH_MAPSETWAYPOINTRESPONSE] = yprc_crc8((uint8_t *)&(_txpacket.length), _txpacket.length);

		_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);

		//for(int i = 0;i<_txpacket.length + 3;i++)
		//{
		//	hal.console->printf("%3X",((const uint8_t *)&_txpacket)[i]);
		//}
		//hal.console->printf("\n");
	}
}


void YP_Radio::send_map_waypoint_camera(CameraActionRequest request, uint8_t waypoint_index, uint8_t docmd_index, uint8_t shooting_time)
{
	MapWaypointCameraRequest tx;

	tx.action_type = ACTION_TYPE_MAP_WAYPOINT;
	tx.waypointIndex = waypoint_index;
	tx.docmdIndex = docmd_index;
	tx.actionRequest = request;
	tx.shooting_time = shooting_time;
	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_ACTIONCAMERAREQUEST + 2;
	memcpy(_txpacket.yprc_data, (const uint8_t *) &tx, PACKET_LENGTH_ACTIONCAMERAREQUEST);
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	_txpacket.yprc_data[PACKET_LENGTH_ACTIONCAMERAREQUEST] = yprc_crc8((uint8_t *) &(_txpacket.length), _txpacket.length);

	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
	_port->write((const uint8_t *) &_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_takeoff_landing_response(ActionType_e type, ActionRequest_e request, ActionResult_e result)
{
	Response	tx;
	tx.action_response_type = ActionResponseType_RESPONSE;
	tx.action_type = type;
	tx.action_request = request;
	tx.action_result = result;
	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_RESPONSE + 2;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_RESPONSE);
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	_txpacket.yprc_data[PACKET_LENGTH_RESPONSE] = yprc_crc8((uint8_t *)&(_txpacket.length), _txpacket.length);

	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_map_fence_response(ActionType_e type, ActionRequest_e request, ActionResult_e result, uint8_t index)
{
	FenceSetWaypointResponse tx;
	tx.response.action_response_type = ActionResponseType_RESPONSE;
	tx.response.action_type = type;
	tx.response.action_request = request;
	tx.response.action_result = result;
	tx.waypointIndex = index;

	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_FENCESETWAYPOINTRESPONSE+ 2;
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	memcpy(_txpacket.yprc_data, (const uint8_t *)&tx, PACKET_LENGTH_FENCESETWAYPOINTRESPONSE);
	_txpacket.yprc_data[PACKET_LENGTH_FENCESETWAYPOINTRESPONSE] = yprc_crc8((uint8_t *)&(_txpacket.length), _txpacket.length);

	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void YP_Radio::send_action_current_mapwapoint(void)
{
#if 0
	MapWaypoint tx;
	tx.waypointIndex = 0;
	tx.lat = _cur_loc.lat;
	tx.lon = _cur_loc.lng;
	tx.alt = _cur_loc.alt;
	tx.gimbal_pitch = 0;
	tx.gimbal_yaw = 0;
	tx.speed = uint8_t(get_speed()*10);
	tx.waitTime = 0;
	tx.waypointAction = WAYPOINT_ACTION_NONE;
	// uint8_t arr_temp[8] = {0};
	// memcpy(tx.waypointActionParam, arr_temp, sizeof(arr_temp));
	for (uint8_t i = 0; i < 8; ++i)
		tx.waypointActionParam[i] = 0;
	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PACKET_LENGTH_MAPWAYPOINT + 2;
	memcpy(_txpacket.yprc_data, (const uint8_t *) &tx, PACKET_LENGTH_MAPWAYPOINT);
	_txpacket.yprc_data[PACKET_LENGTH_MAPWAYPOINT] = yprc_crc8((uint8_t *)&(_txpacket.length), _txpacket.length);
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
#endif
}

void YP_Radio::send_action_mission_feedback(ActionType_e type, uint8_t droneState,uint8_t index, MissionState missionState)
{
	MissionFeedback tx;
	tx.action_response_type = ActionResponseType_FEEDBACK;
	tx.drone_state = droneState;
	tx.action_type = type;
	tx.mission_state = missionState;
	tx.value = index;
	tx.yaw_angle = (uint16_t)copter.gimCam.get_ef_yaw_angle();
	tx.pitch_angle = (uint8_t)copter.gimCam.get_pitch_angle();
	_txpacket.type = YPRC_PACKET_TYPE_ACTION;
	_txpacket.header1 = YPRC_STX1;
	_txpacket.header2 = YPRC_STX2;
	_txpacket.length = PCKKET_LENGTH_MISSIONFEEDBACK + 2;
	memcpy(_txpacket.yprc_data, (const uint8_t *) &tx, PCKKET_LENGTH_MISSIONFEEDBACK);
	_txpacket.yprc_data[PCKKET_LENGTH_MISSIONFEEDBACK] = yprc_crc8((uint8_t *)&(_txpacket.length), _txpacket.length);
	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);

}


/////
/////////////code add for the remote controls waypoint misssion////////////////

void YP_Radio::vehicle_kalman_filter_pos_vel(const struct Location& vehicle_location, const Vector2f& vehicle_speed, bool new_gps_data, bool vehicle_position_ok)
{
	if (!vehicle_position_ok) {
		_vehicle_kf_initialised = false;
	}

	// update T
	uint32_t tnow = hal.scheduler->millis();
	float updateT = constrain_float(((float)(tnow - _vehicle_last_kalman_update_time))*0.001f, 0.001f, 2.0f);
	_vehicle_last_kalman_update_time = tnow;

	// we haven't updated kalman filter for a long time
	// reset kalman filter state
	if (!_vehicle_kf_initialised || (updateT >= 2.0f)) {
		if (!new_gps_data) return;
		_vehicle_origin_loc = vehicle_location;
		_vehicle_kalman_filter_init(vehicle_speed.x, vehicle_speed.y);
		_vehicle_kf_initialised = true;
	}

	// initialize Q
	float sigma_A = sq(_process_noise);
	float T_p2 = sq(updateT);
	float T_p3 = updateT * T_p2;
	float T_p4 = updateT * T_p3;
	_vehicle_latQ[0][0] = sigma_A*T_p4*0.25f; 	_vehicle_latQ[0][1] = sigma_A*T_p3*0.5f;
	_vehicle_latQ[1][0] = _vehicle_latQ[0][1];  _vehicle_latQ[1][1] = sigma_A*T_p2;
	_vehicle_lngQ[0][0] = _vehicle_latQ[0][0];  _vehicle_lngQ[0][1] = _vehicle_latQ[0][1];
	_vehicle_lngQ[1][0] = _vehicle_latQ[1][0];  _vehicle_lngQ[1][1] = _vehicle_latQ[1][1];

	/*
	 * predict step
	 */
	// 1. predict state: Xk|k-1 = F * Xk-1|k-1
	_vehicle_latX[0] += _vehicle_latX[1] * updateT; // _vehicle_latX[1] = _vehicle_latX[1];
	_vehicle_lngX[0] += _vehicle_lngX[1] * updateT; // _vehicle_lngX[1] = _vehicle_lngX[1];

	// 2. update convariance: Pk|k-1 = F * Pk-1|k-1 + Q
	_vehicle_latnextP[0][0] = _vehicle_latP[0][0] + (_vehicle_latP[0][1]+_vehicle_latP[1][0])*updateT + _vehicle_latP[1][1]*T_p2; _vehicle_latnextP[0][1] = _vehicle_latP[0][1] + _vehicle_latP[1][1]*updateT;
	_vehicle_latnextP[1][0] = _vehicle_latP[1][0] + _vehicle_latP[1][1]*updateT; _vehicle_latnextP[1][1] = _vehicle_latP[1][1];
	_vehicle_lngnextP[0][0] = _vehicle_lngP[0][0] + (_vehicle_lngP[0][1]+_vehicle_lngP[1][0])*updateT + _vehicle_lngP[1][1]*T_p2; _vehicle_lngnextP[0][1] = _vehicle_lngP[0][1] + _vehicle_lngP[1][1]*updateT;
	_vehicle_lngnextP[1][0] = _vehicle_lngP[1][0] + _vehicle_lngP[1][1]*updateT; _vehicle_lngnextP[1][1] = _vehicle_lngP[1][1];

	// add process noise
	for(uint8_t i = 0; i < 2; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			_vehicle_latnextP[i][j] += _vehicle_latQ[i][j];
			_vehicle_lngnextP[i][j] += _vehicle_lngQ[i][j];
		}
	}
	// copy predicted variances
	for (uint8_t i = 0; i < 2; i++) {
		_vehicle_latP[i][i] = _vehicle_latnextP[i][i];
		_vehicle_lngP[i][i] = _vehicle_lngnextP[i][i];
	}
	// copy predicted covariances and force symmetry
	_vehicle_latP[1][0] = 0.5f*(_vehicle_latnextP[1][0] + _vehicle_latnextP[0][1]); _vehicle_latP[0][1] = _vehicle_latP[1][0];
	_vehicle_lngP[1][0] = 0.5f*(_vehicle_lngnextP[1][0] + _vehicle_lngnextP[0][1]); _vehicle_lngP[0][1] = _vehicle_lngP[1][0];
	// constrain variances
	_vehicle_latP[0][0] = constrain_float(_vehicle_latP[0][0], 0.0f, 1.0e6f);
	_vehicle_latP[1][1] = constrain_float(_vehicle_latP[1][1], 0.0f, 1.0e3f);
	_vehicle_lngP[0][0] = constrain_float(_vehicle_lngP[0][0], 0.0f, 1.0e6f);
	_vehicle_lngP[1][1] = constrain_float(_vehicle_lngP[1][1], 0.0f, 1.0e3f);

	// get measurement
	if (new_gps_data) {
		// get lat/lng changes in meters
		Vector2f delta_latlng = location_diff(_vehicle_origin_loc, vehicle_location);

		/*
		 * update step
		 */
		// 3. update measurement residual: Yk = Zk - H * Xk|k-1
		//    							   H = I(4x4)
		_vehicle_latY[0] = delta_latlng.x - _vehicle_latX[0];
		_vehicle_latY[1] = vehicle_speed.x - _vehicle_latX[1];
		_vehicle_lngY[0] = delta_latlng.y - _vehicle_lngX[0];
		_vehicle_lngY[1] = vehicle_speed.y - _vehicle_lngX[1];

		// 4. update optimal kalman gain: Sk = H * Pk|k-1 * H' + R
		//								  Kk = Pk|k-1 * H' * inv(Sk)
		// copy predicted covariances and force symmetry
		for (uint8_t i = 0; i < 2; i++) {
			for (uint8_t j = 0; j < 2; j++)	{
				_vehicle_latS[i][j] = _vehicle_latP[i][j];
				_vehicle_lngS[i][j] = _vehicle_lngP[i][j];
			}
		}

		float R_pos = sq(_pos_measure_noise);
		float R_vel = sq(_vel_measure_noise);
		_vehicle_latS[0][0] += R_pos; _vehicle_latS[1][1] += R_vel;
		_vehicle_lngS[0][0] += R_pos; _vehicle_lngS[1][1] += R_vel;

		float latdet = _vehicle_latS[0][0]*_vehicle_latS[1][1] - _vehicle_latS[0][1]*_vehicle_latS[1][0];
		float lngdet = _vehicle_lngS[0][0]*_vehicle_lngS[1][1] - _vehicle_lngS[0][1]*_vehicle_lngS[1][0];
		if ((latdet != 0.0f) && (lngdet != 0.0f)) {
			float inv_latdet = 1.0f / latdet;
			_vehicle_latinvS[0][0] = _vehicle_latS[1][1]*inv_latdet;	_vehicle_latinvS[0][1] = -_vehicle_latS[0][1]*inv_latdet; _vehicle_latinvS[1][0] = -_vehicle_latS[1][0]*inv_latdet; _vehicle_latinvS[1][1] = _vehicle_latS[0][0]*inv_latdet;
			float inv_lngdet = 1.0f / lngdet;
			_vehicle_lnginvS[0][0] = _vehicle_lngS[1][1]*inv_lngdet;	_vehicle_lnginvS[0][1] = -_vehicle_lngS[0][1]*inv_lngdet; _vehicle_lnginvS[1][0] = -_vehicle_lngS[1][0]*inv_lngdet; _vehicle_lnginvS[1][1] = _vehicle_lngS[0][0]*inv_lngdet;

			_vehicle_latK[0][0] = _vehicle_latP[0][0]*_vehicle_latinvS[0][0] + _vehicle_latP[0][1]*_vehicle_latinvS[1][0];	_vehicle_latK[0][1] = _vehicle_latP[0][0]*_vehicle_latinvS[0][1] + _vehicle_latP[0][1]*_vehicle_latinvS[1][1];
			_vehicle_latK[1][0] = _vehicle_latP[1][0]*_vehicle_latinvS[0][0] + _vehicle_latP[1][1]*_vehicle_latinvS[1][0];	_vehicle_latK[1][1] = _vehicle_latP[1][0]*_vehicle_latinvS[0][1] + _vehicle_latP[1][1]*_vehicle_latinvS[1][1];
			_vehicle_lngK[0][0] = _vehicle_lngP[0][0]*_vehicle_lnginvS[0][0] + _vehicle_lngP[0][1]*_vehicle_lnginvS[1][0];	_vehicle_lngK[0][1] = _vehicle_lngP[0][0]*_vehicle_lnginvS[0][1] + _vehicle_lngP[0][1]*_vehicle_lnginvS[1][1];
			_vehicle_lngK[1][0] = _vehicle_lngP[1][0]*_vehicle_lnginvS[0][0] + _vehicle_lngP[1][1]*_vehicle_lnginvS[1][0];	_vehicle_lngK[1][1] = _vehicle_lngP[1][0]*_vehicle_lnginvS[0][1] + _vehicle_lngP[1][1]*_vehicle_lnginvS[1][1];

			// 5. update estimate state: Xk|k = Xk|k-1 + Kk * Yk
			//							 Pk|k = (I - Kk * H) * Pk|k-1
			for (uint8_t i = 0; i < 2; i++) {
				for (uint8_t j = 0; j < 2; j++) {
					_vehicle_latX[i] += _vehicle_latK[i][j] * _vehicle_latY[j];
					_vehicle_lngX[i] += _vehicle_lngK[i][j] * _vehicle_lngY[j];
				}
			}

			_vehicle_latKHP[0][0] = _vehicle_latK[0][0]*_vehicle_latP[0][0] + _vehicle_latK[0][1]*_vehicle_latP[1][0];	_vehicle_latKHP[0][1] = _vehicle_latK[0][0]*_vehicle_latP[0][1] + _vehicle_latK[0][1]*_vehicle_latP[1][1];
			_vehicle_latKHP[1][0] = _vehicle_latK[1][0]*_vehicle_latP[0][0] + _vehicle_latK[1][1]*_vehicle_latP[1][0];	_vehicle_latKHP[1][1] = _vehicle_latK[1][0]*_vehicle_latP[0][1] + _vehicle_latK[1][1]*_vehicle_latP[1][1];
			_vehicle_lngKHP[0][0] = _vehicle_lngK[0][0]*_vehicle_lngP[0][0] + _vehicle_lngK[0][1]*_vehicle_lngP[1][0];	_vehicle_lngKHP[0][1] = _vehicle_lngK[0][0]*_vehicle_lngP[0][1] + _vehicle_lngK[0][1]*_vehicle_lngP[1][1];
			_vehicle_lngKHP[1][0] = _vehicle_lngK[1][0]*_vehicle_lngP[0][0] + _vehicle_lngK[1][1]*_vehicle_lngP[1][0];	_vehicle_lngKHP[1][1] = _vehicle_lngK[1][0]*_vehicle_lngP[0][1] + _vehicle_lngK[1][1]*_vehicle_lngP[1][1];

			for (uint8_t i = 0; i < 2; i++) {
				for (uint8_t j = 0; j < 2; j++) {
					_vehicle_latP[i][j] -= _vehicle_latKHP[i][j];
					_vehicle_lngP[i][j] -= _vehicle_lngKHP[i][j];
				}
			}
		}
	}

    // position limit 1000 km
    _vehicle_latX[0] = constrain_float(_vehicle_latX[0],-1.0e6f,1.0e6f);
    _vehicle_lngX[0] = constrain_float(_vehicle_lngX[0],-1.0e6f,1.0e6f);
    // velocity limit 100 m/sec
    _vehicle_latX[1] = constrain_float(_vehicle_latX[1],-1.0e2f,1.0e2f);
    _vehicle_lngX[1] = constrain_float(_vehicle_lngX[1],-1.0e2f,1.0e2f);

	// update kalman filter location
    float speed_length = pythagorous2(_vehicle_latX[1], _vehicle_lngX[1]);
    float forward_pos_scalar = _forward_pos_scalar.get() * constrain_float(speed_length/3.0f, 0.0f, 1.0f);
    int32_t dlat = (_vehicle_latX[0] + forward_pos_scalar * _vehicle_latX[1]) * LOCATION_SCALING_FACTOR_INV;
	int32_t dlng = ((_vehicle_lngX[0] + forward_pos_scalar * _vehicle_lngX[1]) * LOCATION_SCALING_FACTOR_INV) / longitude_scale(_vehicle_origin_loc);

	_vehicle_kf_position.lat = _vehicle_origin_loc.lat + dlat;
	_vehicle_kf_position.lng = _vehicle_origin_loc.lng + dlng;
}

void YP_Radio::_vehicle_kalman_filter_init(float vel_N, float vel_E)
{
	// initialize RC state
	_vehicle_latX[0] = 0; _vehicle_latX[1] = vel_N;
	_vehicle_lngX[0] = 0; _vehicle_lngX[1] = vel_E;

	// initialize P
    memset(&_vehicle_latP[0][0], 0.0f, sizeof(_vehicle_latP));
    memset(&_vehicle_lngP[0][0], 0.0f, sizeof(_vehicle_lngP));
    memset(&_vehicle_latnextP[0][0], 0.0f, sizeof(_vehicle_latnextP));
    memset(&_vehicle_lngnextP[0][0], 0.0f, sizeof(_vehicle_lngnextP));
    memset(&_vehicle_latS[0][0], 0.0f, sizeof(_vehicle_latS));
    memset(&_vehicle_lngS[0][0], 0.0f, sizeof(_vehicle_lngS));
    memset(&_vehicle_latinvS[0][0], 0.0f, sizeof(_vehicle_latinvS));
    memset(&_vehicle_lnginvS[0][0], 0.0f, sizeof(_vehicle_lnginvS));
    memset(&_vehicle_latK[0][0], 0.0f, sizeof(_vehicle_latK));
    memset(&_vehicle_lngK[0][0], 0.0f, sizeof(_vehicle_lngK));
    memset(&_vehicle_latKHP[0][0], 0.0f, sizeof(_vehicle_latKHP));
    memset(&_vehicle_lngKHP[0][0], 0.0f, sizeof(_vehicle_lngKHP));
}

void YP_Radio::kalman_filter_pos_vel(void)
{
	if (!position_ok()) _kf_initialised = false;

	// update T
	uint32_t tnow = hal.scheduler->millis();
	float updateT = constrain_float(((float)tnow - _last_kalman_update_time)*0.001f, 0.001f, 2.0f);
	_last_kalman_update_time = tnow;

	// we haven't updated kalman filter for a long time
	// reset kalman filter state
	if (!_kf_initialised || updateT >= 2.0f) {
		if (!_new_gps_data) return;
		_origin_loc.lat = _gps_data.lat;
		_origin_loc.lng = _gps_data.lon;
		_origin_loc.alt = _gps_data.alt;
		float ground_speed = get_speed();
		float ground_course = get_course() * DEG_TO_RAD;
		float vel_N = ground_speed * cosf(ground_course);
		float vel_E = ground_speed * sinf(ground_course);
		_kalman_filter_init(vel_N, vel_E);
		_kf_initialised = true;
	}

	// initialize Q
	float sigma_A = sq(_process_noise);
	float T_p2 = sq(updateT);
	float T_p3 = updateT * T_p2;
	float T_p4 = updateT * T_p3;
	latQ[0][0] = sigma_A*T_p4*0.25f; latQ[0][1] = sigma_A*T_p3*0.5f;
	latQ[1][0] = latQ[0][1];  latQ[1][1] = sigma_A*T_p2;
	lngQ[0][0] = latQ[0][0];  lngQ[0][1] = latQ[0][1];
	lngQ[1][0] = latQ[1][0];  lngQ[1][1] = latQ[1][1];

	/*
	 * predict step
	 */
	// 1. predict state: Xk|k-1 = F * Xk-1|k-1
	latX[0] += latX[1] * updateT; // latX[1] = latX[1];
	lngX[0] += lngX[1] * updateT; // lngX[1] = lngX[1];

	// 2. update convariance: Pk|k-1 = F * Pk-1|k-1 + Q
	latnextP[0][0] = latP[0][0] + (latP[0][1]+latP[1][0])*updateT + latP[1][1]*T_p2; latnextP[0][1] = latP[0][1] + latP[1][1]*updateT;
	latnextP[1][0] = latP[1][0] + latP[1][1]*updateT; latnextP[1][1] = latP[1][1];
	lngnextP[0][0] = lngP[0][0] + (lngP[0][1]+lngP[1][0])*updateT + lngP[1][1]*T_p2; lngnextP[0][1] = lngP[0][1] + lngP[1][1]*updateT;
	lngnextP[1][0] = lngP[1][0] + lngP[1][1]*updateT; lngnextP[1][1] = lngP[1][1];

	// add process noise
	for(uint8_t i = 0; i < 2; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			latnextP[i][j] += latQ[i][j];
			lngnextP[i][j] += lngQ[i][j];
		}
	}
	// copy predicted variances
	for (uint8_t i = 0; i < 2; i++) {
		latP[i][i] = latnextP[i][i];
		lngP[i][i] = lngnextP[i][i];
	}
	// copy predicted covariances and force symmetry
	latP[1][0] = 0.5f*(latnextP[1][0] + latnextP[0][1]); latP[0][1] = latP[1][0];
	lngP[1][0] = 0.5f*(lngnextP[1][0] + lngnextP[0][1]); lngP[0][1] = lngP[1][0];
	// constrain variances
	latP[0][0] = constrain_float(latP[0][0], 0.0f, 1.0e6f);
	latP[1][1] = constrain_float(latP[1][1], 0.0f, 1.0e3f);
	lngP[0][0] = constrain_float(lngP[0][0], 0.0f, 1.0e6f);
	lngP[1][1] = constrain_float(lngP[1][1], 0.0f, 1.0e3f);

	// get measurement
	if (_new_gps_data) {
		_new_gps_data = false;

		// get lat/lng changes in meters
		Vector2f delta_latlng = location_diff(_origin_loc, get_location());
		float ground_speed = get_speed();
		float ground_course = get_course() * DEG_TO_RAD;
		float vel_N = ground_speed * cosf(ground_course);
		float vel_E = ground_speed * sinf(ground_course);

		/*
		 * update step
		 */
		// 3. update measurement residual: Yk = Zk - H * Xk|k-1
		//    							   H = I(4x4)
		latY[0] = delta_latlng.x - latX[0];
		latY[1] = vel_N - latX[1];
		lngY[0] = delta_latlng.y - lngX[0];
		lngY[1] = vel_E - lngX[1];

		// 4. update optimal kalman gain: Sk = H * Pk|k-1 * H' + R
		//								  Kk = Pk|k-1 * H' * inv(Sk)
		// copy predicted covariances and force symmetry
		for (uint8_t i = 0; i < 2; i++) {
			for (uint8_t j = 0; j < 2; j++)	{
				latS[i][j] = latP[i][j];
				lngS[i][j] = lngP[i][j];
			}
		}

		float R_pos = sq(_pos_measure_noise);
		float R_vel = sq(_vel_measure_noise);
		latS[0][0] += R_pos; latS[1][1] += R_vel;
		lngS[0][0] += R_pos; lngS[1][1] += R_vel;

		float latdet = latS[0][0]*latS[1][1] - latS[0][1]*latS[1][0];
		float lngdet = lngS[0][0]*lngS[1][1] - lngS[0][1]*lngS[1][0];
		if ((latdet != 0.0f) && (lngdet != 0.0f)) {
			float inv_latdet = 1.0f / latdet;
			latinvS[0][0] = latS[1][1]*inv_latdet;	latinvS[0][1] = -latS[0][1]*inv_latdet; latinvS[1][0] = -latS[1][0]*inv_latdet; latinvS[1][1] = latS[0][0]*inv_latdet;
			float inv_lngdet = 1.0f / lngdet;
			lnginvS[0][0] = lngS[1][1]*inv_lngdet;	lnginvS[0][1] = -lngS[0][1]*inv_lngdet; lnginvS[1][0] = -lngS[1][0]*inv_lngdet; lnginvS[1][1] = lngS[0][0]*inv_lngdet;

			latK[0][0] = latP[0][0]*latinvS[0][0] + latP[0][1]*latinvS[1][0];	latK[0][1] = latP[0][0]*latinvS[0][1] + latP[0][1]*latinvS[1][1];
			latK[1][0] = latP[1][0]*latinvS[0][0] + latP[1][1]*latinvS[1][0];	latK[1][1] = latP[1][0]*latinvS[0][1] + latP[1][1]*latinvS[1][1];
			lngK[0][0] = lngP[0][0]*lnginvS[0][0] + lngP[0][1]*lnginvS[1][0];	lngK[0][1] = lngP[0][0]*lnginvS[0][1] + lngP[0][1]*lnginvS[1][1];
			lngK[1][0] = lngP[1][0]*lnginvS[0][0] + lngP[1][1]*lnginvS[1][0];	lngK[1][1] = lngP[1][0]*lnginvS[0][1] + lngP[1][1]*lnginvS[1][1];

			// 5. update estimate state: Xk|k = Xk|k-1 + Kk * Yk
			//							 Pk|k = (I - Kk * H) * Pk|k-1
			for (uint8_t i = 0; i < 2; i++) {
				for (uint8_t j = 0; j < 2; j++) {
					latX[i] += latK[i][j] * latY[j];
					lngX[i] += lngK[i][j] * lngY[j];
				}
			}

			latKHP[0][0] = latK[0][0]*latP[0][0] + latK[0][1]*latP[1][0];	latKHP[0][1] = latK[0][0]*latP[0][1] + latK[0][1]*latP[1][1];
			latKHP[1][0] = latK[1][0]*latP[0][0] + latK[1][1]*latP[1][0];	latKHP[1][1] = latK[1][0]*latP[0][1] + latK[1][1]*latP[1][1];
			lngKHP[0][0] = lngK[0][0]*lngP[0][0] + lngK[0][1]*lngP[1][0];	lngKHP[0][1] = lngK[0][0]*lngP[0][1] + lngK[0][1]*lngP[1][1];
			lngKHP[1][0] = lngK[1][0]*lngP[0][0] + lngK[1][1]*lngP[1][0];	lngKHP[1][1] = lngK[1][0]*lngP[0][1] + lngK[1][1]*lngP[1][1];

			for (uint8_t i = 0; i < 2; i++) {
				for (uint8_t j = 0; j < 2; j++) {
					latP[i][j] -= latKHP[i][j];
					lngP[i][j] -= lngKHP[i][j];
				}
			}
		}
	}

    // position limit 1000 km
    latX[0] = constrain_float(latX[0],-1.0e6f,1.0e6f);
    lngX[0] = constrain_float(lngX[0],-1.0e6f,1.0e6f);
    // velocity limit 100 m/sec
    latX[1] = constrain_float(latX[1],-1.0e2f,1.0e2f);
    lngX[1] = constrain_float(lngX[1],-1.0e2f,1.0e2f);

	// update kalman filter location
    float speed_length = pythagorous2(latX[1], lngX[1]);
    float forward_pos_scalar = _forward_pos_scalar.get() * constrain_float(speed_length/3.0f, 0.0f, 1.0f);
    int32_t dlat = (latX[0] + forward_pos_scalar * latX[1]) * LOCATION_SCALING_FACTOR_INV;
	int32_t dlng = ((lngX[0] + forward_pos_scalar * lngX[1]) * LOCATION_SCALING_FACTOR_INV) / longitude_scale(_origin_loc);

	_kf_position.lat = _origin_loc.lat + dlat;
	_kf_position.lng = _origin_loc.lng + dlng;
}

void YP_Radio::_kalman_filter_init(float vel_N, float vel_E) {
	// initialize RC state
	latX[0] = 0; latX[1] = vel_N;
	lngX[0] = 0; lngX[1] = vel_E;

	// initialize P
    memset(&latP[0][0], 0.0f, sizeof(latP));
    memset(&lngP[0][0], 0.0f, sizeof(lngP));
    memset(&latnextP[0][0], 0, sizeof(latnextP));
    memset(&lngnextP[0][0], 0, sizeof(lngnextP));
    memset(&latS[0][0], 0, sizeof(latS));
    memset(&lngS[0][0], 0, sizeof(lngS));
    memset(&latinvS[0][0], 0, sizeof(latinvS));
    memset(&lnginvS[0][0], 0, sizeof(lnginvS));
    memset(&latK[0][0], 0, sizeof(latK));
    memset(&lngK[0][0], 0, sizeof(lngK));
    memset(&latKHP[0][0], 0, sizeof(latKHP));
    memset(&lngKHP[0][0], 0, sizeof(lngKHP));
}

void YP_Radio::set_photo_state_resume()
{
	if (_mode_type != AUTO_NONES) {
		_action_state = ACTION_STATUS_RUNNING;
	}
}
