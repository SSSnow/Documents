// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Mission.h
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

/*
 *   The AP_Mission library:
 *   - responsible for managing a list of commands made up of "nav", "do" and "conditional" commands
 *   - reads and writes the mission commands to storage.
 *   - provides easy acces to current, previous and upcoming waypoints
 *   - calls main program's command execution and verify functions.
 *   - accounts for the DO_JUMP command
 *
 */
#ifndef AP_Mission_h
#define AP_Mission_h

#include <AP_HAL.h>
#include <AP_Vehicle.h>
#include <GCS_MAVLink.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_AHRS.h>
#include <StorageManager.h>
#include <YUNEEC_Mission_WayPoint.h>

// definitions
#define AP_MISSION_EEPROM_VERSION           0x65AE  // version number stored in first four bytes of eeprom.  increment this by one when eeprom format is changed
#define AP_MISSION_EEPROM_COMMAND_SIZE      15      // size in bytes of all mission commands

#if HAL_CPU_CLASS < HAL_CPU_CLASS_75
 # define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 3     // allow up to 3 do-jump commands (due to RAM limitations) on the APM2
#else
 # define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 15    // allow up to 15 do-jump commands all high speed CPUs
#endif

#define AP_MISSION_JUMP_REPEAT_FOREVER      -1      // when do-jump command's repeat count is -1 this means endless repeat

#define AP_MISSION_CMD_GIMBAL_ANGLE_NONE	0XFFFF
#define AP_MISSION_CMD_ID_NONE              0       // mavlink cmd id of zero means invalid or missing command
#define AP_MISSION_CMD_INDEX_NONE           65535   // command index of 65535 means invalid or missing command
#define AP_MISSION_JUMP_TIMES_MAX           32767   // maximum number of times a jump can be executed.  Used when jump tracking fails (i.e. when too many jumps in mission)

#define AP_MISSION_FIRST_REAL_COMMAND       1       // command #0 reserved to hold home position
#define AP_MISSION_CCC_START_COMMAND   		0       // command #0 reserved to hold start position

#define AP_MISSION_RESTART_DEFAULT          0       // resume the mission from the last command run by default

#define AP_MISSION_JOURNEY_DISTANCE         20		// meters

#define AP_MISSION_MAX_REPLAY_NUM			100
#define AP_MISSION_MIN_REPLAY_NUM			10

#define MAP_WAYPOINT_AUTO_TAKEOFF_ALT		500

/// @class    AP_Mission
/// @brief    Object managing Mission
class AP_Mission {

public:
    // jump command structure
    struct PACKED Jump_Command {
        uint16_t target;        // target command id
        int16_t num_times;      // num times to repeat.  -1 = repeat forever
    };

    // condition delay command structure
    struct PACKED Conditional_Delay_Command {
        float seconds;          // period of delay in seconds
    };

    // condition delay command structure
    struct PACKED Conditional_Distance_Command {
        float meters;           // distance from next waypoint in meters
    };

    // condition yaw command structure
    struct PACKED Yaw_Command {
        float angle_deg;        // target angle in degrees (0=north, 90=east)
        float turn_rate_dps;    // turn rate in degrees / second (0=use default)
        int8_t direction;       // -1 = ccw, +1 = cw
        uint8_t relative_angle; // 0 = absolute angle, 1 = relative angle
    };

    // change speed command structure
    struct PACKED Change_Speed_Command {
        uint8_t speed_type;     // 0=airspeed, 1=ground speed
        float target_ms;        // target speed in m/s, -1 means no change
        float throttle_pct;     // throttle as a percentage (i.e. 0 ~ 100), -1 means no change
    };

    // set relay command structure
    struct PACKED Set_Relay_Command {
        uint8_t num;            // relay number from 1 to 4
        uint8_t state;          // on = 3.3V or 5V (depending upon board), off = 0V.  only used for do-set-relay, not for do-repeat-relay
    };

    // repeat relay command structure
    struct PACKED Repeat_Relay_Command {
        uint8_t num;            // relay number from 1 to 4
        int16_t repeat_count;   // number of times to trigger the relay
        float cycle_time;       // cycle time in seconds (the time between peaks or the time the relay is on and off for each cycle?)
    };

    // set servo command structure
    struct PACKED Set_Servo_Command {
        uint8_t channel;        // servo channel
        uint16_t pwm;           // pwm value for servo
    };

    // repeat servo command structure
    struct PACKED Repeat_Servo_Command {
        uint8_t channel;        // servo channel
        uint16_t pwm;           // pwm value for servo
        int16_t repeat_count;   // number of times to move the servo (returns to trim in between)
        float cycle_time;       // cycle time in seconds (the time between peaks or the time the servo is at the specified pwm value for each cycle?)
    };

    // mount control command structure
    struct PACKED Mount_Control {
        float pitch;            // pitch angle in degrees
        float roll;             // roll angle in degrees
        float yaw;              // yaw angle (relative to vehicle heading) in degrees
    };

    // digicam control command structure
    struct PACKED Digicam_Configure {
        uint8_t shooting_mode;  // ProgramAuto = 1, AV = 2, TV = 3, Man=4, IntelligentAuto=5, SuperiorAuto=6
        uint16_t shutter_speed;
        uint8_t aperture;       // F stop number * 10
        uint16_t ISO;           // 80, 100, 200, etc
        uint8_t exposure_type;
        uint8_t cmd_id;
        float engine_cutoff_time;   // seconds
    };

    // digicam control command structure
    struct PACKED Digicam_Control {
        uint8_t session;        // 1 = on, 0 = off
        uint8_t zoom_pos;
        int8_t zoom_step;       // +1 = zoom in, -1 = zoom out
        uint8_t focus_lock;
        uint8_t shooting_cmd;
        uint8_t cmd_id;
    };

    // set cam trigger distance command structure
    struct PACKED Cam_Trigg_Distance {
        float meters;           // distance
    };

    // gripper command structure
    struct PACKED Gripper_Command {
        uint8_t num;            // gripper number
        uint8_t action;         // action (0 = release, 1 = grab)
    };

    // high altitude balloon altitude wait
    struct PACKED Altitude_Wait {
        float altitude; // meters
        float descent_rate; // m/s
        uint8_t wiggle_time; // seconds
    };

    // nav guided command
    struct PACKED Guided_Limits_Command {
        // max time is held in p1 field
        float alt_min;          // min alt below which the command will be aborted.  0 for no lower alt limit
        float alt_max;          // max alt above which the command will be aborted.  0 for no upper alt limit
        float horiz_max;        // max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
    };

    //nav speed command
    struct PACKED DroneSpeed {
    	uint8_t speed;
    };

    //nav gimbal command
    struct PACKED GimbalParam {
    		uint16_t GimbalParam_yaw_angle;			//if GimbalParam_yaw_orientation == Gimbal_Orientation_Programed
    		uint16_t GimbalParam_pitch_angle;			//if GimbalParam_pitch_orientation == Gimbal_Orientation_Programed
    };

    //do camera control
    struct PACKED CameraControl {
    	uint8_t nav_index;
    	uint8_t do_index;
    	uint8_t photo;      // 1: start photo;
    	uint8_t shooting;   //1:start shooting;
    	uint8_t shootTime;  //shooting time (1~60)
    };

    union PACKED Content {
        // jump structure
        Jump_Command jump;

        // conditional delay
        Conditional_Delay_Command delay;

        // conditional distance
        Conditional_Distance_Command distance;

        // conditional yaw
        Yaw_Command yaw;

        // change speed
        Change_Speed_Command speed;

        // do-set-relay
        Set_Relay_Command relay;

        // do-repeat-relay
        Repeat_Relay_Command repeat_relay;

        // do-set-servo
        Set_Servo_Command servo;

        // do-repeate-servo
        Repeat_Servo_Command repeat_servo;

        // mount control
        Mount_Control mount_control;

        // camera configure
        Digicam_Configure digicam_configure;

        // camera control
        Digicam_Control digicam_control;

        // cam trigg distance
        Cam_Trigg_Distance cam_trigg_dist;

        // do-gripper
        Gripper_Command gripper;

        // do-guided-limits
        Guided_Limits_Command guided_limits;

        // cam trigg distance
        Altitude_Wait altitude_wait;

        // location
        Location location;      // Waypoint location

        //drone speed
        DroneSpeed dronespeed;

        //gimbal param
        GimbalParam gimbalParam;

        CameraControl cameracontrol;

        // raw bytes, for reading/writing to eeprom
        uint8_t bytes[12];
    };

    // command structure
    struct PACKED Mission_Command {
        uint16_t index;             // this commands position in the command list
        uint8_t id;                 // mavlink command id
        uint16_t p1;                // general purpose parameter 1
        Content content;
    };

    // main program function pointers
    FUNCTOR_TYPEDEF(mission_cmd_fn_t, bool, const Mission_Command&);
    FUNCTOR_TYPEDEF(mission_complete_fn_t, void);

    // mission state enumeration
    enum mission_state {
        MISSION_STOPPED=0,
        MISSION_RUNNING=1,
        MISSION_COMPLETE=2
    };

    /// constructor
    AP_Mission(AP_AHRS &ahrs, const int32_t &home_bearing, mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn);

    ///
    /// public mission methods
    ///
    /// - Curve-Cable-Cam mission supported
    ///

    /// init - initialises this library including checks the version in eeprom matches this library
    /// 	allocate memory for Curve-Cable-Cam mission
    void init();

    enum mission_type {
    	MISSION_TYPE_MAVLINK,
		MISSION_TYPE_CABLECAM,
		MISSION_TYPE_MAPWAYPOINT,
    };

    // return true if set type of mission successfully
    bool set_mission_type(enum mission_type type);

    // return current mission type
    enum mission_type current_type() const { return _mission_type; }

    /// status - returns the status of the mission (i.e. Mission_Started, Mission_Complete, Mission_Stopped
    mission_state state() const { return _flags.state; }

    /// num_commands - returns total number of commands in the mission
    uint16_t num_commands() const;

    /// start - resets current commands to point to the beginning of the mission
    ///     To-Do: should we validate the mission first and return true/false?
    void start();

    /// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
    void stop();

    /// resume - continues the mission execution from where we last left off
    ///     previous running commands will be re-initialised
    void resume();

    /// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
    void start_or_resume();

    /// reset - reset mission to the first command
    void reset();

    /// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
    ///     should be called at 10hz or higher
    void update();

    /// get the ground course of the next navigation leg in centidegrees
    /// from 0 36000. Return default_angle if next navigation
    /// leg cannot be determined
    int32_t get_next_ground_course_cd(int32_t default_angle);

    /// load_cmd_from_storage - load command from storage
    ///     true is return if successful
    bool read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const;

    bool read_do_cmd_from_storage(uint16_t nav_index,uint16_t do_index, Mission_Command& cmd) const;
    /// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
    static bool is_nav_cmd(const Mission_Command& cmd);

    /// get_current_nav_cmd - returns the current "navigation" command
    const Mission_Command& get_current_nav_cmd() const { return _nav_cmd; }

    /// get_current_nav_index - returns the current "navigation" command index
    /// Note that this will return 0 if there is no command. This is
    /// used in MAVLink reporting of the mission command
    uint16_t get_current_nav_index() const {
        return _nav_cmd.index==AP_MISSION_CMD_INDEX_NONE?0:_nav_cmd.index; }

    uint16_t get_current_cable_cam_point_total() const {
        return _cable_cam_points_total;
    }

	uint16_t get_current_map_waypoint_total() const {
		return  _map_waypoint_total;
	}

	uint8_t get_current_map_waypoint_docmd_total() const{

		return _map_waypoint_do_total;
	}

    /// get_prev_nav_cmd_index - returns the previous "navigation" commands index (i.e. position in the mission command list)
    ///     if there was no previous nav command it returns AP_MISSION_CMD_INDEX_NONE
    ///     we do not return the entire command to save on RAM
    uint16_t get_prev_nav_cmd_index() const { return _prev_nav_cmd_index; }

	uint16_t get_reached_nav_index() const {
		return (_prev_nav_cmd_index==AP_MISSION_CMD_INDEX_NONE)?0:_prev_nav_cmd_index;
	}

    ///
    /// MAVLINK mission type command methods
    ///

    /// num_commands_max - returns maximum number of commands that can be stored
    uint16_t num_commands_max() const;

    /// clear - clears out mission
    ///     returns true if mission was running so it could not be cleared
    ///
    bool clear();

    /// truncate - truncate any mission items beyond given index
    ///		only for MAVLINK mission type
    void truncate(uint16_t index);

    /// add_cmd - adds a command to the end of the command list and writes to storage
    ///     returns true if successfully added, false on failure
    ///     cmd.index is updated with it's new position in the mission
    bool add_cmd(Mission_Command& cmd);

    /// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
    ///     replacing the current active command will have no effect until the command is restarted
    ///     returns true if successfully replaced, false on failure
    bool replace_cmd(uint16_t index, Mission_Command& cmd);

    /// get_next_nav_cmd - gets next "navigation" command found at or after start_index
    ///     returns true if found, false if not found (i.e. reached end of mission command list)
    ///     accounts for do_jump commands
    bool get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd);

    /// get_current_do_cmd - returns active "do" command
    const Mission_Command& get_current_do_cmd() const { return _do_cmd; }

    // set_current_cmd - jumps to command specified by index
    bool set_current_cmd(uint16_t index);

    /// write_cmd_to_storage - write a command to storage
    ///     cmd.index is used to calculate the storage location
    ///     true is returned if successful
    bool write_cmd_to_storage(uint16_t index, Mission_Command& cmd);

    /// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
    ///     home is taken directly from ahrs
    void write_home_to_storage();

    // mavlink_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
    //  return true on success, false on failure
    static bool mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd);

    // mission_cmd_to_mavlink - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
    //  return true on success, false on failure
    static bool mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet);

    // return the last time the mission changed in milliseconds
    uint32_t last_change_time_ms(void) const { return _last_change_time_ms; }

    // find the nearest landing sequence starting point (DO_LAND_START) and
    // return its index.  Returns 0 if no appropriate DO_LAND_START point can
    // be found.
    uint16_t get_landing_sequence_start();

    ///
    /// CURVE-CABLE-CAM mission type command methods
    ///

    typedef struct {
    	struct Location location;
    	float copter_yaw;
    	float gimbal_yaw;
    	float gimbal_pitch;
    } cableCamPoint;

    enum mission_direction {
    	MISSION_DIRECTION_FORWARD,
		MISSION_DIRECTION_BACKWARD,
    };

    // reset cable cam
    bool init_cable_cam();

    // return true if set cable cam direction successfully
    bool set_cable_cam_direction(enum mission_direction direction);

    // get cable cam direction
    enum mission_direction get_cable_cam_direction(void) const { return _cable_cam_direction; }

    // add cable-cam point
    // index will not allow be set by user because it's
    // more safe to manager by itself rather than user
    // return true if set successfully
    bool add_cable_cam_point(cableCamPoint &cable_cam_point);

    // delete number cable-cam point at the top of points array
    // return number of points that has been deleted
    uint16_t delete_cable_cam_points(uint16_t amount);

    /// always set start cable cam point before start cable cam mission
    void set_start_cable_cam_point(cableCamPoint &cable_cam_point);

    /// get_next_CCC_cmd - gets next "navigation" command found after cur_index
    ///     returns true if found, false if not found (i.e. reached end of mission command list)
    bool get_next_CCC_cmd(uint16_t cur_index, Mission_Command& cmd);

    // return true if get current cable cam point successfully
    bool get_current_cable_cam_point(cableCamPoint& cable_cam_point);

    // return true if get next cable cam point successfully
    bool get_prev_cable_cam_point(cableCamPoint& cable_cam_point);

    // return true if one direction finished
    bool one_direction_finished() const { return _one_direction_finished; }

    ///
    /// Map Waypoint mission type command methods
    ///
	typedef  struct {
		uint8_t waypointIndex;
		uint8_t doCmdIndex;
		uint8_t doCmdType;
		uint16_t delayTime;
		DoCmdParam Content;
	} MapWaypoint_doCmd;

	typedef struct {
		uint8_t  waypointIndex;				// index of waypoint
		int32_t  lat;						// lattitude (degrees)	+/- 90 deg
		int32_t  lon;						// longitude (degrees)	+/- 180 deg
		int32_t  alt;						// 0.01m resolution, altitude (meters)
		uint8_t  waitTime;                  // wait Time
		uint8_t  speed;
		int16_t  pan_angle;				//yaw angle
		int16_t  tilt_angle;				//pitch angle
		TiltMode tilt_mode;             //current tilt mode
		uint8_t	 doCmdCount;
		MapWaypoint_doCmd *do_cmd;
	} MapWaypoint;

	typedef struct {
		uint8_t waypointCount;
		AirlineType airline_type;
		AirlineEndingAction airline_ending_action;
		uint8_t altitude_type;		// the altitude type: absolute or relative
		uint8_t airline_default_speed;
		PanMode pan_mode;
		TiltMode tilt_mode;
	} MapWayPointConfig;


	typedef enum {
		LOOK_AT_HOLD = 0,
		LOOK_AT_HEAD,
		LOOK_AT_NEXT_WP,
		LOOK_AT_CURVATURE_CENTER,
		LOOK_AT_CERTAIN_ANGLE,
		LOOK_AT_TURN_RATE_DIPS
	} YawLookAt;

	// init map_waypoint
	bool init_map_waypoint(MapWayPointConfig *mw_cfg);
	bool add_map_waypoint_point(MapWaypoint &map_waypoint);
	bool add_map_waypoint_do_cmd(MapWaypoint_doCmd &Waypoint_docmd);
	bool delect_map_waypoint_point(uint8_t num);
	void set_start_map_waypoint(MapWaypoint &way_point);
	void set_map_waypoint_auto_takeoff_enable(bool autotakeoff_enabled);
	// set copter yaw looking
	void set_yaw_looking_cmd(YawLookAt yaw_looking) { _yaw_looking = yaw_looking; }
	bool get_next_MWP_nav_cmd(uint16_t cur_index, Mission_Command& cmd);
	bool get_current_map_waypoint(MapWaypoint& map_waypoint);
	uint16_t get_current_mwp_cmd_speed();
	bool get_prev_map_waypoint(MapWaypoint& mapWayPoint);
	uint8_t get_map_waypoint_airline_type() const { return _map_waypoint_cfg.airline_type; }
	//uint8_t get_map_waypoint_gimbal_orign() const { return _map_waypoint_cfg.gimbal_orientation; }
	uint8_t get_map_waypoint_endingaction() const { return _map_waypoint_cfg.airline_ending_action; }
	void set_map_waypoint_endingAction(AirlineEndingAction action) { _map_waypoint_cfg.airline_ending_action = action; }
	bool get_map_airline_ending_land_flag() const { return _map_airline_ending_land_enable; }
	YawLookAt get_yaw_looking_cmd() const { return _yaw_looking; }
	PanMode get_pan_config_mod() const { return _map_waypoint_cfg.pan_mode; }
	TiltMode get_tilt_config_mod() const { return _map_waypoint_cfg.tilt_mode; }
	int16_t get_pan_angle() const { return _gimbal_pan; }
	int16_t get_tilt_angle() const {return _gimbal_tilt; }
	int16_t get_pan_angle_pre() const { return _gimbal_pan_pre; }
	int16_t get_tilt_angle_pre() const {return _gimbal_tilt_pre; }
	TiltMode get_cur_tilt_mod() const { return _cur_tilt_mode; }
	/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
	void complete_flag();

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    static StorageAccess _storage;

    struct Mission_Flags {
        mission_state state;
        uint8_t nav_cmd_loaded  : 1; // true if a "navigation" command has been loaded into _nav_cmd
        uint8_t do_cmd_loaded   : 1; // true if a "do"/"conditional" command has been loaded into _do_cmd
        uint8_t do_cmd_all_done : 1; // true if all "do"/"conditional" commands have been completed (stops unnecessary searching through eeprom for do commands)
    } _flags;

    ///
    /// private methods
    ///

    /// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
    void complete();

	//current_wmp_nav_do_cmd -yuneec map waypoint do cmd sync witch nav cmd
	bool advance_current_MWP_do_cmd(void);

	//advance current_wmp_nav_nav_cmd -advanced yuneec map waypoint nav cmd, moves current nav command forward
	bool advance_current_MWP_nav_cmd(void);

    /// advance_current_nav_cmd - moves current nav command forward
    ///     do command will also be loaded
    ///     accounts for do-jump commands
    //      returns true if command is advanced, false if failed (i.e. mission completed)
    bool advance_current_nav_cmd();

    /// advance_current_CCC_cmd - moves current CCC command forward
    //      returns true if command is advanced, false if failed (i.e. mission completed)
    bool advance_current_CCC_cmd();

    /// advance_current_do_cmd - moves current do command forward
    ///     accounts for do-jump commands
    ///     returns true if successfully advanced (can it ever be unsuccessful?)
    void advance_current_do_cmd();

    /// get_next_cmd - gets next command found at or after start_index
    ///     returns true if found, false if not found (i.e. mission complete)
    ///     accounts for do_jump commands
    ///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
    bool get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found);

    bool get_next_cmd(uint16_t nav_index,uint16_t do_index, Mission_Command& cmd);
   // bool get_next_do_cmd(uint16_t start_index, Mission_Command& cmd);
    /// get_next_do_cmd - gets next "do" or "conditional" command after start_index
    ///     returns true if found, false if not found
    ///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
    ///     accounts for do_jump commands but never increments the jump's num_times_run (get_next_nav_cmd is responsible for this)
    bool get_next_do_cmd(uint16_t start_index, Mission_Command& cmd);

    bool get_next_nav_do_cmd(uint16_t nav_index,uint16_t do_index, Mission_Command& cmd);

    ///
    /// jump handling methods
    ///
    // init_jump_tracking - initialise jump_tracking variables
    void init_jump_tracking();

    /// get_jump_times_run - returns number of times the jump command has been run
    ///     return is signed to be consistent with do-jump cmd's repeat count which can be -1 (to signify to repeat forever)
    int16_t get_jump_times_run(const Mission_Command& cmd);

    /// increment_jump_times_run - increments the recorded number of times the jump command has been run
    void increment_jump_times_run(Mission_Command& cmd);

    /// check_eeprom_version - checks version of missions stored in eeprom matches this library
    /// command list will be cleared if they do not match
    void check_eeprom_version();

    // references to external libraries
    const AP_AHRS&   _ahrs;      // used only for home position

    // parameters
    AP_Int16	_cmd_total;  // total number of commands in the mission
    AP_Int8		_restart;   // controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
    AP_Int8		_max_ccc_points; // maximum number of replay state
    AP_Int8		_max_map_waypoints;	// maximum number of mapwaypoint
    AP_Int8		_max_map_waypoints_docmd;	// maximum number of mapwaypoint

    enum mission_type _mission_type;

    // map waypoint variables
	uint16_t				_map_waypoint_total;
	uint16_t    			_map_waypoint_do_total;
	MapWaypoint*			_map_waypoint_point;
	MapWaypoint				_start_map_waypoint_point;
	MapWayPointConfig		_map_waypoint_cfg;
	bool					_map_waypoint_initiated;
	bool					_map_airline_ending_cycle_enable;
	bool					_map_airline_ending_land_enable;
	bool 					_pre_auto_takeoff_enabled;
	int16_t 				_auto_takeoff_alt;
	struct Mission_Command 	_pre_autotakeoff_cmd;
	YawLookAt				_yaw_looking;
	int16_t              	_gimbal_pan;
	int16_t				 	_gimbal_tilt;
	int16_t               	_gimbal_pan_pre;
	int16_t 				_gimbal_tilt_pre;
	TiltMode				_cur_tilt_mode;

    // curve cable cam variables
    uint16_t 				_cable_cam_points_total;
    cableCamPoint*			_cable_cam_point;
    bool					_cable_cam_initiated;
    enum mission_direction	_cable_cam_direction;
    bool					_one_direction_finished;
    cableCamPoint			_start_cable_cam_point;

    // pointer to main program functions
    mission_cmd_fn_t        _cmd_start_fn;  // pointer to function which will be called when a new command is started
    mission_cmd_fn_t        _cmd_verify_fn; // pointer to function which will be called repeatedly to ensure a command is progressing
    mission_complete_fn_t   _mission_complete_fn;   // pointer to function which will be called when mission completes

    // internal variables
    struct Mission_Command  _nav_cmd;   // current "navigation" command.  It's position in the command list is held in _nav_cmd.index
    struct Mission_Command  _do_cmd;    // current "do" command.  It's position in the command list is held in _do_cmd.index
    uint16_t                _prev_nav_cmd_index;    // index of the previous "navigation" command.  Rarely used which is why we don't store the whole command

    // jump related variables
    struct jump_tracking_struct {
        uint16_t index;                 // index of do-jump commands in mission
        int16_t num_times_run;          // number of times this jump command has been run
    } _jump_tracking[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];

    // last time that mission changed
    uint32_t _last_change_time_ms;

    const int32_t& _home_bearing;
};

#endif
