/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_WPNAV_H
#define AC_WPNAV_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl.h> // Attitude control library
#include <AP_Mission.h>
#include <YUNEEC_NoFlyZone.h>

// common setting
#define WPNAV_UPDATE_TIME          						0.020f      // 50hz update rate on high speed CPUs
#define WPNAV_ROLL_PITCH_INPUT_MAX	   					4500.0f		// maximum pilot input

// follow mode default setting
#define WPNAV_FOLLOW_MIN_PILOT_INPUT	 				50.0f		// minimum speed from pilot input
#define WPNAV_FOLLOW_MAX_PILOT_INPUT					600.0f		// maximum speed from pilot input
#define WPNAV_FOLLOW_MIN_PILOT_ACCEL					100.0f		// minimum acceleration of pilot
#define WPNAV_FOLLOW_MAX_PILOT_ACCEL    				500.0f		// maximum acceleration of pilot
#define WPNAV_FOLLOW_MAX_SPEED		  		 			2000.0f		// maximum speed in follow mode
#define WPNAV_FOLLOW_MAX_VEL_FROM_PERR 					1000.0f		// maximum compensate velocity from position error in cm/s
#define WPNAV_CA_FOLLOW_MAX_VEL_FROM_PERR 				500.0f		// maximum compensate velocity from position error in cm/s
#define WPNAV_FOLLOW_MIN_VEL_FROM_PERR	 				80.0f		// minimum compensate velocity from position error in cm/s
#define WPNAV_FOLLOW_MIN_PERR							180.0f		// minimum position error
#define WPNAV_FOLLOW_TERR_CUTOFF_FREQ	  				1.0f		// track error low pass filter cutoff frequency(unit: hz)
#define WPNAV_FOLLOW_TERR_LEN_CUTOFF_FREQ 				0.35f		// track error length low pass filter cutoff frequency(unit: hz)
#define WPNAV_FOLLOW_SPD_LEN_CUTOFF_FREQ  				1.0f		// track error length low pass filter cutoff frequency(unit: hz)
#define WPNAV_FOLLOW_MIN_MOV_SPEED		 				80.0f		// minimum pilot's movement speed
#define	WPNAV_FOLLOW_CMD_ACCEL							500.0f		// maximum acceleration from user input
#define WPNAV_FOLLOW_VEL_COMP_P			  				1.5f		// track PID - P gain
#define WPNAV_FOLLOW_VEL_COMP_I			 				0.00f		// track PID - I gain
#define WPNAV_FOLLOW_VEL_COMP_D		      				0.00f		// track PID - D gain
#define WPNAV_FOLLOW_VEL_COMP_IMAX		  				WPNAV_FOLLOW_MAX_VEL_FROM_PERR	// track PID - Imax
#define WPNAV_FOLLOW_VEL_COMP_FILT_HZ	  				1.0f		// track PID - filter cutoff frequency
#define WPNAV_FOLLOW_ACCEL_XY           				500.0f      // default horizontal acceleration in follow mode
#define WPNAV_FOLLOW_ACCEL_Z							300.0f		// default vertical acceleration in follow mode
#define WPNAV_FOLLOW_SPEED_UP							300.0f		// default maximum climb velocity
#define WPNAV_FOLLOW_SPEED_DOWN							300.0f		// default maximum descent velocity
#define WPNAV_FOLLOW_JERK_MAX							500.0f      // maximum jerk in cm/s/s/s in follow mode

// angle mode default setting
#define WPNAV_ANGLE_SPEED_UP							500.0f		// default maximum climb velocity
#define WPNAV_ANGLE_SPEED_DOWN							300.0f		// default maximum descent velocity
#define WPNAV_ANGLE_ACCEL_Z			   					600.0f		// default maximum vertical acceleration in angle mode
#define WPNAV_ANGLE_STOP_TIME			  				1.0f		// maximum stop time after break velocity is alreay zero
#define WPNAV_ANGLE_SPEED_DEADZONE		  				5.0f		// speed dead zone

// loiter mode default setting
#define WPNAV_LOITER_SPEED            		 			1350.0f     // default loiter speed in cm/s
#define WPNAV_LOITER_SPEED_MIN            				1.0f      	// minimum loiter speed in cm/s
#define WPNAV_LOITER_ACCEL             					500.0f      // default acceleration in loiter mode
#define WPNAV_LOITER_ACCEL_MIN          				100.0f      // minimum acceleration in loiter mode
#define WPNAV_LOITER_JERK_MAX_DEFAULT   				500.0f      // maximum jerk in cm/s/s/s in loiter mode

// waypoint mode default setting
#if defined(YUNEEC_PRODUCT_TYPHOONH)

#define WPNAV_WP_ACCELERATION              				200.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WPNAV_WP_SPEED                 					500.0f      // default horizontal speed betwen waypoints in cm/s
#define WPNAV_WP_SPEED_MIN              				100.0f      // minimum horizontal speed between waypoints in cm/s
#define WPNAV_WP_TRACK_SPEED_MIN        	 			50.0f       // minimum speed along track of the target point the vehicle is chasing in cm/s (used as target slows down before reaching destination)
#define WPNAV_WP_RADIUS                 				200.0f      // default waypoint radius in cm
#define WPNAV_WP_SPEED_UP               				300.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             				300.0f      // default maximum descent velocity
#define WPNAV_WP_ACCEL_Z_DEFAULT        				200.0f      // default vertical acceleration betwen waypoints in cm/s/s
#define WPNAV_WP_LEASH_LENGTH_MIN          				80.0f       // minimum leash lengths in cm
#define WPNAV_WP_FAST_OVERSHOOT_MAX     				200.0f      // 2m overshoot is allowed during fast waypoints to allow for smooth transitions to next waypoint
#define WPNAV_WP_YAW_DIST_MIN                 			200.0f     	// minimum track length which will lead to target yaw being updated to point at next waypoint.  Under this distance the yaw target will be frozen at the current heading
#define WPNAV_ANGLE_SPEED_XY           					1350.0f     // default maximum speed in cm/s in angle mode
#define WPNAV_ANGLE_ACCEL_XY							500.0f		// default maximum acceleration in cm/s/s in angle mode

#elif defined(YUNEEC_PRODUCT_TORNADO)

#define WPNAV_WP_ACCELERATION              				200.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WPNAV_WP_SPEED                 					700.0f      // default horizontal speed betwen waypoints in cm/s
#define WPNAV_WP_SPEED_MIN              				100.0f      // minimum horizontal speed between waypoints in cm/s
#define WPNAV_WP_TRACK_SPEED_MIN        	 			100.0f      // minimum speed along track of the target point the vehicle is chasing in cm/s (used as target slows down before reaching destination)
#define WPNAV_WP_RADIUS                 				200.0f      // default waypoint radius in cm
#define WPNAV_WP_SPEED_UP               				300.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             				300.0f      // default maximum descent velocity
#define WPNAV_WP_ACCEL_Z_DEFAULT        				200.0f      // default vertical acceleration betwen waypoints in cm/s/s
#define WPNAV_WP_LEASH_LENGTH_MIN          				80.0f       // minimum leash lengths in cm
#define WPNAV_WP_FAST_OVERSHOOT_MAX     				200.0f      // 2m overshoot is allowed during fast waypoints to allow for smooth transitions to next waypoint
#define WPNAV_WP_YAW_DIST_MIN                 			200.0f     	// minimum track length which will lead to target yaw being updated to point at next waypoint.  Under this distance the yaw target will be frozen at the current heading
#define WPNAV_ANGLE_SPEED_XY           					1150.0f     // default maximum speed in cm/s in angle mode
#define WPNAV_ANGLE_ACCEL_XY							400.0f		// default maximum acceleration in cm/s/s in angle mode

#elif defined(YUNEEC_PRODUCT_TYPHOONH_PLUS)
#define WPNAV_WP_ACCELERATION              				200.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WPNAV_WP_SPEED                 					500.0f      // default horizontal speed betwen waypoints in cm/s
#define WPNAV_WP_SPEED_MIN              				100.0f      // minimum horizontal speed between waypoints in cm/s
#define WPNAV_WP_TRACK_SPEED_MIN        	 			50.0f       // minimum speed along track of the target point the vehicle is chasing in cm/s (used as target slows down before reaching destination)
#define WPNAV_WP_RADIUS                 				200.0f      // default waypoint radius in cm
#define WPNAV_WP_SPEED_UP               				300.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             				300.0f      // default maximum descent velocity
#define WPNAV_WP_ACCEL_Z_DEFAULT        				200.0f      // default vertical acceleration betwen waypoints in cm/s/s
#define WPNAV_WP_LEASH_LENGTH_MIN          				80.0f       // minimum leash lengths in cm
#define WPNAV_WP_FAST_OVERSHOOT_MAX     				200.0f      // 2m overshoot is allowed during fast waypoints to allow for smooth transitions to next waypoint
#define WPNAV_WP_YAW_DIST_MIN                 			200.0f     	// minimum track length which will lead to target yaw being updated to point at next waypoint.  Under this distance the yaw target will be frozen at the current heading
#define WPNAV_ANGLE_SPEED_XY           					1650.0f     // default maximum speed in cm/s in angle mode
#define WPNAV_ANGLE_ACCEL_XY							500.0f		// default maximum acceleration in cm/s/s in angle mode

#else
#error Please specify YUNEEC_PRODUCT_XXX

#endif

class AC_WPNav
{
public:

    // spline segment end types enum
    enum spline_segment_end_type {
        SEGMENT_END_STOP = 0,
        SEGMENT_END_STRAIGHT,
        SEGMENT_END_SPLINE
    };

    /// Constructor
    AC_WPNav(const AP_InertialNav& inav, AP_AHRS_NavEKF& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control, AP_Mission& mission, NoFlyZone &noflyzone);

    ///
    /// dynamic smart mode controller
    ///
    void follow_init();
    void follow_init(float max_follow_speed);
    const Vector2f& get_follow_desired_velocity();
    void update_obstacle_distance(bool data_valid, float distance);
    void set_follow_desired_velocity(float roll_in, float pitch_in, float follow_speed_lat, float follow_speed_lon, Vector2f follow_accel, bool smooth_break, float dt);
    void clear_follow_desired_velocity();
    void follow_update_track(const Vector2f &delta_vehicle_pos, const Vector3f &vehicle_speed, const Vector2f &delta_rc_pos);
    void clear_follow_track();
    void update_follow(float ekfGndSpdLimit, float ekfNavVelGainScaler);
    void get_track(Vector2f &desired_track, Vector2f &track_covered_pos, Vector2f &track_error, Vector2f &pilot_mov_speed, Vector2f &pilot_mov_accel, Vector2f &follow_speed, float &track_error_length_deep_filtered) {
    	desired_track = _desired_track;
    	track_covered_pos = _track_covered;
    	track_error = _track_error_filtered;
    	pilot_mov_speed = _pilot_mov_speed;
    	pilot_mov_accel = _pilot_mov_accel;
    	follow_speed = _follow_speed;
    	track_error_length_deep_filtered = _track_error_length_deep_filtered;
    }
    float get_pilot_mov_speed_length() const { return _pilot_mov_speed_length_filtered; }

    ///
    /// angle mode controller
    ///
    void init_angle_target();
    void init_angle_target(float max_speed, float max_accel, bool force_smooth_transtion=false);
    void set_angle_desired_velocity(float control_roll, float control_pitch, bool smooth_break, uint8_t ca_state, float dt);
    void clear_angle_desired_velocity();
    void update_angle(float ekfGndSpdLimit, float ekfNavVelGainScaler);
    ///
    /// loiter controller
    ///

    /// init_loiter_target to a position in cm from home
    ///     caller can set reset_I to false to preserve I term since previous time loiter controller ran.  Should only be false when caller is sure that not too much time has passed to invalidate the I terms
    void init_loiter_target(const Vector3f& position, bool reset_I=true);

    /// init_loiter_target - initialize's loiter position and feed-forward velocity from current pos and velocity
    void init_loiter_target();

    /// loiter_soften_for_landing - reduce response for landing
    void loiter_soften_for_landing();

    /// calculate_loiter_leash_length - calculates the maximum distance in cm that the target position may be from the current location
    void calculate_loiter_leash_length();

    /// set_pilot_desired_acceleration - sets pilot desired acceleration from roll and pitch stick input
    void set_pilot_desired_acceleration(float control_roll, float control_pitch);

    /// clear_pilot_desired_acceleration - clear pilot desired acceleration
    void clear_pilot_desired_acceleration() { _pilot_accel_fwd_cms = 0.0f; _pilot_accel_rgt_cms = 0.0f; }

    /// get_stopping_point - returns vector to stopping point based on a horizontal position and velocity
    void get_loiter_stopping_point_xy(Vector3f& stopping_point) const;

    /// get_loiter_distance_to_target - get horizontal distance to loiter target in cm
    float get_loiter_distance_to_target() const { return _pos_control.get_distance_to_target(); }

    /// get_loiter_bearing_to_target - get bearing to loiter target in centi-degrees
    int32_t get_loiter_bearing_to_target() const;

    /// update_loiter - run the loiter controller - should be called at 10hz
    void update_loiter(float ekfGndSpdLimit, float ekfNavVelGainScaler);

    ///
    /// brake controller
    ///
    /// init_brake_target - initialize's position and feed-forward velocity from current pos and velocity
    void init_brake_target(float accel_cmss);
    ///
    /// update_brake - run the brake controller - should be called at 400hz
    void update_brake(float ekfGndSpdLimit, float ekfNavVelGainScaler);

    ///
    /// waypoint controller
    ///

    /// wp_and_spline_init - initialise straight line and spline waypoint controllers
    ///     updates target roll, pitch targets and I terms based on vehicle lean angles
    ///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
    void wp_and_spline_init();

    /// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
    void set_speed_xy(float speed_cms);

    /// get_speed_xy - allows main code to retrieve target horizontal velocity for wp navigation
    float get_speed_xy() const { return _wp_speed_cms; }

    /// get_speed_up - returns target climb speed in cm/s during missions
    float get_speed_up() const { return WPNAV_WP_SPEED_UP; }

    /// get_speed_down - returns target descent speed in cm/s during missions.  Note: always positive
    float get_speed_down() const { return WPNAV_WP_SPEED_DOWN; }

    /// get_speed_z - returns target descent speed in cm/s during missions.  Note: always positive
    float get_accel_z() const { return WPNAV_WP_ACCEL_Z_DEFAULT; }

    /// get_wp_radius - access for waypoint radius in cm
    float get_wp_radius() const { return WPNAV_WP_RADIUS; }

    /// get_wp_acceleration - returns acceleration in cm/s/s during missions
    float get_wp_acceleration() const { return WPNAV_WP_ACCELERATION; }

    /// get_wp_destination waypoint using position vector (distance from home in cm)
    const Vector3f &get_wp_destination() const { return _destination; }

    /// set_wp_destination waypoint using position vector (distance from home in cm)
    void set_wp_destination(const Vector3f& destination);
    void set_SWP_destination(const Vector3f& destination,float speed_user_limit);

    /// set_wp_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
    void set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination);
	void set_SWP_origin_and_destination(const Vector3f& origin, const Vector3f& destination);

    /// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
    ///     used to reset the position just before takeoff
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_to_current_pos();

    /// get_wp_stopping_point_xy - calculates stopping point based on current position, velocity, waypoint acceleration
    ///		results placed in stopping_position vector
    void get_wp_stopping_point_xy(Vector3f& stopping_point) const;

    /// get_wp_distance_to_destination - get horizontal distance to destination in cm
    float get_wp_distance_to_destination() const;

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    int32_t get_wp_bearing_to_destination() const;

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_wp_destination() const { return _flags.reached_destination; }

    /// set_fast_waypoint - set to true to ignore the waypoint radius and consider the waypoint 'reached' the moment the intermediate point reaches it
    void set_fast_waypoint(bool fast) { _flags.fast_waypoint = fast; }

    /// update_wpnav - run the wp controller - should be called at 100hz or higher
    void update_wpnav();

    /// set_wpnav_stop - stop wp controller
    void set_wpnav_stop(bool stop) { _wpnav_stop = stop; }

    /// return wp nav controller stop status
    bool wpnav_stopped() { return _wpnav_stop; }

    // check_wp_leash_length - check recalc_wp_leash flag and calls calculate_wp_leash_length() if necessary
    //  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
    void check_wp_leash_length();
	void check_SWP_leash_length();

    /// calculate_wp_leash_length - calculates track speed, acceleration and leash lengths for waypoint controller
    void calculate_wp_leash_length();
	void calculate_SWP_leash_length();

    ///
    /// spline methods
    ///

    // segment start types
    // stop - vehicle is not moving at origin
    // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
    //     _flag.segment_type holds whether prev segment is straight vs spline but we don't know if it has a delay
    // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

    // segment end types
    // stop - vehicle is not moving at destination
    // straight-fast - next segment is straight, vehicle's destination velocity should be directly along track from this segment's destination to next segment's destination
    // spline-fast - next segment is spline, vehicle's destination velocity should be parallel to position difference vector from previous segment's origin to this segment's destination

    // get_yaw - returns target yaw in centi-degrees (used for wp and spline navigation)
    float get_yaw() const { return _yaw; }

    /// set_spline_destination waypoint using position vector (distance from home in cm)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    void set_spline_destination(const Vector3f& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    void set_spline_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// set_CCC_destination waypoint using position vector (distance from home in cm)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    void set_CCC_destination(const Vector3f& destination, spline_segment_end_type seg_end_type, const Vector3f& next_destination);
	void set_CWP_destination(const Vector3f& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// set_CCC_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    void set_CCC_origin_and_destination(const Vector3f& origin, const Vector3f& destination, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// reached_spline_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_spline_destination() const { return _flags.reached_destination; }

    /// update_spline - update spline controller
    void update_spline();


    ///
    ///	Curve-cable-cam spline methods
    ///

    /// init CCC mode
    void init_CCC();

    /// set_CCC_accel
    void set_CCC_accel(int16_t control_in, bool &stop_status);

    /// update_CCC - update spline curve-cable-cam controller
    void update_CCC();

    /// stop CCC immediately
    void set_CCC_stop(bool stop) { _ccc_stop = stop; }

    // return if CCC has been stopped
    bool CCC_stopped() { return _ccc_stop; }

    /// get replay look yaw for copter
    float get_repaly_look_yaw() { return _ccc_look_yaw; };

    /// get replay gimbal pitch
    float get_replay_gimbal_pitch() { return _ccc_gimbal_pitch; };

    /// get replay gimbal yaw
    float get_replay_gimbal_yaw() { return _ccc_gimbal_yaw; };

    //////Map_waypoint mode
    ////
    void init_MWP();
    void set_MapWaypoint_stop(bool flags) { _mwp_stop = flags; }
	/// set CWP_accel
	void set_CWP_accel(int16_t control_in, bool &stop_status);
	/// update cwp - update spline curve - waypoint controller
	void update_CWP();
	void update_SWP();

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); };
    int32_t get_pitch() const { return _pos_control.get_pitch(); };

    /// get_desired_alt - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    float get_desired_alt() const { return _pos_control.get_alt_target(); }

    /// set_desired_alt - set desired altitude (in cm above home)
    void set_desired_alt(float desired_alt) { _pos_control.set_alt_target(desired_alt); }

    /// advance_wp_target_along_track - move target location along track from origin to destination
    void advance_wp_target_along_track(float dt);
	void advance_SWP_target_along_track(float dt);

	float get_run_percentage() const { return _run_percentage; };

    /// get_position_bearing_cd - return bearing in centi-degrees between two positions
    float get_position_bearing_cd(const Vector3f &origin, const Vector3f &destination) const;

    bool get_wmp_stop_status() const {return _mwp_stop;}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // segment types, either straight or spine
    enum SegmentType {
        SEGMENT_STRAIGHT = 0,
        SEGMENT_SPLINE = 1
    };

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        uint8_t slowing_down            : 1;    // true when target point is slowing down before reaching the destination
        uint8_t recalc_wp_leash         : 1;    // true if we need to recalculate the leash lengths because of changes in speed or acceleration
        uint8_t new_wp_destination      : 1;    // true if we have just received a new destination.  allows us to freeze the position controller's xy feed forward
        SegmentType segment_type        : 1;    // active segment is either straight or spline
    } _flags;

    /// calc_loiter_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
    ///		updated velocity sent directly to position controller
    void calc_loiter_desired_velocity(float nav_dt, float ekfGndSpdLimit);
    void calc_follow_desired_velocity(float nav_dt, float ekfGndSpdLimit);
    void calc_angle_desired_velocity(float nav_dt, float ekfGndSpdLimit);

    /// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is traveling at full speed
    void calc_slow_down_distance(float speed_cms, float accel_cmss);

    /// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
    float get_slow_down_speed(float dist_from_dest_cm, float accel_cmss);

    /// spline protected functions

    /// update_spline_solution - recalculates hermite_spline_solution grid
    void update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel);

    /// advance_spline_target_along_track - move target location along track from origin to destination
    void advance_spline_target_along_track(float dt);

    /// calc_spline_pos_vel - update position and velocity from given spline time
    /// 	relies on update_spline_solution being called since the previous
    void calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity);

    // advance_CCC_target_along_track - move target location along track from origin to destination with user control
    void advance_CCC_target_along_track(float dt);
	void advance_CWP_target_along_track(float dt);

    /// update_cable_cam_replay - update copter yaw and gimbal pitch & yaw control output
    void update_cable_cam_replay();

    // smooth transition to manual mode
    void smooth_transition_to_manual_mode(bool force_smooth_transition = false);

    // noflyzone desired velocity limit
    void noflyzone_correct_desired_velocity(Vector2f &desired_vel);

    // references to inertial nav and ahrs libraries
    const AP_InertialNav&   _inav;
    AP_AHRS_NavEKF&   		_ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;
    AP_Mission&				_mission;
    NoFlyZone&				_noflyzone;

    // parameters
    AP_Float    _wp_speed_cms;          // maximum horizontal speed in cm/s during missions

    uint32_t 	_flight_mode_init_time;

    // break procedure variables
	float _vel_x_factor;
	float _vel_y_factor;
	float _break_accel_length;
	float _break_accel_x;
	float _break_accel_y;

    // follow controller internal variables
    bool		_start_point_initialised;
    Vector3f 	_last_position;
    bool		_low_speed_mode;
    float 		_low_speed_mode_enter_time;
    float		_max_follow_mode_speed;
    float 		_max_pilot_cmd_speed;
    Vector2f	_pilot_cmd_speed;
    Vector2f	_pilot_mov_speed;
    Vector2f	_pilot_mov_accel;
    Vector2f	_follow_speed;
    Vector2f 	_desired_track;
    Vector2f 	_track_covered;
    Vector2f    _track_error_filtered;
    LowPassFilterVector2f 	_track_error_filter;
    LowPassFilterVector2f	_track_error_deep_filter;
    LowPassFilterVector2f	_pilot_mov_speed_filter;
    AC_PID		_pid_vel_comp_x;
    AC_PID		_pid_vel_comp_y;
    float		_pilot_mov_speed_length_filtered;
    float		_follow_speed_change_rate;
    float 		_track_error_length_deep_filtered;

    struct Location _vehicle_loc;
    Vector3f _vehicle_speed;

    // angle controller internal variables
    float 		_max_speed;
    float 		_max_accel;
    float 		_angle_break_accel;
    Vector2f	_break_vel;
    Vector2f 	_break_accel;
    int16_t     _pilot_speed_fwd_cms; 	// pilot's desired acceleration forward (body-frame)
    int16_t     _pilot_speed_rgt_cms;   // pilot's desired acceleration right (body-frame)
    bool 		_break_enabled;			// enable smooth breaking procedure
    uint8_t 	_ca_state;
    LowPassFilterFloat	_yaw_filter;
    float 		_yaw_radians;
    Vector2f	_last_pilot_speed;
    Vector2f	_pilot_desired_velocity;
    uint32_t 	_hold_input_start_time = 0;

    // loiter controller internal variables
    uint8_t     _loiter_step;           // used to decide which portion of loiter controller to run during this iteration
    int16_t     _pilot_accel_fwd_cms; 	// pilot's desired acceleration forward (body-frame)
    int16_t     _pilot_accel_rgt_cms;   // pilot's desired acceleration right (body-frame)
    Vector2f    _loiter_desired_accel;  // slewed pilot's desired acceleration in lat/lon frame

    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    uint8_t     _wp_step;               // used to decide which portion of wpnav controller to run during this iteration
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    Vector3f    _pos_delta_unit;        // each axis's percentage of the total track from origin to destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _limited_speed_xy_cms;  // horizontal speed in cm/s used to advance the intermediate target towards the destination.  used to limit extreme acceleration after passing a waypoint
    float       _track_accel;           // acceleration along track
    float       _track_speed;           // speed in cm/s along track
    float       _track_leash_length;    // leash length along track
    float       _slow_down_dist;        // vehicle should begin to slow down once it is within this distance from the destination
    bool 		_wpnav_stop;
    float       _run_percentage;

    // spline variables
    float       _spline_time;           // current spline time between origin and destination
    float       _spline_time_scale;     // current spline time between origin and destination
    Vector3f    _spline_origin_vel;     // the target velocity vector at the origin of the spline segment
    Vector3f    _spline_destination_vel;// the target velocity vector at the destination point of the spline segment
    Vector3f    _hermite_spline_solution[4]; // array describing spline path between origin and destination
    float       _spline_vel_scaler;	    //
    float       _yaw;                   // heading according to yaw

    // Curve-Cable-Cam variables
    float 		_ccc_control_in;
    float 		_ccc_user_vel_limit_scale;
    float 		_ccc_vel_limit_scale;
    bool 		_ccc_direction_changed;
    bool 		_ccc_one_direction_finished;
    float 		_ccc_look_yaw;
    float 		_ccc_gimbal_yaw;
    float 		_ccc_gimbal_pitch;
    bool 		_ccc_stop;
    bool        _first_into_ccc_flag;
	float 		_record_user_vel_limit_scale;
	bool 		_last_fast_waypoint_flag;

	// MAP WAYPOINT ,seperate into curved waypoint(CWP) and straight waypoint(SWP) according to the airline type,curve or straight
	// map-waypoint variables
	float 		_mwp_control_in;
    bool        _mwp_stop;
	bool		_first_into_mwp_flag;
	bool		_mwp_gimbal_yaw;
	bool		_mwp_gimbal_pitch;
	uint16_t	_mwp_speed_user_limit;
	float		_mwp_user_vel_limit_scale;
	float 		_mwp_vel_limit_scale;

    AC_PosControl::xy_mode _pos_control_mode;

    LowPassFilterFloat _user_roll_input;
    LowPassFilterFloat _user_pitch_input;

    Vector2f 	_desired_vel;
	float 		_last_stop_timer;
    bool		_update_break_accel;
    bool 		_reset_pos_target;
    Vector2f 	_user_cmd_vel;
    bool		_waypoint_mode;
};
#endif	// AC_WPNAV_H
