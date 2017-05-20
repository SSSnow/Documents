/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_WPNav.h>
#include "../../ArtIntCopter/Copter.h"

extern const AP_HAL::HAL& hal;
extern Copter copter;

const AP_Param::GroupInfo AC_WPNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

	AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(const AP_InertialNav& inav, AP_AHRS_NavEKF& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control, AP_Mission& mission,  NoFlyZone &noflyzone) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control),
	_mission(mission),
	_noflyzone(noflyzone),
	_pid_vel_comp_x(WPNAV_FOLLOW_VEL_COMP_P,WPNAV_FOLLOW_VEL_COMP_I,WPNAV_FOLLOW_VEL_COMP_D,WPNAV_FOLLOW_VEL_COMP_IMAX,WPNAV_FOLLOW_VEL_COMP_FILT_HZ,WPNAV_UPDATE_TIME),
	_pid_vel_comp_y(WPNAV_FOLLOW_VEL_COMP_P,WPNAV_FOLLOW_VEL_COMP_I,WPNAV_FOLLOW_VEL_COMP_D,WPNAV_FOLLOW_VEL_COMP_IMAX,WPNAV_FOLLOW_VEL_COMP_FILT_HZ,WPNAV_UPDATE_TIME),
	_pilot_accel_fwd_cms(0),
	_pilot_accel_rgt_cms(0),
    _wp_last_update(0),
    _wp_step(0),
    _track_length(0.0f),
    _track_desired(0.0f),
    _limited_speed_xy_cms(0.0f),
    _track_accel(0.0f),
    _track_speed(0.0f),
    _track_leash_length(0.0f),
    _slow_down_dist(0.0f),
    _spline_time(0.0f),
    _spline_time_scale(0.0f),
    _spline_vel_scaler(0.0f),
    _yaw(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;
    _flags.slowing_down = false;
    _flags.recalc_wp_leash = false;
    _flags.new_wp_destination = false;
    _flags.segment_type = SEGMENT_STRAIGHT;
    _user_pitch_input.set_cutoff_frequency(2.0f);
    _user_roll_input.set_cutoff_frequency(2.0f);
    _pos_control.init_xy_controller(false);
	_break_vel.zero();
	_break_accel.zero();
	_user_cmd_vel.zero();
	_yaw_filter.set_cutoff_frequency(10.0f);
}
	

///
/// waypoint navigation
///

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav::wp_and_spline_init()
{
    // initialise position controller speed and acceleration
	_wp_speed_cms = WPNAV_WP_SPEED;
    _pos_control.set_speed_xy(WPNAV_WP_SPEED);
    _pos_control.set_accel_xy(WPNAV_WP_ACCELERATION);
    _pos_control.set_speed_z(-WPNAV_WP_SPEED_DOWN, WPNAV_WP_SPEED_UP);
    _pos_control.set_accel_z(WPNAV_WP_ACCEL_Z_DEFAULT);
    _pos_control.set_alt_accel(WPNAV_WP_ACCEL_Z_DEFAULT);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();
    _wpnav_stop = false;
}

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check new target speed and update position controller
    if (speed_cms >= WPNAV_WP_SPEED_MIN) {
    	_wp_speed_cms = speed_cms;
        _pos_control.set_speed_xy(speed_cms);
        // flag that wp leash must be recalculated
        _flags.recalc_wp_leash = true;
    }
}




// check_wp_leash_length - check if waypoint leash lengths need to be recalculated
//  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
void AC_WPNav::check_wp_leash_length()
{
    // exit immediately if recalc is not required
    if (_flags.recalc_wp_leash) {
        calculate_wp_leash_length();
    }
}

// check_wp_leash_length - check if waypoint leash lengths need to be recalculated
//  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
void AC_WPNav::check_SWP_leash_length()
{
    // exit immediately if recalc is not required
    if (_flags.recalc_wp_leash) {
        calculate_SWP_leash_length();
    }
}

/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
void AC_WPNav::calculate_wp_leash_length()
{
    // length of the unit direction vector in the horizontal
    float pos_delta_unit_xy = pythagorous2(_pos_delta_unit.x, _pos_delta_unit.y);
    float pos_delta_unit_z = fabsf(_pos_delta_unit.z);

    float speed_z;
    float leash_z;
    if (_pos_delta_unit.z >= 0.0f) {
        speed_z = WPNAV_WP_SPEED_UP;
        leash_z = _pos_control.get_leash_up_z();
    }else{
        speed_z = WPNAV_ANGLE_SPEED_DOWN;
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
    if(is_zero(pos_delta_unit_z) && is_zero(pos_delta_unit_xy)){
        _track_accel = 0;
        _track_speed = 0;
        _track_leash_length = WPNAV_WP_LEASH_LENGTH_MIN;
    }else if(is_zero(pos_delta_unit_z)){
        _track_accel = WPNAV_WP_ACCELERATION/pos_delta_unit_xy;
        _track_speed = _wp_speed_cms/pos_delta_unit_xy;
        _track_leash_length = _pos_control.get_leash_xy()/pos_delta_unit_xy;
    }else if(is_zero(pos_delta_unit_xy)){
        _track_accel = WPNAV_WP_ACCEL_Z_DEFAULT/pos_delta_unit_z;
        _track_speed = speed_z/pos_delta_unit_z;
        _track_leash_length = leash_z/pos_delta_unit_z;
    }else{
        _track_accel = min(WPNAV_WP_ACCEL_Z_DEFAULT/pos_delta_unit_z, WPNAV_WP_ACCELERATION/pos_delta_unit_xy);
        _track_speed = min(speed_z/pos_delta_unit_z, _wp_speed_cms/pos_delta_unit_xy);
        _track_leash_length = min(leash_z/pos_delta_unit_z, _pos_control.get_leash_xy()/pos_delta_unit_xy);
    }

    // calculate slow down distance (the distance from the destination when the target point should begin to slow down)
    calc_slow_down_distance(_track_speed, _track_accel);

    // set recalc leash flag to false
    _flags.recalc_wp_leash = false;
}

/// set_spline_destination waypoint using position vector (distance from home in cm)
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
void AC_WPNav::set_CCC_destination(const Vector3f& destination, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    Vector3f origin;

    // if waypoint controller is active and copter has reached the previous waypoint use current pos target as the origin
    if ((hal.scheduler->millis() - _wp_last_update) < 1000) {
        origin = _pos_control.get_pos_target();
    }else{
        // otherwise calculate origin from the current position and velocity
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // set origin and destination
    set_CCC_origin_and_destination(origin, destination, seg_end_type, next_destination);
}

/// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     seg_type should be calculated by calling function based on the mission
void AC_WPNav::set_CCC_origin_and_destination(const Vector3f& origin, const Vector3f& destination, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    float dt = _pos_control.get_dt_xy();

    bool update_hermite_solution = true;

    // calculate spline velocity at origin
	if (_mission.get_current_nav_index() == AP_MISSION_CCC_START_COMMAND) {
		_spline_origin_vel = (destination - origin);
        _spline_destination_vel = _spline_origin_vel;
		_spline_time = 0.0f;
		_spline_vel_scaler = 0.0f;
		_ccc_user_vel_limit_scale = 0.0f;
        _flags.fast_waypoint = false;
	} else if ((_mission.get_current_nav_index() == AP_MISSION_FIRST_REAL_COMMAND) && (_mission.get_prev_nav_cmd_index() == AP_MISSION_CCC_START_COMMAND)) {
		_spline_origin_vel = _spline_destination_vel;
        _spline_destination_vel = (next_destination - origin) * dt;
		_spline_time = 0.0f;
		_spline_vel_scaler = 0.0f;
		_ccc_user_vel_limit_scale = 0.0f;
	} else {
		// if direction has been changed, then we need to reverse velocity
		if (_ccc_direction_changed) {
			_ccc_direction_changed = false;
			if (_last_fast_waypoint_flag == false && _flags.fast_waypoint == true) {
		        _flags.fast_waypoint = false;
			} else {
				_flags.fast_waypoint = true;
			}
			// set spline time
			if (_ccc_one_direction_finished) {
				_ccc_one_direction_finished = false;
			    switch (_mission.get_cable_cam_direction()) {
			    case AP_Mission::MISSION_DIRECTION_FORWARD:
			    	_spline_time = 0.0f;
			        break;
			    case AP_Mission::MISSION_DIRECTION_BACKWARD:
			    	_spline_time = 1.0f;
			    	break;
			    }
				_spline_vel_scaler = 0.0f;
			}

			update_hermite_solution = false;
		} else {
		    switch (_mission.get_cable_cam_direction()) {
		    case AP_Mission::MISSION_DIRECTION_FORWARD:
				_spline_origin_vel = _spline_destination_vel;
			    // calculate spline velocity at destination
			    switch (seg_end_type) {

			    case SEGMENT_END_STOP:
			        // if vehicle stops at the destination set destination velocity to 0.1 * distance vector from origin to destination
			        _spline_destination_vel = (destination - origin) * dt;
			        _flags.fast_waypoint = false;
			        _last_fast_waypoint_flag = false;
			        break;

			    case SEGMENT_END_STRAIGHT:
			        // if next segment is straight, vehicle's final velocity should face along the next segment's position
			        _spline_destination_vel = (next_destination - destination);
			        _flags.fast_waypoint = true;
			        _last_fast_waypoint_flag = true;
			        break;

			    case SEGMENT_END_SPLINE:
			        // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
			    	if (_mission.get_current_nav_index() == 1) {
				        _spline_destination_vel = (destination - origin) * dt;
				        _flags.fast_waypoint = false;
				        _last_fast_waypoint_flag = false;
			    	} else {
			    		_spline_destination_vel = (next_destination - origin);
				        _flags.fast_waypoint = true;
				        _last_fast_waypoint_flag = true;
			    	}
			        break;
			    }

		    	_spline_time = 0.0f;
		        break;
		    case AP_Mission::MISSION_DIRECTION_BACKWARD:
		    	_spline_destination_vel = _spline_origin_vel;
			    // calculate spline velocity at destination
			    switch (seg_end_type) {

			    case SEGMENT_END_STOP:
			        // if vehicle stops at the destination set destination velocity to 0.1 * distance vector from origin to destination
			    	_spline_origin_vel = (destination - origin) * dt;
			        _flags.fast_waypoint = false;
			        _last_fast_waypoint_flag = false;
			        break;

			    case SEGMENT_END_STRAIGHT:
			        // if next segment is straight, vehicle's final velocity should face along the next segment's position
			    	_spline_origin_vel = (next_destination - destination);
			        _flags.fast_waypoint = true;
			        _last_fast_waypoint_flag = true;
			        break;

			    case SEGMENT_END_SPLINE:
			        // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
			    	if (_mission.get_current_nav_index() == AP_MISSION_FIRST_REAL_COMMAND) {
			    		_spline_origin_vel = (destination - origin) * dt;
				        _flags.fast_waypoint = false;
				        _last_fast_waypoint_flag = false;
			    	} else {
			    		_spline_origin_vel = (next_destination - origin);
				        _flags.fast_waypoint = true;
				        _last_fast_waypoint_flag = true;
			    	}
			        break;
			    }
			    _spline_origin_vel = -_spline_origin_vel;
		    	_spline_time = 1.0f;
		    	break;
		    }

			update_hermite_solution = true;
		}
	}

	if (update_hermite_solution) {
		Vector3f origin_solution, destination_solution;

	    switch (_mission.get_cable_cam_direction()) {
	    case AP_Mission::MISSION_DIRECTION_FORWARD:
	    	origin_solution = origin;
	    	destination_solution = destination;
	        break;
	    case AP_Mission::MISSION_DIRECTION_BACKWARD:
	    	origin_solution = destination;
	    	destination_solution = origin;
	    	break;
	    }

		// code below ensures we don't get too much overshoot when the next segment is short
		float vel_len = _spline_origin_vel.length() + _spline_destination_vel.length();
		float pos_len = (destination_solution - origin_solution).length() * 4.0f;
		if (vel_len > pos_len) {
			// if total start+stop velocity is more than twice position difference
			// use a scaled down start and stop velocity scale the start and stop velocities down
			float vel_scaling = pos_len / vel_len;
			// update spline calculator
			update_spline_solution(origin_solution, destination_solution, _spline_origin_vel * vel_scaling, _spline_destination_vel * vel_scaling);
		}else{
			// update spline calculator
			update_spline_solution(origin_solution, destination_solution, _spline_origin_vel, _spline_destination_vel);
		}
	}

    // store origin and destination locations
    _origin = origin;
    _destination = destination;

    // initialise yaw heading to current heading
    _yaw = _attitude_control.angle_ef_targets().z;

    // calculate slow down distance
    calc_slow_down_distance(_wp_speed_cms, WPNAV_WP_ACCELERATION);

    // initialise intermediate point to the origin
    _flags.reached_destination = false;
    _flags.segment_type = SEGMENT_SPLINE;
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
}


/// init CCC - reset variables
void AC_WPNav::init_CCC()
{
	_ccc_stop = false;
    _ccc_control_in = 0.0f;
    _ccc_user_vel_limit_scale = 0.0f;
    _ccc_vel_limit_scale = 0.0f;
    _ccc_direction_changed = false;
    _ccc_one_direction_finished = false;
    _ccc_look_yaw = 0.0f;
    _ccc_gimbal_yaw = 0.0f;
    _ccc_gimbal_pitch = 0.0f;
    _first_into_ccc_flag = true;
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();
}

/// set_CCC_accel - set user' command acceleration
void AC_WPNav::set_CCC_accel(int16_t control_in, bool &stop_status)
{
	// check if user command stop before and restart the mission using stick
	if (_ccc_stop && abs(control_in) > 100) {
		_ccc_stop = false;
		_ccc_user_vel_limit_scale = 0.0f;
		_record_user_vel_limit_scale = 0.0f;
	}

	stop_status = _ccc_stop;

	_ccc_control_in = (float)control_in/WPNAV_ROLL_PITCH_INPUT_MAX;
}

/// update_spline - update spline controller
void AC_WPNav::update_CCC()
{
    // exit immediately if this is not a spline segment
    if (_flags.segment_type != SEGMENT_SPLINE) {
        return;
    }

    float dt = _pos_control.time_since_last_xy_update();

    // run at poscontrol update rate
    if (dt >= _pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

		// advance the target if necessary
		advance_CCC_target_along_track(dt);

		// update cable cam replay output
		update_cable_cam_replay();

        if (_flags.new_wp_destination) {
            _flags.new_wp_destination = false;
        }

        // run horizontal position controller
        _pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, 1.0f);
        check_wp_leash_length();

        _wp_last_update = hal.scheduler->millis();
    }
}


/// update_spline_solution - recalculates hermite_spline_solution grid
///		relies on _spline_origin_vel, _spline_destination_vel and _origin and _destination
void AC_WPNav::update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel)
{
    _hermite_spline_solution[0] = origin;
    _hermite_spline_solution[1] = origin_vel;
    _hermite_spline_solution[2] = -origin*3.0f -origin_vel*2.0f + dest*3.0f - dest_vel;
    _hermite_spline_solution[3] = origin*2.0f + origin_vel -dest*2.0f + dest_vel;
 }


/// advance_CCC_target_along_track - move target location along track from origin to destination
void AC_WPNav::advance_CCC_target_along_track(float dt)
{
	static float desired_control_in = 0.0f;
	static bool user_vel_limit_scale_recorded = false;

	if (_first_into_ccc_flag) {
		desired_control_in = 0.0f;
	}
	
    // smooth user input
	if (_mission.get_current_nav_index() == AP_MISSION_CCC_START_COMMAND) {
		_ccc_user_vel_limit_scale = 0.5f;
	} else {
		if (_ccc_stop) {
			if (!user_vel_limit_scale_recorded) {
				user_vel_limit_scale_recorded = true;
				_record_user_vel_limit_scale = _ccc_user_vel_limit_scale;
			}
			switch (_mission.get_cable_cam_direction()) {
			case AP_Mission::MISSION_DIRECTION_FORWARD:
				if (_ccc_user_vel_limit_scale > 0.2f) {
					_ccc_control_in = -1.0f;
				} else {
					_ccc_user_vel_limit_scale = 0.0f;
				}
				break;
			case AP_Mission::MISSION_DIRECTION_BACKWARD:
				if (_ccc_user_vel_limit_scale < -0.2f) {
					_ccc_control_in = 1.0f;
				} else {
					_ccc_user_vel_limit_scale = 0.0f;
				}
				break;
			}
		} else {
			user_vel_limit_scale_recorded = false;
			if (_record_user_vel_limit_scale != 0.0f) {
				switch (_mission.get_cable_cam_direction()) {
				case AP_Mission::MISSION_DIRECTION_FORWARD:
					if (_ccc_user_vel_limit_scale < _record_user_vel_limit_scale) {
						_ccc_control_in = 1.0f;
					} else {
						_record_user_vel_limit_scale = 0.0f;
					}
					break;
				case AP_Mission::MISSION_DIRECTION_BACKWARD:
					if (_ccc_user_vel_limit_scale > _record_user_vel_limit_scale) {
						_ccc_control_in = -1.0f;
					} else {
						_record_user_vel_limit_scale = 0.0f;
					}
					break;
				}
			}
		}
		float diff_control_in = (_ccc_control_in - desired_control_in);
		float gain = 1.0f;
		if (fabs(_ccc_control_in) > 0.8f)			gain = 2.0f;
		else if (fabs(_ccc_control_in) > 0.6f)		gain = 1.5f;
		else if (fabs(_ccc_control_in) > 0.4f)		gain = 1.0f;
		else if (fabs(_ccc_control_in) > 0.2f)		gain = 0.8f;
		else										gain = 0.6f;

		desired_control_in += gain*diff_control_in*dt;
		_ccc_user_vel_limit_scale += desired_control_in*dt;
		_ccc_user_vel_limit_scale = constrain_float(_ccc_user_vel_limit_scale, -1.0f, 1.0f);
	}

    // if it is the first time entering into ccc, don't allow negative control-in 
    if(_first_into_ccc_flag && (_ccc_user_vel_limit_scale < 0.0f)){
        _ccc_user_vel_limit_scale = 0.0f;
    }
    // when it get into 2 index, make flag false to stop
    if(_mission.get_current_nav_index() > 1){
        _first_into_ccc_flag = false;
    }

    // update cable cam direction according to user command and current velocity
    switch (_mission.get_cable_cam_direction()) {
    case AP_Mission::MISSION_DIRECTION_FORWARD:
    	if (_ccc_user_vel_limit_scale < 0.0f && _ccc_user_vel_limit_scale > -0.1f) { _ccc_vel_limit_scale = 0.0f; }
    	else { _ccc_vel_limit_scale = _ccc_user_vel_limit_scale; }
        if (_ccc_vel_limit_scale < 0.0f) {
        	_ccc_direction_changed = true;
        	_ccc_one_direction_finished = _mission.one_direction_finished();
        	_mission.set_cable_cam_direction(AP_Mission::MISSION_DIRECTION_BACKWARD);
        }
        break;
    case AP_Mission::MISSION_DIRECTION_BACKWARD:
    	if (_ccc_user_vel_limit_scale > 0.0f && _ccc_user_vel_limit_scale < 0.1f) { _ccc_vel_limit_scale = 0.0f; }
    	else { _ccc_vel_limit_scale = -_ccc_user_vel_limit_scale; }
        if (_ccc_vel_limit_scale < 0.0f) {
        	_ccc_direction_changed = true;
        	_ccc_one_direction_finished = _mission.one_direction_finished();
        	_mission.set_cable_cam_direction(AP_Mission::MISSION_DIRECTION_FORWARD);
        }
    	break;
    }

    if (!_flags.reached_destination) {
        Vector3f target_pos, target_vel;

        // update target position and velocity from spline calculator
        calc_spline_pos_vel(_spline_time, target_pos, target_vel);

        _pos_delta_unit = target_vel/target_vel.length();
        calculate_wp_leash_length();

        // get current location
        Vector3f curr_pos = _inav.get_position();
        Vector3f track_error = curr_pos - target_pos;

        // calculate the horizontal error
        float track_error_xy = pythagorous2(track_error.x, track_error.y);

        // calculate the vertical error
        float track_error_z = fabsf(track_error.z);

        // get position control leash lengths
        float leash_xy = _pos_control.get_leash_xy();
        float leash_z;
        if (track_error.z >= 0)	leash_z = _pos_control.get_leash_up_z();
        else			        leash_z = _pos_control.get_leash_down_z();

        // calculate how far along the track we could move the intermediate target before reaching the end of the leash
        float track_leash_slack = min(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-track_error_xy)/leash_xy);
        if (track_leash_slack < 0.0f) {
            track_leash_slack = 0.0f;
        }

        // update velocity
        float spline_dist_to_wp = (_destination - target_pos).length();
        float vel_limit = _wp_speed_cms * _ccc_vel_limit_scale;
        if (dt > 0.0f) {
            vel_limit = min(vel_limit, track_leash_slack/dt);
        }

        // if within the stopping distance from destination, set target velocity to sqrt of distance * 2 * acceleration
        if (!_flags.fast_waypoint && spline_dist_to_wp < _slow_down_dist) {
            _spline_vel_scaler = safe_sqrt(spline_dist_to_wp * 2.0f * WPNAV_WP_ACCELERATION);
        } else {
            // increase/decrease velocity using acceleration
        	if (_spline_vel_scaler > vel_limit)			_spline_vel_scaler -= WPNAV_WP_ACCELERATION * dt;
        	else if (_spline_vel_scaler < vel_limit)	_spline_vel_scaler += WPNAV_WP_ACCELERATION * dt;
        }

        // constrain target velocity
        _spline_vel_scaler = constrain_float(_spline_vel_scaler, 0.0f, vel_limit);

        // scale the spline_time by the velocity we've calculated vs the velocity that came out of the spline calculator
        float target_vel_length = target_vel.length();
        if (!is_zero(target_vel_length)) {
            _spline_time_scale = _spline_vel_scaler/target_vel_length;
        }

        // update target position
        _pos_control.set_pos_target(target_pos);

        // update the yaw
        _yaw = RadiansToCentiDegrees(atan2f(target_vel.y,target_vel.x));

        // advance spline time to next step
        switch (_mission.get_cable_cam_direction()) {
        case AP_Mission::MISSION_DIRECTION_FORWARD:
            _spline_time += _spline_time_scale*dt;
            if (_spline_time >= 1.0f) {
                _flags.reached_destination = true;
            }
            break;
        case AP_Mission::MISSION_DIRECTION_BACKWARD:
            _spline_time -= _spline_time_scale*dt;
            if (_spline_time <= 0.0f) {
                _flags.reached_destination = true;
            }
        	break;
        }

    }
}

void AC_WPNav::update_cable_cam_replay()
{
	AP_Mission::cableCamPoint prev_cable_cam_point, cur_cable_cam_point;

	// return if can't get current or next cable cam point
    switch (_mission.get_cable_cam_direction()) {
    case AP_Mission::MISSION_DIRECTION_FORWARD:
    	if (!_mission.get_prev_cable_cam_point(prev_cable_cam_point) || !_mission.get_current_cable_cam_point(cur_cable_cam_point)) {
    		return;
    	}
        break;
    case AP_Mission::MISSION_DIRECTION_BACKWARD:
    	if (!_mission.get_prev_cable_cam_point(cur_cable_cam_point) || !_mission.get_current_cable_cam_point(prev_cable_cam_point)) {
    		return;
    	}
    	break;
    }

	/*
	 * update copter replay yaw
	 */
	float angle_to_small = 0.0f;
	float angle_dir = 1.0f;
	float abs_angle = fabs(prev_cable_cam_point.copter_yaw - cur_cable_cam_point.copter_yaw);

	if (abs_angle >= 18000.0f) {
		angle_to_small = 36000.0f - abs_angle;
		angle_dir = (prev_cable_cam_point.copter_yaw >= cur_cable_cam_point.copter_yaw) ? 1.0f : -1.0f;
	} else {
		angle_to_small = cur_cable_cam_point.copter_yaw - prev_cable_cam_point.copter_yaw;
		angle_dir = 1.0f;
	}

	_ccc_look_yaw = prev_cable_cam_point.copter_yaw + angle_dir * angle_to_small * _spline_time;

	if (_ccc_look_yaw >= 36000.0f)	 _ccc_look_yaw -= 36000.0f;
	else if (_ccc_look_yaw <= 0.0f)  _ccc_look_yaw += 36000.0f;

	/*
	 * update gimbal replay pitch/yaw
	 */
	angle_to_small = 0.0f;
	abs_angle = fabs(prev_cable_cam_point.gimbal_yaw - cur_cable_cam_point.gimbal_yaw);

	if (abs_angle >= 18000.0f) {
		angle_to_small = 36000.0f - abs_angle;
		angle_dir = (prev_cable_cam_point.gimbal_yaw >= cur_cable_cam_point.gimbal_yaw) ? 1.0f : -1.0f;
	} else {
		angle_to_small = cur_cable_cam_point.gimbal_yaw - prev_cable_cam_point.gimbal_yaw;
		angle_dir = 1.0f;
	}

	_ccc_gimbal_yaw = prev_cable_cam_point.gimbal_yaw + angle_dir * angle_to_small * _spline_time;

	if(_ccc_gimbal_yaw >= 36000.0f)		_ccc_gimbal_yaw -= 36000.0f;
	else if (_ccc_gimbal_yaw <= 0.0f)	_ccc_gimbal_yaw += 36000.0f;

	//hal.uartG->printf("pre pitch:%f\n",prev_cable_cam_point.gimbal_pitch);
	//hal.uartG->printf("cur pitch:%f\n",cur_cable_cam_point.gimbal_pitch);
	//hal.uartG->printf("spline time:%f\n",_spline_time);
	_ccc_gimbal_pitch = prev_cable_cam_point.gimbal_pitch + (cur_cable_cam_point.gimbal_pitch - prev_cable_cam_point.gimbal_pitch) * _spline_time;


}

// calc_spline_pos_vel_accel - calculates target position, velocity and acceleration for the given "spline_time"
/// 	relies on update_spline_solution being called when the segment's origin and destination were set
void AC_WPNav::calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity)
{
    float spline_time_sqrd = spline_time * spline_time;
    float spline_time_cubed = spline_time_sqrd * spline_time;

    position = _hermite_spline_solution[0] + \
               _hermite_spline_solution[1] * spline_time + \
               _hermite_spline_solution[2] * spline_time_sqrd + \
               _hermite_spline_solution[3] * spline_time_cubed;

    velocity = _hermite_spline_solution[1] + \
               _hermite_spline_solution[2] * 2.0f * spline_time + \
               _hermite_spline_solution[3] * 3.0f * spline_time_sqrd;
}


///
/// shared methods
///

// get_position_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AC_WPNav::get_position_bearing_cd(const Vector3f &origin, const Vector3f &destination) const
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed
void AC_WPNav::calc_slow_down_distance(float speed_cms, float accel_cmss)
{
	// protect against divide by zero
	if (accel_cmss <= 0.0f) {
		_slow_down_dist = 0.0f;
		return;
	}
    // To-Do: should we use a combination of horizontal and vertical speeds?
    // To-Do: update this automatically when speed or acceleration is changed
    _slow_down_dist = speed_cms * speed_cms / (4.0f*accel_cmss);
}

/// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
float AC_WPNav::get_slow_down_speed(float dist_from_dest_cm, float accel_cmss)
{
    // return immediately if distance is zero (or less)
    if (dist_from_dest_cm <= 0) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    }

    // calculate desired speed near destination
    float target_speed = safe_sqrt(dist_from_dest_cm * 4.0f * accel_cmss);

    // ensure desired speed never becomes too low
    if (target_speed < WPNAV_WP_TRACK_SPEED_MIN) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    } else {
        return target_speed;
    }
}
