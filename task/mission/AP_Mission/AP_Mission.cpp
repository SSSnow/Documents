// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"
#include <AP_Math.h>
#include <AP_Terrain.h>
#include "../../ArtIntCopter/Copter.h"

const AP_Param::GroupInfo AP_Mission::var_info[] PROGMEM = {

    // @Param: TOTAL
    // @DisplayName: Total mission commands
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 32766
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TOTAL",  0, AP_Mission, _cmd_total, 0),

    // @Param: RESTART
    // @DisplayName: Mission Restart when entering Auto mode
    // @Description: Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
    // @Values: 0:Resume Mission, 1:Restart Mission
    AP_GROUPINFO("RESTART",  1, AP_Mission, _restart, AP_MISSION_RESTART_DEFAULT),

    // @Param: CCC_NUM
    // @DisplayName: Maximum number of replay state
    // @Description: Maximum number of replay state
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CCC_NUM",  2, AP_Mission, _max_ccc_points, 100),

	// @Param: MAP_NUM
    // @DisplayName: Maximum number of map waypoint
    // @Description: Maximum number of map waypoint
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
	AP_GROUPINFO("MAP_NUM", 3, AP_Mission, _max_map_waypoints, 100),

	// @Param: MAP_DO_CMD_NUM
    // @DisplayName: Maximum number of map waypoint_do_cmd
    // @Description: Maximum number of map waypoint_do_cmd
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
	AP_GROUPINFO("MAP_DOCMD_NUM", 4, AP_Mission, _max_map_waypoints_docmd, 20),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;
#define DEBUG_UARTG_PRINTF1(x)    //hal.uartG->printf(x)
#define DEBUG_UARTG_PRINTF2(x,y)  //hal.uartG->printf(x,y)

// storage object
StorageAccess AP_Mission::_storage(StorageManager::StorageMission);

/// constructor
AP_Mission::AP_Mission(AP_AHRS &ahrs, const int32_t &home_bearing, mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn) :
    _ahrs(ahrs),
	_home_bearing(home_bearing),
	_mission_type(MISSION_TYPE_MAVLINK),
	_cable_cam_points_total(0),
	_cable_cam_point(NULL),
	_cable_cam_initiated(false),
	_cable_cam_direction(MISSION_DIRECTION_FORWARD),
	_map_waypoint_initiated(false),
	_map_waypoint_total(0),
	_map_waypoint_point(NULL),
	_one_direction_finished(false),
    _cmd_start_fn(cmd_start_fn),
    _cmd_verify_fn(cmd_verify_fn),
    _mission_complete_fn(mission_complete_fn),
    _prev_nav_cmd_index(AP_MISSION_CMD_INDEX_NONE),
    _last_change_time_ms(0),
    _pre_auto_takeoff_enabled(false),
    _auto_takeoff_alt(MAP_WAYPOINT_AUTO_TAKEOFF_ALT),
    _map_airline_ending_cycle_enable(false),
    _map_airline_ending_land_enable(false),
    _yaw_looking(LOOK_AT_HOLD),
    _gimbal_tilt(AP_MISSION_CMD_GIMBAL_ANGLE_NONE),
    _gimbal_pan(AP_MISSION_CMD_GIMBAL_ANGLE_NONE)
{
    // load parameter defaults
    AP_Param::setup_object_defaults(this, var_info);

    // clear commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
	_pre_autotakeoff_cmd.index = AP_MISSION_CMD_INDEX_NONE;

    // initialise other internal variables
    _flags.state = MISSION_STOPPED;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;
}

///
/// public mission methods
///

/// init - initialises this library including checks the version in eeprom matches this library
void AP_Mission::init()
{
    // check_eeprom_version - checks version of missions stored in eeprom matches this library
    // command list will be cleared if they do not match
    check_eeprom_version();

    // limit maximum number of replay state and set to EEPROM
    uint8_t max_ccc_points = constrain_int8(_max_ccc_points.get(), AP_MISSION_MIN_REPLAY_NUM, AP_MISSION_MAX_REPLAY_NUM);
    _max_ccc_points.set_and_save_ifchanged(max_ccc_points);

    // allocate memory for replay state
    // the drone will run only one mode ,ccc or mapwaypoint,so  the two points shares the same  memery ,
    // spaces used by MapWayPoint  is large than cableCamPoint,
    size_t waypoints_size = _max_map_waypoints*(sizeof(MapWaypoint)+12);
    size_t ccc_size = _max_ccc_points*sizeof(cableCamPoint);
    size_t memory_size = max(waypoints_size, ccc_size);
    void * addr = malloc(memory_size);
	_map_waypoint_point = (MapWaypoint *)addr;
	_cable_cam_point = (cableCamPoint *)addr;

    // check if allocated successfully
    if (_cable_cam_point == NULL) {
    	_cable_cam_initiated = false;
    	_map_waypoint_initiated = false;
#if DRIVER_LEVEL_CHECK == 1
        hal.scheduler->panic(PSTR("AP_Mission Can't allocate memory for replay state"));
#endif
    } else {
    	_cable_cam_initiated = true;
    	_map_waypoint_initiated = true;
        memset(addr, 0, memory_size);
    }

    // prevent an easy programming error, this will be optimised out
#if DRIVER_LEVEL_CHECK == 1
    if (sizeof(union Content) != 12) {
        hal.scheduler->panic(PSTR("AP_Mission Content must be 12 bytes"));
    }
#endif

    _last_change_time_ms = hal.scheduler->millis();
}

// return true if set type of mission successfully
bool AP_Mission::set_mission_type(enum mission_type type)
{
	bool result = false;

	switch (type) {
	case MISSION_TYPE_MAVLINK:
		_mission_type = MISSION_TYPE_MAVLINK;
		result = true;
		break;
	case MISSION_TYPE_CABLECAM:
		if (_cable_cam_initiated) {
			_mission_type = MISSION_TYPE_CABLECAM;
			result = true;
		}
		break;
	case MISSION_TYPE_MAPWAYPOINT:
		if (_map_waypoint_initiated) {
			_mission_type = MISSION_TYPE_MAPWAYPOINT;
			result = true;
		}
		break;
	}

	return result;
}

/// num_commands - returns total number of commands in the mission
uint16_t AP_Mission::num_commands() const
{
	switch (_mission_type) {
	case MISSION_TYPE_MAVLINK:
	{
		return _cmd_total;
		break;
	}
	case MISSION_TYPE_CABLECAM:
	{
		// if user already set cable cam point, add one for home point
		// the state machine of mavlink type mission need user to add
		// home point first and the first real command is start from 1
		// we follow this rule but don't need user to set home point first
		if (_cable_cam_points_total == 0) {
			return 0;
		} else {
			return _cable_cam_points_total + 1;
		}
		break;
	}
	case MISSION_TYPE_MAPWAYPOINT:
	{
		//same with ccc mode
		if (_map_waypoint_total == 0) {
			return 0;
		} else {
			return _map_waypoint_total + 1;
		}
		break;
	}
	default:
		break;
	}

	return 0;
}

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AP_Mission::start()
{
    _flags.state = MISSION_RUNNING;

    reset(); // reset mission to the first command, resets jump tracking
    
	switch (_mission_type) {
	case MISSION_TYPE_MAVLINK:
	{
	    // advance to the first command
	    if (!advance_current_nav_cmd()) {
	        // on failure set mission complete
	        complete();
	    }

		break;
	}
	case MISSION_TYPE_CABLECAM:
	{
	    // advance to the first command
	    if (!advance_current_CCC_cmd()) {
	        // on failure set mission complete
	        complete();
	    }

		break;
	}
	case MISSION_TYPE_MAPWAYPOINT:
	{
		if (_pre_auto_takeoff_enabled && _map_waypoint_total > 1) {
			_pre_autotakeoff_cmd.content.location.alt = _start_map_waypoint_point.alt;		//auto takeoff alt 500cm
			_pre_autotakeoff_cmd.id = MAV_CMD_NAV_TAKEOFF;
			_pre_autotakeoff_cmd.index = AP_MISSION_CMD_INDEX_NONE;
			_cmd_start_fn(_pre_autotakeoff_cmd);
		} else {
			//advance to tht first command
			if (!advance_current_MWP_nav_cmd())	{
				complete();	//have not confirm the condition about mwp
			}
		}

		break;
	}
	default:
		break;
	}
}

/// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
void AP_Mission::stop()
{
    _flags.state = MISSION_STOPPED;
}

/// resume - continues the mission execution from where we last left off
///     previous running commands will be re-initialised
void AP_Mission::resume()
{
    // if mission had completed then start it from the first command
    if (_flags.state == MISSION_COMPLETE) {
        start();
        return;
    }

    // if mission had stopped then restart it
    if (_flags.state == MISSION_STOPPED) {
        _flags.state = MISSION_RUNNING;

        // if no valid nav command index restart from beginning
        if (_nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
            start();
            return;
        }
    }

    // ensure cache coherence
    // don't bother checking if the read is successful. If it fails _nav_cmd
    // won't change and we'll continue flying the old cached command.
    read_cmd_from_storage(_nav_cmd.index, _nav_cmd);

	switch (_mission_type) {
	case MISSION_TYPE_MAVLINK:
	{
	    // restart active navigation command. We run these on resume()
	    // regardless of whether the mission was stopped, as we may be
	    // re-entering AUTO mode and the nav_cmd callback needs to be run
	    // to setup the current target waypoint

	    // Note: if there is no active command then the mission must have been stopped just after the previous nav command completed
	    //      update will take care of finding and starting the nav command
	    if (_flags.nav_cmd_loaded) {
	        _cmd_start_fn(_nav_cmd);
	    }

	    // restart active do command
	    if (_flags.do_cmd_loaded && _do_cmd.index != AP_MISSION_CMD_INDEX_NONE) {
	        _cmd_start_fn(_do_cmd);
	    }

		break;
	}
	case MISSION_TYPE_CABLECAM:
	{
	    if (_flags.nav_cmd_loaded) {
	        _cmd_start_fn(_nav_cmd);
	    }

		break;
	}
	case MISSION_TYPE_MAPWAYPOINT:
	{
		if (_flags.nav_cmd_loaded) {
			_cmd_start_fn(_nav_cmd);
		} else if (_flags.do_cmd_loaded && _do_cmd.index != AP_MISSION_CMD_INDEX_NONE) {
			_cmd_start_fn(_do_cmd);
		}
	}
	default:
		break;
	}

}

/// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
void AP_Mission::start_or_resume()
{
    if (_restart) {
        start();
    } else {
        resume();
    }
}

/// reset - reset mission to the first command
void AP_Mission::reset()
{
    _flags.nav_cmd_loaded  = false;
    _flags.do_cmd_loaded   = false;
    _flags.do_cmd_all_done = false;
    _nav_cmd.index         = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index          = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_index    = AP_MISSION_CMD_INDEX_NONE;
    init_jump_tracking();
}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{
	switch (_mission_type) {
	case MISSION_TYPE_MAVLINK:
	{
	    // exit immediately if not running or no mission commands
	    if (_flags.state != MISSION_RUNNING || _cmd_total == 0) {
	        return;
	    }

	    // check if we have an active nav command
	    if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
	        // advance in mission if no active nav command
	        if (!advance_current_nav_cmd()) {
	            // failure to advance nav command means mission has completed
	            complete();
	            return;
	        }
	    }else{
	        // run the active nav command
	        if (_cmd_verify_fn(_nav_cmd)) {
	            // market _nav_cmd as complete (it will be started on the next iteration)
	            _flags.nav_cmd_loaded = false;
	            // immediately advance to the next mission command
	            if (!advance_current_nav_cmd()) {
	                // failure to advance nav command means mission has completed
	                complete();
	                return;
	            }
	        }
	    }

	    // check if we have an active do command
	    if (!_flags.do_cmd_loaded || _do_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
	        advance_current_do_cmd();
	    }else{
	        // run the active do command
	        if (_cmd_verify_fn(_do_cmd)) {
	            // market _nav_cmd as complete (it will be started on the next iteration)
	            _flags.do_cmd_loaded = false;
	        }
	    }

		break;
	}
	case MISSION_TYPE_CABLECAM:
	{
	    // exit immediately if not running or no mission commands
	    if (_flags.state != MISSION_RUNNING || _cable_cam_points_total == 0) {
	        return;
	    }

	    // check if we have an active nav command
	    if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
	        // advance in mission if no active nav command
	        if (!advance_current_CCC_cmd()) {
	            // failure to advance nav command means mission has completed
	            complete();
	            return;
	        }
	    }else{
	        // run the active nav command
	        if (_cmd_verify_fn(_nav_cmd)) {
	            // market _nav_cmd as complete (it will be started on the next iteration)
	            _flags.nav_cmd_loaded = false;
	            // immediately advance to the next mission command
	            if (!advance_current_CCC_cmd()) {
	                return;
	            }
	        }
	    }
		break;
	}
	case MISSION_TYPE_MAPWAYPOINT:
	{
		//exit if not running or no mission cmd
		if (_flags.state != MISSION_RUNNING || _map_waypoint_total == 0) {
			return ;
		}
		//check if the copter takeoff
		if (_pre_auto_takeoff_enabled) {
			if(_cmd_verify_fn(_pre_autotakeoff_cmd)) {
				_pre_auto_takeoff_enabled = false;
				//advance to tht first command
				if(!advance_current_MWP_nav_cmd()) {
					complete();	//have not confirm the condition about mwp
				}
			}
		} else {
			//check if we have active nav cmd
			if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
				// advance in mission if no active nav cmd
				if (!advance_current_MWP_nav_cmd()) {
					//failure to advance nav cmd means mission has completed
					complete();
					return ;
				}
			} else {
				//run the active nav cmd
				if (_cmd_verify_fn(_nav_cmd)) {
					//marked _nav_cmd has complete(it will be started on the next iteration)
					// check if we have an active do command when the nav cmd completed
#if 0
				        _flags.nav_cmd_loaded = false;
				        if (!advance_current_MWP_nav_cmd()) { //if the final point have done
				        	return ;
#elif 1
				        if (!_flags.do_cmd_loaded || _do_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
				        	if(!advance_current_MWP_do_cmd()) {
				        		//if nav cmd and do cmd all complted
				                _flags.nav_cmd_loaded = false;
				               	_flags.do_cmd_all_done = false;
				               	_do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
								//immediately advance to the next mission cmd
								if (!advance_current_MWP_nav_cmd()) { //if the final point have done
									return ;
								}
				        		return ;
				        	}
				        } else {
				        	// run the active do command
				            if (_cmd_verify_fn(_do_cmd)) {
				                // market _do_cmd as complete (it will be started on the next iteration)
				            	_flags.do_cmd_loaded = false;
				                if (!advance_current_MWP_do_cmd()) {
				                	return ;
				                }
				            }

#endif
				        }

				    }
				}
			}

		break;
	}
	default:
		break;
	}
}


/// get_next_nav_cmd - gets next "navigation" command found at or after start_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd)
{
    uint16_t cmd_index = start_index;

	// search until the end of the mission command list
	while(cmd_index < (unsigned)_cmd_total) {
		// get next command
		if (!get_next_cmd(cmd_index, cmd, false)) {
			// no more commands so return failure
			return false;
		}else{
			// if found a "navigation" command then return it
			if (is_nav_cmd(cmd)) {
				return true;
			}else{
				// move on in list
				cmd_index++;
			}
		}
	}

    // if we got this far we did not find a navigation command
    return false;
}

/// get the ground course of the next navigation leg in centidegrees
/// from 0 36000. Return default_angle if next navigation
/// leg cannot be determined
int32_t AP_Mission::get_next_ground_course_cd(int32_t default_angle)
{
    Mission_Command cmd;

	switch (_mission_type) {
	case MISSION_TYPE_MAVLINK:
	{
	    if (!get_next_nav_cmd(_nav_cmd.index+1, cmd)) {
	        return default_angle;
	    }
		break;
	}
	case MISSION_TYPE_CABLECAM:
	{
	    if (!get_next_CCC_cmd(_nav_cmd.index, cmd)) {
	        return default_angle;
	    }
	}
	case MISSION_TYPE_MAPWAYPOINT:
	{
		break;
	}
	default:
		break;
	}

    return get_bearing_cd(_nav_cmd.content.location, cmd.content.location);
}

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AP_Mission::read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const
{
	switch (_mission_type) {
	case MISSION_TYPE_MAVLINK:
	{
	    // exit immediately if index is beyond last command but we always let cmd #0 (i.e. home) be read
	    if (index > (unsigned)_cmd_total && index != 0) {
	        return false;
	    }

	    // special handling for command #0 which is home
	    if (index == 0) {
	        cmd.index = 0;
	        cmd.id = MAV_CMD_NAV_WAYPOINT;
	        cmd.p1 = 0;
	        cmd.content.location = _ahrs.get_home();
	    }else{
	        // Find out proper location in memory by using the start_byte position + the index
	        // we can load a command, we don't process it yet
	        // read WP position
	        uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

	        cmd.id = _storage.read_byte(pos_in_storage);
	        cmd.p1 = _storage.read_uint16(pos_in_storage+1);
	        _storage.read_block(cmd.content.bytes, pos_in_storage+3, 12);

	        // set command's index to it's position in eeprom
	        cmd.index = index;
	    }

	    // return success
	    return true;
		break;
	}
	case MISSION_TYPE_CABLECAM:
	{
		// exit immediately if cable-cam not initiated
		if (!_cable_cam_initiated) {
			return false;
		}

		// exit immediately if cable-cam not setted
		if (_cable_cam_points_total == 0) {
			return false;
		}

	    // exit immediately if index is beyond last command but we always let cmd #0 (i.e. home) be read
	    if (index > _cable_cam_points_total && index != 0) {
	        return false;
	    }

	    // special handling for command #0 which is start point
	    if (index == 0) {
	        cmd.index = 0;
	        cmd.id = MAV_CMD_NAV_SPLINE_WAYPOINT;
	        cmd.p1 = 0;
	        cmd.content.location = _cable_cam_point[0].location;
	    } else {
	        cmd.index = index;
			cmd.id = MAV_CMD_NAV_SPLINE_WAYPOINT;
			cmd.p1 = 0;
			cmd.content.location = _cable_cam_point[index-1].location;
	    }

	    // return success
	    return true;
		break;
	}
	case MISSION_TYPE_MAPWAYPOINT:
	{
		if (!_map_waypoint_initiated) {
			return false;
		}

		if (_map_waypoint_total == 0) {
			return false;
		}


		if (index > (_map_waypoint_total + 1) && index != 0) {
			return false;
		}
		// check what type of ariline,curve or straight
		switch (_map_waypoint_cfg.airline_type) {
			case AIRLINE_TYPE_CURVE:
				cmd.id = MAV_CMD_NAV_SPLINE_WAYPOINT;
				break;
			case AIRLINE_TYPE_STRAIGHT:
				cmd.id = MAV_CMD_NAV_WAYPOINT;
				break;
			default:break;
		}
		//special handling for command #0 which is start point
		if (index == 0) {
			cmd.index = 0;
			cmd.p1 = 0;				//store by loiter time
			cmd.content.location.lat = _start_map_waypoint_point.lat;
			cmd.content.location.alt = _start_map_waypoint_point.alt;
			cmd.content.location.lng = _start_map_waypoint_point.lon;
		} else if (index == _map_waypoint_total) {		//the last point ,check if the cmd be landint cmd(judge all the parameter be zero)
			cmd.index = index;
			cmd.p1 = _map_waypoint_point[index-1].waitTime;
			cmd.content.location.lat = _map_waypoint_point[index-1].lat;
			cmd.content.location.alt = _map_waypoint_point[index-1].alt;
			cmd.content.location.lng = _map_waypoint_point[index-1].lon;
			if (cmd.content.location.lat == 0 && cmd.content.location.alt == 0 && cmd.content.location.lat == 0) {
				cmd.id = MAV_CMD_NAV_LAND;
			}
		} else if (index == (_map_waypoint_total + 1) && (cmd.id != MAV_CMD_NAV_LAND)) {
			cmd.index = index;
			cmd.id = MAV_CMD_CONDITION_YAW;
			cmd.p1 = 1;
			cmd.content.yaw.angle_deg = wrap_360_cd(_home_bearing+18000)*0.01f; // target angle in degrees (0=north, 90=east) 0 for look at head
			cmd.content.yaw.turn_rate_dps = 0;// turn rate in degrees / second (0=use default)
			cmd.content.yaw.direction = 1; // -1 = ccw, +1 = cw
			cmd.content.yaw.relative_angle = 0;// 0 = absolute angle, 1 = relative angle
			DEBUG_UARTG_PRINTF1("look at head\n");
		} else {
			cmd.index = index;
			cmd.p1 = _map_waypoint_point[index-1].waitTime ;
			cmd.content.location.lat = _map_waypoint_point[index-1].lat;
			cmd.content.location.alt = _map_waypoint_point[index-1].alt;
			cmd.content.location.lng = _map_waypoint_point[index-1].lon;
		}
		return true;
		break;
	}
	default:
		break;
	}

	return false;
}

bool AP_Mission::read_do_cmd_from_storage(uint16_t nav_index,uint16_t do_index, Mission_Command& cmd) const
{
	DEBUG_UARTG_PRINTF1("read do cmd\n");
	switch (_mission_type) {
	case MISSION_TYPE_MAVLINK:
		break;
	case MISSION_TYPE_CABLECAM:
		break;
	case MISSION_TYPE_MAPWAYPOINT:
	{
		if (!_map_waypoint_initiated) {
			return false;
		}

		if (_map_waypoint_total == 0) {
			return false;
		}

		if (nav_index > _map_waypoint_total && nav_index != 0) {
			return false;
		}

		// check what type of ariline,curve or straight
		switch (_map_waypoint_cfg.airline_type) {
			case AIRLINE_TYPE_CURVE:
				cmd.id = MAV_CMD_NAV_SPLINE_WAYPOINT;
				break;
			case AIRLINE_TYPE_STRAIGHT:
				cmd.id = MAV_CMD_NAV_WAYPOINT;
				break;
			default:break;
		}
		cmd.index = do_index;
		switch(_map_waypoint_point[nav_index-1].do_cmd[do_index].doCmdType) {
			case	DoCmdType_Wait:
				cmd.id = MAV_CMD_NAV_LOITER_TIME;
				cmd.p1 = _map_waypoint_point[nav_index - 1].do_cmd[do_index].Content.delaytime.time/10;
				cmd.content.gimbalParam.GimbalParam_yaw_angle = 0;
				cmd.content.gimbalParam.GimbalParam_pitch_angle = 0;
				break;
			case 	DoCmdType_Camera:
				cmd.id = MAV_CMD_DO_DIGICAM_CONTROL;
				cmd.p1 = _map_waypoint_point[nav_index - 1].do_cmd[do_index].Content.camera.ShootTime;
				cmd.content.cameracontrol.nav_index = nav_index-1;
				cmd.content.cameracontrol.do_index = do_index;
				cmd.content.cameracontrol.photo = _map_waypoint_point[nav_index - 1].do_cmd[do_index].Content.camera.Photo;
				cmd.content.cameracontrol.shooting = _map_waypoint_point[nav_index - 1].do_cmd[do_index].Content.camera.Shooting;
				cmd.content.cameracontrol.shootTime = _map_waypoint_point[nav_index - 1].do_cmd[do_index].Content.camera.ShootTime;
				break;
			case	DoCmdType_PanRot360:
				cmd.id = MAV_CMD_NAV_GIMBAL_CONTROL;
				cmd.p1 = 15;
				cmd.content.gimbalParam.GimbalParam_yaw_angle = (uint16_t)labs(copter.gimCam.get_ef_yaw_angle());
				cmd.content.gimbalParam.GimbalParam_pitch_angle = (uint16_t)labs(copter.gimCam.get_pitch_angle());

			//	cmd.content.gimbalParam.GimbalParam_yaw_angle = _map_waypoint_point[nav_cmd_index - 1].pan_angle;
			//	cmd.content.gimbalParam.GimbalParam_pitch_angle = _map_waypoint_point[nav_cmd_index - 1].tilt_angle;
				break;
			case	DoCmdType_FullView:
				break;
			case	DOCmdType_GimbalSet:
				cmd.id = MAV_CMD_NAV_GIMBAL_CONTROL;
				cmd.p1 = 5;
				cmd.content.gimbalParam.GimbalParam_yaw_angle = _map_waypoint_point[nav_index - 1].do_cmd[do_index].Content.gimbalset.GimbalParam_yaw_angle;
				cmd.content.gimbalParam.GimbalParam_pitch_angle = _map_waypoint_point[nav_index -1].do_cmd[do_index].Content.gimbalset.GimbalParam_pitch_angle;
				break;
			default:
				break;
		}
		return true;
		break;
	}
	default:
		break;
	}

	return false;
}

///
/// MAVLINK mission type command methods
///

/// num_commands_max - returns maximum number of commands that can be stored
uint16_t AP_Mission::num_commands_max(void) const
{
    // -4 to remove space for eeprom version number
    return (_storage.size() - 4) / AP_MISSION_EEPROM_COMMAND_SIZE;
}

/// clear - clears out mission
///     returns true if mission was running so it could not be cleared
bool AP_Mission::clear()
{
    // do not allow clearing the mission while it is running
    if (_flags.state == MISSION_RUNNING) {
        return false;
    }

    // remove all commands
	_cmd_total.set_and_save(0);

    // clear index to commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;

    // return success
    return true;
}


/// trucate - truncate any mission items beyond index
void AP_Mission::truncate(uint16_t index)
{
	if ((unsigned)_cmd_total > index) {
		_cmd_total.set_and_save(index);
	}
}

/// add_cmd - adds a command to the end of the command list and writes to storage
///     returns true if successfully added, false on failure
///     cmd.index is updated with it's new position in the mission
bool AP_Mission::add_cmd(Mission_Command& cmd)
{
    // attempt to write the command to storage
    bool ret = write_cmd_to_storage(_cmd_total, cmd);

    if (ret) {
        // update command's index
        cmd.index = _cmd_total;
        // increment total number of commands
        _cmd_total.set_and_save(_cmd_total + 1);
    }

    return ret;
}

/// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
///     replacing the current active command will have no effect until the command is restarted
///     returns true if successfully replaced, false on failure
bool AP_Mission::replace_cmd(uint16_t index, Mission_Command& cmd)
{
    // sanity check index
    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // attempt to write the command to storage
    return write_cmd_to_storage(index, cmd);
}

/// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
bool AP_Mission::is_nav_cmd(const Mission_Command& cmd)
{
    return (cmd.id <= MAV_CMD_NAV_LAST);
}

// set_current_cmd - jumps to command specified by index
bool AP_Mission::set_current_cmd(uint16_t index)
{
    Mission_Command cmd;

    // sanity check index and that we have a mission
    if (index >= (unsigned)_cmd_total || _cmd_total == 1) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;

    // stop current nav cmd
    _flags.nav_cmd_loaded = false;

    // if index is zero then the user wants to completely restart the mission
    if (index == 0 || _flags.state == MISSION_COMPLETE) {
        _prev_nav_cmd_index = AP_MISSION_CMD_INDEX_NONE;
        // reset the jump tracking to zero
        init_jump_tracking();
        if (index == 0) {
            index = 1;
        }
    }

    // if the mission is stopped or completed move the nav_cmd index to the specified point and set the state to stopped
    // so that if the user resumes the mission it will begin at the specified index
    if (_flags.state != MISSION_RUNNING) {
        // search until we find next nav command or reach end of command list
        while (!_flags.nav_cmd_loaded) {
            // get next command
            if (!get_next_cmd(index, cmd, true)) {
                _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
                return false;
            }

            // check if navigation or "do" command
            if (is_nav_cmd(cmd)) {
                // set current navigation command
                _nav_cmd = cmd;
                _flags.nav_cmd_loaded = true;
            }else{
                // set current do command
                if (!_flags.do_cmd_loaded) {
                    _do_cmd = cmd;
                    _flags.do_cmd_loaded = true;
                }
            }
            // move onto next command
            index = cmd.index+1;
        }

        // if we got this far then the mission can safely be "resumed" from the specified index so we set the state to "stopped"
        _flags.state = MISSION_STOPPED;
        return true;
    }

    // the state must be MISSION_RUNNING
    // search until we find next nav command or reach end of command list
    while (!_flags.nav_cmd_loaded) {
        // get next command
        if (!get_next_cmd(index, cmd, true)) {
            // if we run out of nav commands mark mission as complete
            complete();
            // return true because we did what was requested
            // which was apparently to jump to a command at the end of the mission
            return true;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // save previous nav command index
            _prev_nav_cmd_index = _nav_cmd.index;
            // set current navigation command and start it
            _nav_cmd = cmd;
            _flags.nav_cmd_loaded = true;
            _cmd_start_fn(_nav_cmd);
        }else{
            // set current do command and start it (if not already set)
            if (!_flags.do_cmd_loaded) {
                _do_cmd = cmd;
                _flags.do_cmd_loaded = true;
                _cmd_start_fn(_do_cmd);
            }
        }
        // move onto next command
        index = cmd.index+1;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
bool AP_Mission::write_cmd_to_storage(uint16_t index, Mission_Command& cmd)
{
	// range check cmd's index
	if (index >= num_commands_max()) {
		return false;
	}

	// calculate where in storage the command should be placed
	uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

	_storage.write_byte(pos_in_storage, cmd.id);
	_storage.write_uint16(pos_in_storage+1, cmd.p1);
	_storage.write_block(pos_in_storage+3, cmd.content.bytes, 12);

	// remember when the mission last changed
	_last_change_time_ms = hal.scheduler->millis();

	// return success
	return true;
}

/// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
///     home is taken directly from ahrs
void AP_Mission::write_home_to_storage()
{
    Mission_Command home_cmd;
    home_cmd.id = MAV_CMD_NAV_WAYPOINT;
    home_cmd.content.location = _ahrs.get_home();
    write_cmd_to_storage(0,home_cmd);
}

// mavlink_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
//  return true on success, false on failure
bool AP_Mission::mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd)
{
    bool copy_location = false;
    bool copy_alt = false;
    uint8_t num_turns, radius_m; // used by MAV_CMD_NAV_LOITER_TURNS & _TO_ALT
    uint8_t heading_req;         // used by MAV_CMD_NAV_LOITER_TO_ALT

    // command's position in mission list and mavlink id
    cmd.index = packet.seq;
    cmd.id = packet.command;
    cmd.content.location.options = 0;

    // command specific conversions from mavlink packet to mission command
    switch (cmd.id) {

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
        copy_location = true;
        /*
          the 15 byte limit means we can't fit both delay and radius
          in the cmd structure. When we expand the mission structure
          we can do this properly
         */
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters
        cmd.p1 = packet.param2;
#else
        // delay at waypoint in seconds
        cmd.p1 = packet.param1;                         
#endif
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        copy_location = true;
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);    // -1 = counter clockwise, +1 = clockwise
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        copy_location = true;
        num_turns = packet.param1;                      // number of times to circle is held in param1
        radius_m = fabsf(packet.param3);                // radius in meters is held in high in param3
        cmd.p1 = (((uint16_t)radius_m)<<8) | (uint16_t)num_turns;   // store radius in high byte of p1, num turns in low byte of p1
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        copy_location = true;
        cmd.p1 = packet.param1;                         // loiter time in seconds
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        copy_location = true;
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        copy_location = true;
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        copy_location = true;                           // only altitude is used
        cmd.p1 = packet.param1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        copy_location = true;                           // only using alt
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        copy_location = true;

        heading_req = packet.param1;                    //heading towards next waypoint required (0 = False)
                      
        cmd.content.location.flags.loiter_ccw = (packet.param2 < 0);
        //Don't give users the impression I can set the radius size.
        //I can only set the direction at this time and so can every 
        //other command, despite what is implied (I'm looking at YOU
        //NAV_LOITER_TURNS):
        radius_m = 1; 
        
        cmd.p1 = (((uint16_t)radius_m)<<8) | (uint16_t)heading_req; // store "radius" in high byte of p1, heading_req in low byte of p1
        
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        copy_location = true;
        cmd.p1 = packet.param1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        cmd.p1 = packet.param1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        cmd.content.delay.seconds = packet.param1;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:                  // MAV ID: 113
        copy_alt = true;                                // only altitude is used
        cmd.content.location.lat = packet.param1 * 100; // climb/descent converted from m/s to cm/s.  To-Do: store in proper climb_rate structure
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        cmd.content.distance.meters = packet.param1;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        cmd.content.yaw.angle_deg = packet.param1;      // target angle in degrees
        cmd.content.yaw.turn_rate_dps = packet.param2;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        cmd.content.yaw.direction = packet.param3;      // -1 = ccw, +1 = cw
        cmd.content.yaw.relative_angle = packet.param4; // lng=0: absolute angle provided, lng=1: relative angle provided
        break;

    case MAV_CMD_DO_SET_MODE:                           // MAV ID: 176
        cmd.p1 = packet.param1;                         // flight mode identifier
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        cmd.content.jump.target = packet.param1;        // jump-to command number
        cmd.content.jump.num_times = packet.param2;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        cmd.content.speed.speed_type = packet.param1;   // 0 = airspeed, 1 = ground speed
        cmd.content.speed.target_ms = packet.param2;    // target speed in m/s
        cmd.content.speed.throttle_pct = packet.param3; // throttle as a percentage from 0 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:
        copy_location = true;
        cmd.p1 = packet.param1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_PARAMETER:                      // MAV ID: 180
        cmd.p1 = packet.param1;                         // parameter number
        cmd.content.location.alt = packet.param2;       // parameter value
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        cmd.content.relay.num = packet.param1;          // relay number
        cmd.content.relay.state = packet.param2;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        cmd.content.repeat_relay.num = packet.param1;           // relay number
        cmd.content.repeat_relay.repeat_count = packet.param2;  // count
        cmd.content.repeat_relay.cycle_time = packet.param3;    // time converted from seconds to milliseconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        cmd.content.servo.channel = packet.param1;      // channel
        cmd.content.servo.pwm = packet.param2;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        cmd.content.repeat_servo.channel = packet.param1;      // channel
        cmd.content.repeat_servo.pwm = packet.param2;          // PWM
        cmd.content.repeat_servo.repeat_count = packet.param3; // count
        cmd.content.repeat_servo.cycle_time = packet.param4;   // time in seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        copy_location = true;
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        copy_location = true;
        cmd.p1 = packet.param1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        cmd.content.digicam_configure.shooting_mode = packet.param1;
        cmd.content.digicam_configure.shutter_speed = packet.param2;
        cmd.content.digicam_configure.aperture = packet.param3;
        cmd.content.digicam_configure.ISO = packet.param4;
        cmd.content.digicam_configure.exposure_type = packet.x;
        cmd.content.digicam_configure.cmd_id = packet.y;
        cmd.content.digicam_configure.engine_cutoff_time = packet.z;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        cmd.content.digicam_control.session = packet.param1;
        cmd.content.digicam_control.zoom_pos = packet.param2;
        cmd.content.digicam_control.zoom_step = packet.param3;
        cmd.content.digicam_control.focus_lock = packet.param4;
        cmd.content.digicam_control.shooting_cmd = packet.x;
        cmd.content.digicam_control.cmd_id = packet.y;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        cmd.content.mount_control.pitch = packet.param1;
        cmd.content.mount_control.roll = packet.param2;
        cmd.content.mount_control.yaw = packet.param3;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        cmd.content.cam_trigg_dist.meters = packet.param1;  // distance between camera shots in meters
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        cmd.p1 = packet.param1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                         // MAV ID: 208
        cmd.p1 = packet.param1;                        // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        cmd.p1 = packet.param1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        cmd.content.gripper.num = packet.param1;        // gripper number
        cmd.content.gripper.action = packet.param2;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        cmd.p1 = packet.param1;                         // max time in seconds the external controller will be allowed to control the vehicle
        cmd.content.guided_limits.alt_min = packet.param2;  // min alt below which the command will be aborted.  0 for no lower alt limit
        cmd.content.guided_limits.alt_max = packet.param3;  // max alt above which the command will be aborted.  0 for no upper alt limit
        cmd.content.guided_limits.horiz_max = packet.param4;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit

    case MAV_CMD_DO_AUTOTUNE_ENABLE:                    // MAV ID: 211
        cmd.p1 = packet.param1;                         // disable=0 enable=1
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        cmd.content.altitude_wait.altitude = packet.param1;
        cmd.content.altitude_wait.descent_rate = packet.param2;
        cmd.content.altitude_wait.wiggle_time = packet.param3;
        break;

    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (copy_location || copy_alt) {
        switch (packet.frame) {

        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f * packet.x;   // floating point latitude to int32_t
                cmd.content.location.lng = 1.0e7f * packet.y;   // floating point longitude to int32_t
            }
            cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)
            cmd.content.location.flags.relative_alt = 0;
            break;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f * packet.x;   // floating point latitude to int32_t
                cmd.content.location.lng = 1.0e7f * packet.y;   // floating point longitude to int32_t
            }
            cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)
            cmd.content.location.flags.relative_alt = 1;
            break;

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + _ahrs.get_home().lat;
                cmd.content.location.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + _ahrs.get_home().lng;
            }
            cmd.content.location.alt = -packet.z*1.0e2f;
            cmd.content.location.flags.relative_alt = 1;
            break;
#endif

#ifdef MAV_FRAME_LOCAL
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + _ahrs.get_home().lat;
                cmd.content.location.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + _ahrs.get_home().lng;
            }
            cmd.content.location.alt = packet.z*1.0e2f;
            cmd.content.location.flags.relative_alt = 1;
            break;
#endif

#if AP_TERRAIN_AVAILABLE
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f * packet.x;   // floating point latitude to int32_t
                cmd.content.location.lng = 1.0e7f * packet.y;   // floating point longitude to int32_t
            }
            cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            cmd.content.location.flags.relative_alt = 1;
            // mark altitude as above terrain, not above home
            cmd.content.location.flags.terrain_alt = 1;
            break;
#endif

        default:
            return false;
        }
    }

    // if we got this far then it must have been succesful
    return true;
}

// mission_cmd_to_mavlink - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
//  return true on success, false on failure
bool AP_Mission::mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet)
{
    bool copy_location = false;
    bool copy_alt = false;

    // command's position in mission list and mavlink id
    packet.seq = cmd.index;
    packet.command = cmd.id;

    // set defaults
    packet.current = 0;     // 1 if we are passing back the mission command that is currently being executed
    packet.param1 = 0;
    packet.param2 = 0;
    packet.param3 = 0;
    packet.param4 = 0;
    packet.autocontinue = 1;

    // command specific conversions from mission command to mavlink packet
    switch (cmd.id) {

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
        copy_location = true;
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters
        packet.param2 = cmd.p1;
#else
        // delay at waypoint in seconds
        packet.param1 = cmd.p1;
#endif
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        copy_location = true;
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -1;
        }else{
            packet.param3 = 1;
        }
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        copy_location = true;
        packet.param1 = LOWBYTE(cmd.p1);                // number of times to circle is held in low byte of p1
        packet.param3 = HIGHBYTE(cmd.p1);               // radius is held in high byte of p1
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -packet.param3;
        }
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        copy_location = true;
        packet.param1 = cmd.p1;                         // loiter time in seconds
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -1;
        }else{
            packet.param3 = 1;
        }
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        copy_location = true;
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        copy_location = true;
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        copy_location = true;                           // only altitude is used
        packet.param1 = cmd.p1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        copy_location = true;                           //only using alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        copy_location = true;
        packet.param1 = LOWBYTE(cmd.p1);                //heading towards next waypoint required (0 = False) 
        packet.param2 = HIGHBYTE(cmd.p1);               //loiter radius(m)
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param2 = -packet.param2;
        }

        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        copy_location = true;
        packet.param1 = cmd.p1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        packet.param1 = cmd.p1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        packet.param1 = cmd.content.delay.seconds;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:                  // MAV ID: 113
        copy_alt = true;                                // only altitude is used
        packet.param1 = cmd.content.location.lat / 100.0f;  // climb/descent rate converted from cm/s to m/s.  To-Do: store in proper climb_rate structure
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        packet.param1 = cmd.content.distance.meters;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        packet.param1 = cmd.content.yaw.angle_deg;      // target angle in degrees
        packet.param2 = cmd.content.yaw.turn_rate_dps;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        packet.param3 = cmd.content.yaw.direction;      // -1 = ccw, +1 = cw
        packet.param4 = cmd.content.yaw.relative_angle; // 0 = absolute angle provided, 1 = relative angle provided
        break;

    case MAV_CMD_DO_SET_MODE:                           // MAV ID: 176
        packet.param1 = cmd.p1;                         // set flight mode identifier
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        packet.param1 = cmd.content.jump.target;        // jump-to command number
        packet.param2 = cmd.content.jump.num_times;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        packet.param1 = cmd.content.speed.speed_type;   // 0 = airspeed, 1 = ground speed
        packet.param2 = cmd.content.speed.target_ms;    // speed in m/s
        packet.param3 = cmd.content.speed.throttle_pct; // throttle as a percentage from 0 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:                           // MAV ID: 179
        copy_location = true;
        packet.param1 = cmd.p1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_PARAMETER:                      // MAV ID: 180
        packet.param1 = cmd.p1;                         // parameter number
        packet.param2 = cmd.content.location.alt;       // parameter value
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        packet.param1 = cmd.content.relay.num;          // relay number
        packet.param2 = cmd.content.relay.state;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        packet.param1 = cmd.content.repeat_relay.num;           // relay number
        packet.param2 = cmd.content.repeat_relay.repeat_count;  // count
        packet.param3 = cmd.content.repeat_relay.cycle_time;    // time in seconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        packet.param1 = cmd.content.servo.channel;      // channel
        packet.param2 = cmd.content.servo.pwm;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        packet.param1 = cmd.content.repeat_servo.channel;       // channel
        packet.param2 = cmd.content.repeat_servo.pwm;           // PWM
        packet.param3 = cmd.content.repeat_servo.repeat_count;  // count
        packet.param4 = cmd.content.repeat_servo.cycle_time;    // time in milliseconds converted to seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        copy_location = true;
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        copy_location = true;
        packet.param1 = cmd.p1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        packet.param1 = cmd.content.digicam_configure.shooting_mode;
        packet.param2 = cmd.content.digicam_configure.shutter_speed;
        packet.param3 = cmd.content.digicam_configure.aperture;
        packet.param4 = cmd.content.digicam_configure.ISO;
        packet.x = cmd.content.digicam_configure.exposure_type;
        packet.y = cmd.content.digicam_configure.cmd_id;
        packet.z = cmd.content.digicam_configure.engine_cutoff_time;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        packet.param1 = cmd.content.digicam_control.session;
        packet.param2 = cmd.content.digicam_control.zoom_pos;
        packet.param3 = cmd.content.digicam_control.zoom_step;
        packet.param4 = cmd.content.digicam_control.focus_lock;
        packet.x = cmd.content.digicam_control.shooting_cmd;
        packet.y = cmd.content.digicam_control.cmd_id;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        packet.param1 = cmd.content.mount_control.pitch;
        packet.param2 = cmd.content.mount_control.roll;
        packet.param3 = cmd.content.mount_control.yaw;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        packet.param1 = cmd.content.cam_trigg_dist.meters;  // distance between camera shots in meters
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                          // MAV ID: 208
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        packet.param1 = cmd.p1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        packet.param1 = cmd.content.gripper.num;        // gripper number
        packet.param2 = cmd.content.gripper.action;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        packet.param1 = cmd.p1;                         // max time in seconds the external controller will be allowed to control the vehicle
        packet.param2 = cmd.content.guided_limits.alt_min;  // min alt below which the command will be aborted.  0 for no lower alt limit
        packet.param3 = cmd.content.guided_limits.alt_max;  // max alt above which the command will be aborted.  0 for no upper alt limit
        packet.param4 = cmd.content.guided_limits.horiz_max;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        packet.param1 = cmd.p1;                         // disable=0 enable=1
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        packet.param1 = cmd.content.altitude_wait.altitude;
        packet.param2 = cmd.content.altitude_wait.descent_rate;
        packet.param3 = cmd.content.altitude_wait.wiggle_time;
        break;

    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (copy_location) {
        packet.x = cmd.content.location.lat / 1.0e7f;   // int32_t latitude to float
        packet.y = cmd.content.location.lng / 1.0e7f;   // int32_t longitude to float
    }
    if (copy_location || copy_alt) {
        packet.z = cmd.content.location.alt / 100.0f;   // cmd alt in cm to m
        if (cmd.content.location.flags.relative_alt) {
            packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        }else{
            packet.frame = MAV_FRAME_GLOBAL;
        }
#if AP_TERRAIN_AVAILABLE
        if (cmd.content.location.flags.terrain_alt) {
            // this is a above-terrain altitude
            if (!cmd.content.location.flags.relative_alt) {
                // refuse to return non-relative terrain mission
                // items. Internally we do have these, and they
                // have home.alt added, but we should never be
                // returning them to the GCS, as the GCS doesn't know
                // our home.alt, so it would have no way to properly
                // interpret it
                return false;
            }
            packet.z = cmd.content.location.alt * 0.01f;
            packet.frame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
        }
#else
        // don't ever return terrain mission items if no terrain support
        if (cmd.content.location.flags.terrain_alt) {
            return false;
        }
#endif
    }

    // if we got this far then it must have been successful
    return true;
}

// find the nearest landing sequence starting point (DO_LAND_START) and
// return its index.  Returns 0 if no appropriate DO_LAND_START point can
// be found.
uint16_t AP_Mission::get_landing_sequence_start()
{
    struct Location current_loc;

    if (!_ahrs.get_position(current_loc)) {
        return 0;
    }

    uint16_t landing_start_index = 0;
    float min_distance = -1;

    // Go through mission looking for nearest landing start command
    for (uint16_t i = 0; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (tmp.id == MAV_CMD_DO_LAND_START) {
            float tmp_distance = get_distance(tmp.content.location, current_loc);
            if (min_distance < 0 || tmp_distance < min_distance) {
                min_distance = tmp_distance;
                landing_start_index = i;
            }
        }
    }

    return landing_start_index;
}

///
/// CURVE-CABLE-CAM mission type command methods
///

// reset cable cam
bool AP_Mission::init_cable_cam()
{
    // do not allow clearing the mission while it is running
    if (_flags.state == MISSION_RUNNING) {
        return false;
    }

    // set mission type to cable cam
    if (!set_mission_type(MISSION_TYPE_CABLECAM)) {
    	return false;
	}

    // remove all commands
	_cable_cam_points_total = 0;

    // clear index to commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;

    // reset cable cam variables
    _one_direction_finished = false;
    _cable_cam_direction = MISSION_DIRECTION_FORWARD;

    // return success
    return true;
}

// return true if set cable cam direction successfully
bool AP_Mission::set_cable_cam_direction(enum mission_direction direction)
{
    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // exit immediately if current nav index is out of range
    uint16_t cmd_index = get_current_nav_index();
    if (cmd_index < 1 || cmd_index > _cable_cam_points_total) {
    	return false;
    }

    // change direction and start as a command
	if (_cable_cam_direction != direction) {
	    Mission_Command cmd;

	    // calculate the next command index according to the current direction
		switch (_cable_cam_direction) {
		case MISSION_DIRECTION_FORWARD:
			cmd_index -= 1;
			break;
		case MISSION_DIRECTION_BACKWARD:
			cmd_index += 1;
			break;
		}

		// read command
		 if (!read_cmd_from_storage(cmd_index, cmd)) {
			return false;
		}

		// now update the cable cam direction and reset complete flag for this direction
		_cable_cam_direction = direction;
		_one_direction_finished = false;

	     // save previous nav command index
		 _prev_nav_cmd_index = _nav_cmd.index;
		 // set current navigation command and start it
		 _nav_cmd = cmd;
		 _flags.nav_cmd_loaded = true;
		 _cmd_start_fn(_nav_cmd);

	    // if we got this far we must have successfully advanced the nav command
	    return true;
	}

	return false;
}

// add cable-cam point
// index will not allow be set by user because it's more safe to manager by itself rather than user
// return true if set successfully
bool AP_Mission::add_cable_cam_point(cableCamPoint &cable_cam_point)
{
	// exit immediately if memory for cable-cam point is not initiated
	if (!_cable_cam_initiated) {
		return false;
	}

    // range check cmd's index
    if (_cable_cam_points_total >= (uint16_t)_max_ccc_points) {
        return false;
    }

    //	memcpy(&_cable_cam_point[_cable_cam_points_total], &cable_cam_point, sizeof(cableCamPoint));
    _cable_cam_point[_cable_cam_points_total] = cable_cam_point;
    _cable_cam_points_total++;

	return true;
}

// delete number cable-cam point at the top of points array
// return number of points that has been deleted
uint16_t AP_Mission::delete_cable_cam_points(uint16_t amount)
{
	uint16_t count = 0;

	// reduce the amount of cable cam points until clear them all
	do {
		if (_cable_cam_points_total == 0)
			break;

		_cable_cam_points_total--;
		count++;
	} while (count < amount);

	return count;
}

/// always set start cable cam point before start cable cam mission
void AP_Mission::set_start_cable_cam_point(cableCamPoint &cable_cam_point)
{
	_start_cable_cam_point = cable_cam_point;
}

/// get_next_CCC_cmd - gets next "navigation" command found after cur_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
bool AP_Mission::get_next_CCC_cmd(uint16_t cur_index, Mission_Command& cmd)
{
    uint16_t cmd_index = cur_index;

	switch (_cable_cam_direction) {
	case MISSION_DIRECTION_FORWARD:
		cmd_index += 1;
		break;
	case MISSION_DIRECTION_BACKWARD:
		cmd_index -= 1;
		break;
	}

	// search until the end of the mission command list
	// _cable_cam_points_total not include home point
	if (cmd_index > 0 && cmd_index <= _cable_cam_points_total) {
		// get next command
		if (!read_cmd_from_storage(cmd_index, cmd)) {
			return false;
		}

		return true;
	}

    // if we got this far we did not find a navigation command
    return false;
}

// return true if get current cable cam point successfully
bool AP_Mission::get_current_cable_cam_point(cableCamPoint& cable_cam_point)
{
	// exit immediately if memory for cable-cam point is not initiated
	if (!_cable_cam_initiated || _cable_cam_points_total == 0) {
		return false;
	}

	uint16_t current_index = get_current_nav_index();

    // search until the end of the mission command list
	// _cable_cam_points_total not include home point
    if (current_index > 0 && current_index <= _cable_cam_points_total) {
    	cable_cam_point = _cable_cam_point[current_index-1];
        return true;
    } else if (current_index == 0) {
    	cable_cam_point = _cable_cam_point[0];
        return true;
    }

    return false;
}

// return true if get next cable cam point successfully
bool AP_Mission::get_prev_cable_cam_point(cableCamPoint& cable_cam_point)
{
	// exit immediately if memory for cable-cam point is not initiated
	if (!_cable_cam_initiated || _cable_cam_points_total == 0) {
		return false;
	}

	uint16_t prev_index = get_current_nav_index();
	switch (_cable_cam_direction) {
	case MISSION_DIRECTION_FORWARD:
		prev_index -= 1;
		break;
	case MISSION_DIRECTION_BACKWARD:
		prev_index += 1;
		break;
	}

    // search until the end of the mission command list
	// _cable_cam_points_total not include home point
    if (prev_index > 0 && prev_index <= _cable_cam_points_total) {
		cable_cam_point = _cable_cam_point[prev_index-1];
        return true;
    } else if (_nav_cmd.index == 0) {
		cable_cam_point = _start_cable_cam_point;
		return true;
    }

    return false;
}


/////////////code add for the remote controls waypoint misssion////////////////
/////


bool AP_Mission::init_map_waypoint(MapWayPointConfig *mw_cfg)
{
	DEBUG_UARTG_PRINTF1("init map waypoint\n");
	// do not allow clearing the mission while it is running
    if (_flags.state == MISSION_RUNNING) {
        return false;
    }

    // set mission type to map_way_point
    if (!set_mission_type(MISSION_TYPE_MAPWAYPOINT)) {
    	return false;
	}

    // remove all commands
	_map_waypoint_total = 0;

	_map_waypoint_cfg.waypointCount = mw_cfg->waypointCount;
	_map_waypoint_cfg.airline_type = mw_cfg->airline_type;
	_map_waypoint_cfg.airline_ending_action = mw_cfg->airline_ending_action;
	_map_waypoint_cfg.altitude_type = mw_cfg->altitude_type;
	_map_waypoint_cfg.airline_default_speed = mw_cfg->airline_default_speed;
	_map_waypoint_cfg.pan_mode = mw_cfg->pan_mode;
	_map_waypoint_cfg.tilt_mode = mw_cfg->tilt_mode;

    // clear index to commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;
	_pre_auto_takeoff_enabled = false;
	_map_airline_ending_cycle_enable = false;
	_map_airline_ending_land_enable = false;
	_yaw_looking = LOOK_AT_HOLD;

	_gimbal_tilt = AP_MISSION_CMD_GIMBAL_ANGLE_NONE;
	_gimbal_pan = AP_MISSION_CMD_GIMBAL_ANGLE_NONE;
    // return success
	return true;
}

bool AP_Mission::add_map_waypoint_point(AP_Mission::MapWaypoint &map_waypoint)
{
	// AP_Mission::MapWaypoint map_waypoint;
	if(!_map_waypoint_initiated)
		return false;

	if(_map_waypoint_total > _max_map_waypoints || _map_waypoint_total > _map_waypoint_cfg.waypointCount)
		return false;
	_map_waypoint_point[_map_waypoint_total] = map_waypoint;

	uint8_t	do_cmd_count = _map_waypoint_point[_map_waypoint_total].doCmdCount;
	//alloc memory for do cmd
	if (do_cmd_count <= _max_map_waypoints_docmd) {
		_map_waypoint_point[_map_waypoint_total].do_cmd = (MapWaypoint_doCmd *)malloc(sizeof(MapWaypoint_doCmd)*do_cmd_count);
	} else {
		return false;
	}

	//alloc memory fail
	if(_map_waypoint_point[_map_waypoint_total].do_cmd == NULL) {
		return false;
	} else {
		_map_waypoint_do_total = 0;
	}
	_map_waypoint_total++;
	return true;
}

bool AP_Mission::add_map_waypoint_do_cmd(MapWaypoint_doCmd &Waypoint_docmd)
{
	// AP_Mission::MapWaypoint map_waypoint;
	if(!_map_waypoint_initiated) {
		return false;
	}

	if(_map_waypoint_total > _max_map_waypoints || _map_waypoint_total > _map_waypoint_cfg.waypointCount) {
		return false;
	}

	if(_map_waypoint_do_total > _map_waypoint_point[_map_waypoint_total-1].doCmdCount) {
		return false;
	}
	_map_waypoint_point[_map_waypoint_total-1].do_cmd[_map_waypoint_do_total] = Waypoint_docmd;

	hal.uartG->printf("do cmd index: %d added,waypoint index p : %d \n",_map_waypoint_do_total,_map_waypoint_total-1);
	_map_waypoint_do_total++;

	return true;
}


bool AP_Mission::delect_map_waypoint_point(uint8_t num)
{
	return true;
}



// return true if get current map waypoint successfully
bool AP_Mission::get_current_map_waypoint(MapWaypoint& map_waypoint)
{
	// exit immediately if memory formap waypoint point is not initiated
	if (!_map_waypoint_initiated || _map_waypoint_total == 0) {
		return false;
	}

	uint16_t current_index = get_current_nav_index();

    // search until the end of the mission command list
	// _map_waypoint_total not include home point
    if (current_index > 0 && current_index <= _map_waypoint_total) {
    	map_waypoint = _map_waypoint_point[current_index-1];
        return true;
    } else if (current_index == 0) {
    	map_waypoint = _map_waypoint_point[0];
        return true;
    }
    return false;
}

// return true if get next map way point successfully
bool AP_Mission::get_prev_map_waypoint(MapWaypoint& mapWayPoint)
{
	// exit immediately if memory for map way point is not initiated
	if (!_map_waypoint_initiated || _map_waypoint_total == 0) {
		return false;
	}

	uint16_t prev_index = get_current_nav_index();

	prev_index -= 1;
    // search until the end of the mission command list
	// _map_way_points_total not include home point
    if (prev_index > 0 && prev_index <= _map_waypoint_total) {
		mapWayPoint = _map_waypoint_point[prev_index-1];
        return true;
    } else if (_nav_cmd.index == 0) {
		mapWayPoint = _start_map_waypoint_point;
		return true;
    }

    return false;
}

/// get_next_mwp_cmd - gets next "navigation" command found after cur_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
bool AP_Mission::get_next_MWP_nav_cmd(uint16_t cur_index, Mission_Command& cmd)
{
    uint16_t cmd_index = cur_index;

	cmd_index += 1;
	if(cmd_index > _map_waypoint_total){
		//if ending cycle enabled , the next nav cmd will be the cmd 1
		if(_map_airline_ending_cycle_enable == true)
		{
			cmd_index = cmd_index%_map_waypoint_total;
		}
	}
	// search until the end of the mission command list
	// _cable_cam_points_total not include home point
	if (cmd_index > 0 && cmd_index <= _map_waypoint_total) {
		// get next command
		if (!read_cmd_from_storage(cmd_index, cmd)) {
			return false;
		}

		return true;
	}

    // if we got this far we did not find a navigation command
    return false;
}

void AP_Mission::set_start_map_waypoint(MapWaypoint &way_point)
{
	_start_map_waypoint_point = way_point;

	//use the speed of the first point as start point speed
	//start_map_waypoint_point.speed = _map_waypoint_point[0].speed;
}

//must called after set_start_map_waypoint
void AP_Mission::set_map_waypoint_auto_takeoff_enable(bool autotakeoff_enabled)
{
	_pre_auto_takeoff_enabled = autotakeoff_enabled;
	if(_pre_auto_takeoff_enabled){
		_start_map_waypoint_point.alt = _auto_takeoff_alt;
	}

	switch(_map_waypoint_cfg.airline_ending_action)
	{
		case AIRLINE_ENDING_RETURN_START:	//return to the start point
			_start_map_waypoint_point.waitTime = 2;
			add_map_waypoint_point(_start_map_waypoint_point); //add start point as the last dest
		break;
		case AIRLINE_ENDING_SUSPEND:		//suspend
			if(_map_waypoint_point[_map_waypoint_total-1].waitTime < 3)
				_map_waypoint_point[_map_waypoint_total-1].waitTime = 3;	//loiter for more than 3s
		break;

		case AIRLINE_ENDING_LANDING:
			if(_map_waypoint_point[_map_waypoint_total-1].waitTime < 3)
				_map_waypoint_point[_map_waypoint_total-1].waitTime = 3;	//loiter for more than 3s
			_map_airline_ending_land_enable = true;
			MapWaypoint	landpoint;
			memset(&landpoint,0,sizeof(MapWaypoint));	//set all the parametere be zero as the landing judge condition
			landpoint.tilt_mode = Tilt_Manual;
			landpoint.speed = 8;
			add_map_waypoint_point(landpoint);
		break;

		case AIRLINE_ENDING_CYCLE:
			_map_airline_ending_cycle_enable = true;
		break;
		default:break;
	}
}

bool AP_Mission::advance_current_MWP_nav_cmd(void)
{
	Mission_Command cmd;
	uint16_t cmd_index;

	// exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {

        return false;
    }

    // get starting point for search
    cmd_index = _nav_cmd.index;
    // increase index
	if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
		// start from beginning of the mission command list
		cmd_index = AP_MISSION_CCC_START_COMMAND;
	} else {
		// start from one position past the current nav command
		cmd_index++;
		// if we already reached the final point, then check what we will do next
		if (cmd_index > _map_waypoint_total) {
			if(_map_airline_ending_cycle_enable == true) {//do nav cmd cycle
				cmd_index = cmd_index%_map_waypoint_total;		//cmd start frome 1 which is the acutal first point
			} else if (_map_airline_ending_land_enable) {
					return false;						//if land enabled , will do complete
				} else if (cmd_index > (_map_waypoint_total + 1)) {
					_flags.nav_cmd_loaded = false;		//else will not complete(),let copter loiter on the end point
					return false;
				}
			}
		}
	if (!read_cmd_from_storage(cmd_index, cmd)) {
		return false;
	}
	if (cmd_index == 0) {
		 _gimbal_tilt = 0;
		 _gimbal_pan = 0;
		 _cur_tilt_mode = _start_map_waypoint_point.tilt_mode;
	} else if (cmd_index > 0 && cmd_index <= _map_waypoint_total) {
		 _gimbal_tilt = _map_waypoint_point[cmd_index-1].tilt_angle;
		 _gimbal_pan = _map_waypoint_point[cmd_index-1].pan_angle;
		 _cur_tilt_mode = _map_waypoint_point[cmd_index-1].tilt_mode;
	}
	DEBUG_UARTG_PRINTF2("pan_cmd_angle:%d\n",_gimbal_pan);
	DEBUG_UARTG_PRINTF2("tilt_cmd_angle:%d\n",_gimbal_tilt);
	//DEBUG_UARTG_PRINTF2("_cur_tilt_mode:%d\n",_cur_tilt_mode);
	//DEBUG_UARTG_PRINTF2("_cur_pan_mode:%d\n",_map_waypoint_cfg.pan_mode);
	if (cmd_index == 1) {
		_gimbal_tilt_pre = 0;
		_gimbal_pan_pre = _map_waypoint_point[0].pan_angle;

	} else if (cmd_index > 1 && cmd_index <= _map_waypoint_total) {
		_gimbal_tilt_pre = _map_waypoint_point[cmd_index - 2].tilt_angle;
		_gimbal_pan_pre = _map_waypoint_point[cmd_index-2].pan_angle;
	}

	// save previous nav command index
	_prev_nav_cmd_index = _nav_cmd.index;
	// set current navigation command and start it
	_nav_cmd = cmd;
	_flags.nav_cmd_loaded = true;
	_cmd_start_fn(_nav_cmd);
    // if we got this far we must have successfully advanced the nav command
    return true;
}


bool AP_Mission::advance_current_MWP_do_cmd(void)
{
    Mission_Command cmd;
	uint16_t nav_cmd_index;
    uint16_t cmd_index;
    uint16_t do_cmd_count;

	//get the current point
	nav_cmd_index = _nav_cmd.index;
	if (nav_cmd_index < 1) {
		return false;
	}

	do_cmd_count = _map_waypoint_point[nav_cmd_index - 1].doCmdCount;
    cmd_index = _do_cmd.index;

    // exit immediately if we're not running or we've completed all possible "do" commands
    if (_flags.state != MISSION_RUNNING || _flags.do_cmd_all_done) {
    	return false;
    }

    // get starting point for search
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_CCC_START_COMMAND;
    }else{
        // start from one position past the current do command
        cmd_index++;
    }

    // check if we've reached end of mission
    if (cmd_index >= (unsigned)do_cmd_count) {
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
        return false;
    }

    // find next do command
    if (get_next_nav_do_cmd(nav_cmd_index,cmd_index, cmd)) {
        // set current do command and start it
        _do_cmd = cmd;
        _flags.do_cmd_loaded = true;
#if 1
        if(_do_cmd.id == MAV_CMD_NAV_LOITER_TIME || _do_cmd.id == MAV_CMD_DO_DIGICAM_CONTROL ) {
			_gimbal_tilt = _map_waypoint_point[nav_cmd_index - 1].tilt_angle;
			_gimbal_pan = _map_waypoint_point[nav_cmd_index - 1].pan_angle;
			_gimbal_tilt_pre = _map_waypoint_point[nav_cmd_index - 1].tilt_angle;
			_gimbal_pan_pre = _map_waypoint_point[nav_cmd_index - 1].pan_angle;
        } else if (_do_cmd.id == MAV_CMD_NAV_GIMBAL_CONTROL) {
        	if (_do_cmd.p1 == 5) {
				_gimbal_tilt = _do_cmd.content.gimbalParam.GimbalParam_pitch_angle;
				_gimbal_pan = _do_cmd.content.gimbalParam.GimbalParam_yaw_angle;
				_gimbal_tilt_pre = _map_waypoint_point[nav_cmd_index - 1].tilt_angle;
				_gimbal_pan_pre = _map_waypoint_point[nav_cmd_index - 1].pan_angle;
				_cur_tilt_mode = Tilt_Custom; //custom
			} else if (_do_cmd.p1 == 10) {
				DEBUG_UARTG_PRINTF1("get rot 360\n");
				_gimbal_tilt = _map_waypoint_point[nav_cmd_index - 1].tilt_angle;
				_gimbal_pan = _map_waypoint_point[nav_cmd_index - 1].pan_angle;
				_gimbal_tilt_pre = _map_waypoint_point[nav_cmd_index - 1].tilt_angle;
				_gimbal_pan_pre = _map_waypoint_point[nav_cmd_index - 1].pan_angle;
				_cur_tilt_mode = Tilt_Custom; //custom
				DEBUG_UARTG_PRINTF2("pan_docmd_angle:%d\n",_gimbal_pan);
				DEBUG_UARTG_PRINTF2("tilt_docmd_angle:%d\n",_gimbal_tilt);
				DEBUG_UARTG_PRINTF2("_cur_tilt_mode:%d\n",_cur_tilt_mode);
				DEBUG_UARTG_PRINTF2("_cur_pan_mode:%d\n",_map_waypoint_cfg.pan_mode);
			}
        }
#endif
        _cmd_start_fn(_do_cmd);
    }else{
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
        return false;
    }
    return true;
}

uint16_t AP_Mission::get_current_mwp_cmd_speed()
{
	if(!_map_waypoint_initiated)
		return 0;
	uint16_t tempspeed;

	tempspeed = _map_waypoint_cfg.airline_default_speed*10;

	if(_nav_cmd.index == 0) {
		tempspeed = _start_map_waypoint_point.speed*10;
	} else {
		tempspeed = _map_waypoint_point[(_nav_cmd.index-1)].speed*10;
	}

	return tempspeed;
}

///
/// private methods
///

/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
void AP_Mission::complete()
{
    // flag mission as complete
    _flags.state = MISSION_COMPLETE;

    // callback to main program's mission complete function
    _mission_complete_fn();
}


/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
void AP_Mission::complete_flag()
{
    // flag mission as complete
    _flags.state = MISSION_COMPLETE;

}

/// advance_current_nav_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_nav_cmd()
{
    Mission_Command cmd;
    uint16_t cmd_index;

    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // exit immediately if current nav command has not completed
    if (_flags.nav_cmd_loaded) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;

    // get starting point for search
    cmd_index = _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        // start from beginning of the mission command list
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }else{
        // start from one position past the current nav command
        cmd_index++;
    }

    // avoid endless loops
    uint8_t max_loops = 255;

    // search until we find next nav command or reach end of command list
    while (!_flags.nav_cmd_loaded) {
        // get next command
        if (!get_next_cmd(cmd_index, cmd, true)) {
            return false;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // save previous nav command index
            _prev_nav_cmd_index = _nav_cmd.index;
            // set current navigation command and start it
            _nav_cmd = cmd;
            _flags.nav_cmd_loaded = true;
            _cmd_start_fn(_nav_cmd);
        }else{
            // set current do command and start it (if not already set)
            if (!_flags.do_cmd_loaded) {
                _do_cmd = cmd;
                _flags.do_cmd_loaded = true;
                _cmd_start_fn(_do_cmd);
            } else {
                // protect against endless loops of do-commands
                if (max_loops-- == 0) {
                    return false;
                }
            }
        }
        // move onto next command
        cmd_index = cmd.index+1;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// advance_current_CCC_cmd - moves current CCC command forward
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_CCC_cmd()
{
    Mission_Command cmd;
    uint16_t cmd_index;

    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // exit immediately if one direction has already been finished
    // stop update command if user not set new direction
    if (_one_direction_finished) {
    	_flags.nav_cmd_loaded = true;
    	return false;
    }

    // get starting point for search
    cmd_index = _nav_cmd.index;
    // increase or decrease index according to direction from user setting
	switch (_cable_cam_direction) {
	case MISSION_DIRECTION_FORWARD:
		if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
			// start from beginning of the mission command list
			cmd_index = AP_MISSION_CCC_START_COMMAND;
		}else{
			// start from one position past the current nav command
			cmd_index++;
			// if we already reached the final point, then set one direction finished
			if (cmd_index > _cable_cam_points_total) {
				_flags.nav_cmd_loaded = true;
				_one_direction_finished = true;
				return false;
			}
		}

		break;
	case MISSION_DIRECTION_BACKWARD:
		// start from one position past the current nav command
		cmd_index--;
		// if we already reached the start point, then set one direction finished
		if (cmd_index < AP_MISSION_FIRST_REAL_COMMAND) {
			_flags.nav_cmd_loaded = true;
			_one_direction_finished = true;
			return false;
		}

		break;
	}

	 if (!read_cmd_from_storage(cmd_index, cmd)) {
		return false;
	}

	// save previous nav command index
	_prev_nav_cmd_index = _nav_cmd.index;
	// set current navigation command and start it
	_nav_cmd = cmd;
	_flags.nav_cmd_loaded = true;
	_cmd_start_fn(_nav_cmd);

    // if we got this far we must have successfully advanced the nav command
    return true;
}


/// advance_current_do_cmd - moves current do command forward
///     accounts for do-jump commands
void AP_Mission::advance_current_do_cmd()
{
    Mission_Command cmd;
    uint16_t cmd_index;

    // exit immediately if we're not running or we've completed all possible "do" commands
    if (_flags.state != MISSION_RUNNING || _flags.do_cmd_all_done) {
        return;
    }

    // get starting point for search
    cmd_index = _do_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }else{
        // start from one position past the current do command
        cmd_index++;
    }

    // check if we've reached end of mission
    if (cmd_index >= (unsigned)_cmd_total) {
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
        return;
    }

    // find next do command
    if (get_next_do_cmd(cmd_index, cmd)) {
        // set current do command and start it
        _do_cmd = cmd;
        _flags.do_cmd_loaded = true;
        _cmd_start_fn(_do_cmd);
    }else{
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
    }
}

/// get_next_cmd - gets next command found at or after start_index
///     returns true if found, false if not found (i.e. mission complete)
///     accounts for do_jump commands
///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
bool AP_Mission::get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found)
{
    uint16_t cmd_index = start_index;
    Mission_Command temp_cmd;
    uint16_t jump_index = AP_MISSION_CMD_INDEX_NONE;

	// search until the end of the mission command list
	uint8_t max_loops = 64;
	while(cmd_index < (unsigned)_cmd_total) {
		// load the next command
		if (!read_cmd_from_storage(cmd_index, temp_cmd)) {
			// this should never happen because of check above but just in case
			return false;
		}

		// check for do-jump command
		if (temp_cmd.id == MAV_CMD_DO_JUMP) {

			if (max_loops-- == 0) {
				return false;
			}

			// check for invalid target
			if ((temp_cmd.content.jump.target >= (unsigned)_cmd_total) || (temp_cmd.content.jump.target == 0)) {
				// To-Do: log an error?
				return false;
			}

			// check for endless loops
			if (!increment_jump_num_times_if_found && jump_index == cmd_index) {
				// we have somehow reached this jump command twice and there is no chance it will complete
				// To-Do: log an error?
				return false;
			}

			// record this command so we can check for endless loops
			if (jump_index == AP_MISSION_CMD_INDEX_NONE) {
				jump_index = cmd_index;
			}

			// check if jump command is 'repeat forever'
			if (temp_cmd.content.jump.num_times == AP_MISSION_JUMP_REPEAT_FOREVER) {
				// continue searching from jump target
				cmd_index = temp_cmd.content.jump.target;
			}else{
				// get number of times jump command has already been run
				int16_t jump_times_run = get_jump_times_run(temp_cmd);
				if (jump_times_run < temp_cmd.content.jump.num_times) {
					// update the record of the number of times run
					if (increment_jump_num_times_if_found) {
						increment_jump_times_run(temp_cmd);
					}
					// continue searching from jump target
					cmd_index = temp_cmd.content.jump.target;
				}else{
					// jump has been run specified number of times so move search to next command in mission
					cmd_index++;
				}
			}
		}else{
			// this is a non-jump command so return it
			cmd = temp_cmd;
			return true;
		}
	}

    return false;
}

bool AP_Mission::get_next_cmd(uint16_t nav_index,uint16_t do_index, Mission_Command& cmd)
{
	uint16_t cmd_index = do_index;
	Mission_Command temp_cmd;
	uint16_t do_cmd_count;
	//get the current point
	do_cmd_count = _map_waypoint_point[nav_index - 1].doCmdCount;

	while(cmd_index < (unsigned)do_cmd_count) {
		// load the next command
		if (!read_do_cmd_from_storage(nav_index,cmd_index, temp_cmd)) {
			// this should never happen because of check above but just in case
			return false;
		} else {
			cmd = temp_cmd;
			return true;
		}

	}

	return false;
}

/// get_next_do_cmd - gets next "do" or "conditional" command after start_index
///     returns true if found, false if not found
///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_do_cmd(uint16_t start_index, Mission_Command& cmd)
{
    Mission_Command temp_cmd;

    // check we have not passed the end of the mission list
    if (start_index >= (unsigned)_cmd_total) {
        return false;
    }

    // get next command
    if (!get_next_cmd(start_index, temp_cmd, false)) {
        // no more commands so return failure
        return false;
    }else if (is_nav_cmd(temp_cmd)) {
        // if it's a "navigation" command then return false because we do not progress past nav commands
        return false;
    }else{
        // this must be a "do" or "conditional" and is not a do-jump command so return it
        cmd = temp_cmd;
        return true;
    }
}

bool AP_Mission::get_next_nav_do_cmd(uint16_t nav_index,uint16_t do_index, Mission_Command& cmd)
{
    Mission_Command temp_cmd;
    uint16_t do_cmd_count;
    uint8_t do_cmd_type;

	//get the current point
	do_cmd_count = _map_waypoint_point[nav_index - 1].doCmdCount;

    // check we have not passed the end of the mission list
    if (do_index >= (unsigned)do_cmd_count) {
    	return false;
    }

//    if (nav_index != _map_waypoint_point[nav_index - 1].do_cmd[do_index].waypointIndex) {
//    	return false;
//    }

    // get next command
    if (!get_next_cmd(nav_index,do_index, temp_cmd)) {
        // no more commands so return failure
        return false;
    } else {// if (is_nav_cmd(temp_cmd)) {
        // if it's a "navigation" command then return false because we do not progress past nav commands
      //  return false;
   // } else {
        // this must be a "do" or "conditional" and is not a do-jump command so return it
        cmd = temp_cmd;
        return true;
    }
}

///
/// jump handling methods
///

// init_jump_tracking - initialise jump_tracking variables
void AP_Mission::init_jump_tracking()
{
    for(uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking[i].index = AP_MISSION_CMD_INDEX_NONE;
        _jump_tracking[i].num_times_run = 0;
    }
}

/// get_jump_times_run - returns number of times the jump command has been run
int16_t AP_Mission::get_jump_times_run(const Mission_Command& cmd)
{
    // exit immediatley if cmd is not a do-jump command or target is invalid
    if ((cmd.id != MAV_CMD_DO_JUMP) || (cmd.content.jump.target >= (unsigned)_cmd_total) || (cmd.content.jump.target == 0)) {
        // To-Do: log an error?
        return AP_MISSION_JUMP_TIMES_MAX;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            return _jump_tracking[i].num_times_run;
        }else if(_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 0;
            return 0;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error?
    return AP_MISSION_JUMP_TIMES_MAX;
}

/// increment_jump_times_run - increments the recorded number of times the jump command has been run
void AP_Mission::increment_jump_times_run(Mission_Command& cmd)
{
    // exit immediately if cmd is not a do-jump command
    if (cmd.id != MAV_CMD_DO_JUMP) {
        // To-Do: log an error?
        return;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            _jump_tracking[i].num_times_run++;
            return;
        }else if(_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 1;
            return;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error
    return;
}

// check_eeprom_version - checks version of missions stored in eeprom matches this library
// command list will be cleared if they do not match
void AP_Mission::check_eeprom_version()
{
    uint32_t eeprom_version = _storage.read_uint32(0);

    // if eeprom version does not match, clear the command list and update the eeprom version
    if (eeprom_version != AP_MISSION_EEPROM_VERSION) {
        if (clear()) {
            _storage.write_uint32(0, AP_MISSION_EEPROM_VERSION);
        }
    }
}
