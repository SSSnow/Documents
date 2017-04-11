// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// this cpp is for yp_radio handle message interface, like GCS_Mavlink.cpp

#include "Copter.h"

#define DEBUG_PRINTFLN(x)	//hal.console->printf_ln(x)
#define DEBUG_PRINTF(x)    //hal.console->printf(x)

#define DEBUG_UARTG_PRINTF1(x)    //hal.uartG->printf(x)
#define DEBUG_UARTG_PRINTF2(x,y)    //hal.uartG->printf(x,y)

void YP_Radio::_YP_Handle_Packet(ReceiverFcPacket* packet_in)
{
	switch (packet_in->type){

		case YPRC_PACKET_TYPE_CHANNELDATA12:
		{
			ChannelData12 packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_CHANNELDATA12);
	    	_handle_packet_channeldata12(&packet);
    	    break;
		}
	    case YPRC_PACKET_TYPE_CHANNELDATA24:
	    {
	    	ChannelData24 packet;
	    	memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_CHANNELDATA24);
	    	_handle_packet_channeldata24(&packet);
	    	break;
	    }_handle_packet_action
#if DEBUG_YP_RADIO == 1
		case YPRC_PACKET_TYPE_TELEMETRYDATA:
		{
			TelemetryData packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_TELEMETRYDATA);
			//HANDLE_PACKET_TELEMETRYDATA(&packet);
			break;
		}
#endif
	    case YPRC_PACKET_TYPE_CHANNELDATA12_GPS:
	    {
	    	ChannelData12_Gps packet;
	    	memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_CHANNELDATA12_GPS);
	    	_handle_packet_channeldata12_GPS(&packet);
	    	break;
	    }
#if DEBUG_YP_RADIO == 1
	    case YPRC_PACKET_TYPE_BINDCMD:
	    {
	    	StBindCmd packet;
	    	memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_STBINDCMD);
	    	//HANDLE_PACKET_BINDCMD(&packet);
	    	break;
	    }
#endif
	    case YPRC_PACKET_TYPE_ACTION:
	    {
	    	_handle_packet_action(packet_in);
	    	break;
	    }
		default:
			break;	    
	}
}

void YP_Radio::_handle_packet_action(ReceiverFcPacket* packet_in )
{
	switch(packet_in->yprc_data[0]){
		case ACTION_TYPE_REQUEST:
		{
			DEBUG_UARTG_PRINTF2("request action type:%d\n",(uint8_t)ACTION_TYPE_REQUEST);
			ActionRequest packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONREQUEST);
			_handle_action_request(&packet);
			break;
		}	
		case ACTION_TYPE_RESPONSE:
		{
			ActionResponse packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONRESPONSE);
			_handle_action_response(&packet);
			break;
		}	
	    case ACTION_TYPE_FEEDBACK:
	    {
	    	ActionFeedback packet;
	    	memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONFEEDBACK);
	    	//HANDLE_ACTION_FEEDBACK(&packet);
	    	break;
	    }	
		 case ACTION_TYPE_SETTING_CCC:
		{
			DEBUG_UARTG_PRINTF2("action type:%d\n",(uint8_t)ACTION_TYPE_SETTING_CCC);
			ActionSettingCCC packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONSETTING_CCC);
			_handle_action_settingccc(&packet);
			break;
		}
		case ACTION_TYPE_SETTING_ROI:
		{
			ActionSettingROI packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONSETTING_ROI);
			_handle_action_settingroi(&packet, FUNCTOR_BIND(&copter, &Copter::roi_config, bool, const Vector3ul&, const float&));
			break;
		}
		case ACTION_TYPE_SETTING_JOUR:
		{
			ActionSettingJOUR packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONSETTING_JOUR);
			_handle_action_settingjour(&packet, FUNCTOR_BIND(&copter, &Copter::jour_config, bool, uint8_t, uint8_t));
			break;
		}
		case ACTION_TYPE_CA_CONFIG:
		{
			ActionCAConfig packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_CACONFIG);
			if (packet.CA_on)	key_function.ca_on = true;
			else				key_function.ca_on = false;
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case ACTION_TYPE_LED_CONFIG:
		{
			ActionLEDConfig packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_LEDCONFIG);
			if (packet.ledStatus == 0) {
				AP_Notify::flying_flags.photo_at_night = true;
			} else {
				AP_Notify::flying_flags.photo_at_night = false;
			}
			send_response(ACTION_RESULT_OK, 0);
			break;
		}
		case ACTION_TYPE_ONEKEY_TAKEOFF:
		{
			ActionTakeOff packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONTAKEOFF);
			_handle_action_takeoff(&packet, FUNCTOR_BIND(&copter, &Copter::motors_auto_arming, bool, uint8_t, uint8_t));
			break;
		}
		case ACTION_TYPE_ASK_FOR_CONFIG:
		{
			ActionAskForConfig packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ASKFORCONFIG);
			switch (packet.whichConfig) {
			case ACTION_TYPE_GOHOME_CONFIG:
			{
				_send_gohome_config(copter.g.gohome_altitude, 0);
				break;
			}
			default:
			{
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				break;
			}
			}
			break;
		}
		case ACTION_TYPE_GOHOME_CONFIG:
		{
			ActionGoHomeConfig packet;
			memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_GOHOMECONFIG);
			if (packet.goHomeHeight >= GOHOME_ALT_MIN && packet.goHomeHeight <= GOHOME_ALT_MAX && copter.g.gohome_altitude.set_and_save_ifchanged(packet.goHomeHeight)) {
				send_response(ACTION_RESULT_OK, 0);
			} else {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
			}
			break;
		}
		/*----------------------------------------------------------------------------------------------------------------
				below is the new protocol
		----------------------------------------------------------------------------------------------------------------*/
		case ACTION_TYPE_MAP_WAYPOINT:	//
		{
			switch(packet_in->yprc_data[1])
			{
				case ACTION_REQUEST_CONFIG:
				{
					MapConfigRequest packet;
					memcpy((char*)&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_MAPCONFIGREQUEST);
					_handle_MapConfigRequest(&packet,map_waypoint_set_type_yp);

				}
				break;
				case ACTION_REQUEST_SET:
				{
					MapWaypointSetRequest packet;
					memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_MAPSETWAYPOINTREQUEST);
					_handle_MapSetWaypointRequest(&packet,map_waypoint_set_type_yp);
				}break;
				case ACTION_REQUEST_START_AUTO:
				case ACTION_REQUEST_PAUSE:
				case ACTION_REQUEST_RESUME:
				case ACTION_REQUEST_EXIT:
				case ACTION_REQUEST_GET:
				{
					DEBUG_UARTG_PRINTF2("action %d \n",packet_in->yprc_data[1]);
					MapWaypointRequest packet;
					memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_MAPWAYPOINTREQUEST);
					_handle_MapWaypointRequest(&packet,map_waypoint_set_type_yp);
					break;
				}
			}

			break;
		}
		case ACTION_TYPE_TAKEOFF:
		{
			ActionTakeOffStart packet_start;		//new protocol packet
			ActionTakeOff packet;					//old prototol packet
			memcpy((char* )&packet_start, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_ACTIONTAKEOFFSTART);
			packet.actionType = packet_start.action_type;
			packet.takeoffHeight = packet_start.takeoffHeight;
			packet.takeoffSpeed = packet_start.takeoffSpeed;
			//use the old method
			_handle_action_takeoff(&packet, FUNCTOR_BIND(&copter, &Copter::motors_auto_arming, bool, uint8_t, uint8_t));
			break;
		}
		case ACTION_TYPE_FENCE:
		{
			switch(packet_in->yprc_data[1])
			{
				case ACTION_REQUEST_CONFIG:
				{
					DEBUG_UARTG_PRINTF1("fence config \n");
					FenceConfigRequest	packet;
					memcpy((char* )&packet,	_YP_PACKET_DATA(packet_in), PACKET_LENGTH_FENCCONFIGREQUEST);
					_handle_fenceConfigRequest(&packet);

					break;
				}
				case ACTION_REQUEST_SET:
				{
					FenceSetWaypointRequest packet;
					memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_FENCESETWAYPOINTREQUEST);
					_handle_FenceSetWaypointRequest(&packet);
					break;
				}
				case ACTION_REQUEST_START_AUTO: //hal.console->printf("action start \n");
				case ACTION_REQUEST_PAUSE:
				case ACTION_REQUEST_RESUME:
				case ACTION_REQUEST_EXIT:
				case ACTION_REQUEST_GET:
				{
					FenceRequest packet;
					memcpy((char* )&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_FENCEREQUEST);
					_handle_FenceRequest(&packet);
					break;
				}
			}
			break;
		}
		case ACTION_TYPE_LANDING:
		{
			switch(packet_in->yprc_data[1])
			{
				case ACTION_REQUEST_START_AUTO:
					LandingRequest packet;
					memcpy((char*)&packet, _YP_PACKET_DATA(packet_in), PACKET_LENGTH_LANDINGREQUEST);
					DEBUG_UARTG_PRINTF2("ACTION_TYPE_LANDING:%d\n",(uint8_t)ACTION_TYPE_LANDING);
					_handle_LandingRequest(&packet);
					break;
				case ACTION_REQUEST_EXIT:
					// exit unconditionally
					_handle_exit(ACTION_TYPE_LANDING,FUNCTOR_BIND(&copter, &Copter::reset_control_switch, void),map_waypoint_set_type_yp);
					break;
			}
			break;
		}
		default:
			break;
	}
}


void YP_Radio::_handle_action_request(ActionRequest* packet_in)
{
	static uint8_t count = 0;
	switch(packet_in->actionRequest){

		case ACTION_REQUEST_START_AUTO:
		{
			_mode_type = (ModeType_e)(packet_in->modeType);
			DEBUG_UARTG_PRINTF1("handle_start_auto");
			// start to handle startauto mission in specific mode
			_handle_start_auto( FUNCTOR_BIND(&copter, &Copter::CCC_start, bool),
								FUNCTOR_BIND(&copter, &Copter::roi_start, bool),
								FUNCTOR_BIND(&copter, &Copter::journey_start, bool),
								FUNCTOR_BIND(&copter, &Copter::circle_me_config, void));
			break;
		}

		case ACTION_REQUEST_START_MANUAL:
		{
			// manual is not avaiable now
			break;
		}

		case ACTION_REQUEST_PAUSE:
		{
			if(_mode_type == packet_in->modeType && _action_state == ACTION_STATUS_RUNNING) {
				_handle_pause(FUNCTOR_BIND(&copter,&Copter::journey_stop,void));
			} else if (_mode_type == packet_in->modeType && _action_state == ACTION_STATUS_STOPPED) {
				// in case of too many request cmd is in sequence, copter will send back same response
				send_response(ACTION_RESULT_OK, 0);
			} else {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
			}
			break;
		}

		case ACTION_REQUEST_RESUME:
		{
			//resume must meet last process, mode and action is the same
			if (_mode_type == packet_in->modeType && _action_state == ACTION_STATUS_STOPPED) {
				_handle_resume(FUNCTOR_BIND(&copter,&Copter::journey_resume,void));
			} else if (_mode_type == packet_in->modeType && _action_state == ACTION_STATUS_RUNNING) {
				// in case of too many request cmd is in sequence. 
				// if already successed to excute cmd, copter will send back same response
				send_response(ACTION_RESULT_OK, 0);
			} else {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
			}
			break;
		}

		case ACTION_REQUEST_EXIT:
		{
			// exit unconditionally
			_handle_exit(packet_in->actionType, FUNCTOR_BIND(&copter, &Copter::reset_control_switch, void),map_waypoint_set_type_none);
			break;
		}

		// now this cmd ask for current status of copter, gcs will store it
		// there is no need to write the index, gcs will get it done
		case ACTION_REQUEST_GET:
		{
			count++;
			DEBUG_UARTG_PRINTF1("handle ACTION_REQUEST_GET");
			if (packet_in->modeType == AUTO_CURVECABLECAM) {
				DEBUG_UARTG_PRINTF1(PSTR("ACTION_REQUEST_GET ccc point"));
				// send back current position, current gimbal angle
				send_action_setting_ccc();
			} else {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
			}
			break;
		}

		// Two mode support this set: roi and ccc 
		case ACTION_REQUEST_SET:
		{
			DEBUG_PRINTFLN("handle ACTION_REQUEST_SET");
			// switch modetype, enter into different set mode
			if (_mode_type == AUTO_NONES && _action_state == ACTION_STATUS_IDLE) {
				_mode_type = (ModeType_e)packet_in->modeType;
				_action_state = ACTION_STATUS_SETTING;
				_setting_count = packet_in->settingCount;
				_handle_set(FUNCTOR_BIND(&copter, &Copter::roi_reset, void));
			// if we are in second handshake process, and the last response for ok is lost to rc
			} else if (_mode_type == (ModeType_e)packet_in->modeType) {
				// in case of too many request cmd is in sequence, copter will send back same response
				send_response(ACTION_RESULT_OK, 0);
			} else {	
				// if the modetype is not right in this process, it will send fail response
				send_response(ACTION_RESULT_ERR_FAIL, 0);
			}
			break;
		}

		case ACTION_REQUEST_CHECK:
		{
			DEBUG_PRINTFLN("ACTION_REQUEST_CHECK");

			if (packet_in->modeType != _mode_type) {
				send_response(ACTION_RESULT_ERR_FAIL, 0);
				return;
			}
			break;
		}

		default: 
			break;
	}
}


void YP_Radio::_handle_MapConfigRequest(MapConfigRequest *packet_in,Map_waypoint_set_type set_type)
{
	//
	if (_map_waypoint_set_type != map_waypoint_set_type_none && _map_waypoint_set_type != set_type) {
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_CONFIG, ACTION_RESULT_ERR_FAIL, 0,set_type);
		return;
	}

	//MapConfigRequest rx_map_parameters;
	if (packet_in->action_type != ACTION_TYPE_MAP_WAYPOINT || packet_in->action_request != ACTION_REQUEST_CONFIG) {
		return;
	}

	// switch modetype, enter into different set mode
	if (_mode_type == AUTO_NONES && _action_state == ACTION_STATUS_IDLE) {
		_mode_type = AUTO_MAP_WAYPOINT;
		_action_state = ACTION_STATUS_SETTING;

		_setting_count = packet_in->waypointCount;
		_airline_type  = packet_in->airline_type;
		_airline_ending_action = packet_in->airline_ending_action;
		_airline_altitude_type = packet_in->altitude_type;
		_airline_default_speed = packet_in->airline_default_speed;
		_pan_mode = packet_in->pan_mode;
		_tilt_mode= packet_in->tilt_mode;
		DEBUG_UARTG_PRINTF2("pan%d",_pan_mode);
		DEBUG_UARTG_PRINTF2("tilt%d",_tilt_mode);
		_handle_waypoint_config(set_type);
	// if we are in second handshake process, and the last response for ok is lost to rc
	} else if (_mode_type == AUTO_MAP_WAYPOINT) {
		// in case of too many request cmd is in sequence, copter will send back same response.
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_CONFIG, ACTION_RESULT_OK, 0,set_type);
	} else {
		// if the modetype is not right in this process, it will send fail response.
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_CONFIG, ACTION_RESULT_ERR_FAIL, 0,set_type);
	}
}

void YP_Radio::_handle_fenceConfigRequest(FenceConfigRequest *packet_in)
{
	if (packet_in->action_type != ACTION_TYPE_FENCE || packet_in->action_request != ACTION_REQUEST_CONFIG) {
		send_map_fence_response(ACTION_TYPE_FENCE ,ACTION_REQUEST_CONFIG,ACTION_RESULT_OK,0); //send response three times in case RF lost frames
		return;
	}
	if (_mode_type == AUTO_NONES && _action_state == ACTION_STATUS_IDLE) {
		AC_Fence::Y_FenceConfig fenceCfg;
		_setting_count = packet_in->gpsPointCount;
		fenceCfg.counts = packet_in->gpsPointCount;
		fenceCfg.fenceTypes = packet_in->fencetype;
		fenceCfg.altitudeType = packet_in->altitudeType;
		fenceCfg.alt = packet_in->alt;
		if(_fence.init_fence(fenceCfg)){
			_mode_type = MAP_FENCE;
			_action_state = ACTION_STATUS_SETTING;
			// notify user entered setting
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_SETTING;
			send_map_fence_response(ACTION_TYPE_FENCE ,ACTION_REQUEST_CONFIG,ACTION_RESULT_OK,0);
			DEBUG_UARTG_PRINTF1("Action request fence config ok \n");
		}else{
			send_map_fence_response(ACTION_TYPE_FENCE ,ACTION_REQUEST_CONFIG,ACTION_RESULT_ERR_FAIL,0);
			DEBUG_UARTG_PRINTF1("Action request fence config not ok \n");
		}

	} else if (_mode_type == MAP_FENCE) {
		//in case of too many config request cmd is in sequence ,copter will send back same response
		send_map_fence_response(ACTION_TYPE_FENCE ,ACTION_REQUEST_CONFIG,ACTION_RESULT_OK,0); //send response three times in case RF lost frames
		DEBUG_UARTG_PRINTF1("Action requset fence config already ok\n");
	} else {
		//if the mode type is not right in this process , it will send fail response
		send_map_fence_response(ACTION_TYPE_FENCE ,ACTION_REQUEST_CONFIG,ACTION_RESULT_ERR_FAIL,0);
		DEBUG_UARTG_PRINTF1("mode the diffenent with pre set\n");
	}
}
// ack for the map waypoint, its a request.
void YP_Radio::_handle_MapWaypointRequest(MapWaypointRequest *packet_in,Map_waypoint_set_type set_type)
{
	//if data chain changed ,return immediate
	//in case that packet lost,when copter have exit the _map_waypoint_set_type will be map_waypoint_set_type_none
	if (_map_waypoint_set_type != map_waypoint_set_type_none && _map_waypoint_set_type != set_type) {
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
		return ;
	}

	if (packet_in->action_type != ACTION_TYPE_MAP_WAYPOINT) {
		send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0,set_type);
		return;
	}
	switch(packet_in->action_request) {
	case ACTION_REQUEST_START_AUTO:
	{
		_mode_type = AUTO_MAP_WAYPOINT;
		// delivery in the function reference, set_mode function, XXXX_start function, XXXX_config funtion.
		_handle_waypoint_start(copter.ap.land_complete,copter.ap.auto_armed,FUNCTOR_BIND(&copter, &Copter::set_mode, bool, uint8_t));
		break;
	}

	case ACTION_REQUEST_PAUSE:
	{
		DEBUG_UARTG_PRINTF1("map waypoint action pause \n");
		// pause must meet the process, mode and the status must matched.
		if (_action_state == ACTION_STATUS_RUNNING) {
			_action_state = ACTION_STATUS_STOPPED;
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_STOPPED;
			mission_state = MissionState_Pause;
			_wp_nav.set_MapWaypoint_stop(true);
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_PAUSE, ACTION_RESULT_OK, 0,set_type);
		} else if (_action_state == ACTION_STATUS_STOPPED) {
			// in case of too many request cmd is in sequence, copter will send back same response
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_PAUSE, ACTION_RESULT_OK, 0,set_type);
		} else {
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_PAUSE, ACTION_RESULT_ERR_FAIL, 0,set_type);
		}
		break;
	}

	case ACTION_REQUEST_RESUME:
	{
		DEBUG_UARTG_PRINTF1("map waypoint action resume \n");
		// resume must meet the process, mode and the status must matched.
		if (_action_state == ACTION_STATUS_STOPPED) {
			// notify user photo mode is running
			AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
			mission_state = MissionState_Running;
			_wp_nav.set_MapWaypoint_stop(false);
			_action_state = ACTION_STATUS_RUNNING;
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_RESUME, ACTION_RESULT_OK, 0,set_type);
		} else if (_action_state == ACTION_STATUS_RUNNING) {
			// in case of too many request cmd is in sequence, copter will send back same response
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_RESUME, ACTION_RESULT_OK, 0,set_type);
		} else {
			send_map_waypoint_response(ACTION_TYPE_MAP_WAYPOINT, ACTION_REQUEST_RESUME, ACTION_RESULT_ERR_FAIL, 0,set_type);
		}
		break;
	}

	case ACTION_REQUEST_EXIT:
	{
		// exit unconditionally
		_handle_exit(packet_in->action_type,FUNCTOR_BIND(&copter, &Copter::reset_control_switch, void),set_type);
		break;
	}

	// noe this cmd ask for current status of copter, gcs will store it
	// there is no need write the index, gcs will get it done.
	case ACTION_REQUEST_GET:
	{
		//send_action_current_mapwapoint();			now no use this cmd
		break;
	}

	default:
		break;
	}
}


void YP_Radio::_handle_FenceRequest(FenceRequest *packet_in)
{
	if (packet_in->action_type != ACTION_TYPE_FENCE) {
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_SET, ACTION_RESULT_ERR_FAIL, 0);
		return;
	}
	switch(packet_in->action_request) {
	case ACTION_REQUEST_START_AUTO:
	{

		// delivery in the function reference, set_mode function, XXXX_start function, XXXX_config funtion.
		_fence.set_fence_enabled(true);
		//Notify user photo mode is running
		_running_action_type = ACTION_TYPE_FENCE;
		mission_state = MissionState_Running;
		_mode_type = MAP_FENCE;
		AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;//setting is ended
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_START_AUTO, ACTION_RESULT_OK, 0);
		break;
	}

	case ACTION_REQUEST_EXIT:
	{
		// exit unconditionally
		_fence.set_fence_enabled(false);
		_handle_exit(packet_in->action_type,FUNCTOR_BIND(&copter, &Copter::reset_control_switch, void),map_waypoint_set_type_yp);
		break;
	}

	case ACTION_REQUEST_PAUSE:
	{
		// pause must meet the process, mode and the status must matched.
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_PAUSE, ACTION_RESULT_OK, 0);
		break;
	}
	case ACTION_REQUEST_RESUME:
	{
		// resume must meet the process, mode and the status must matched.
		// in case of too many request cmd is in sequence, copter will send back same response
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_RESUME, ACTION_RESULT_OK, 0);
		break;
	}

	// noe this cmd ask for current status of copter, gcs will store it
	// there is no need write the index, gcs will get it done.
	case ACTION_REQUEST_GET:
	{
		send_map_fence_response(ACTION_TYPE_FENCE, ACTION_REQUEST_GET, ACTION_RESULT_ERR_FAIL, 0);
		break;
	}

	default:
		break;
	}


}


void YP_Radio::_handle_LandingRequest(LandingRequest *packet_in)
{
	//MapConfigRequest rx_map_parameters;
	if (packet_in->action_type != ACTION_TYPE_LANDING || packet_in->action_request != ACTION_REQUEST_START_AUTO || AP_Notify::diagnose_flags.low_battery_warning >= 1) {
		send_takeoff_landing_response(ACTION_TYPE_LANDING, ACTION_REQUEST_START_AUTO, ACTION_RESULT_ERR_FAIL);
		return;
	}

	if (_mode_type != AUTO_NONES || _action_state != ACTION_STATUS_IDLE || AP_Notify::diagnose_flags.low_battery_warning >= 1) {
		send_takeoff_landing_response(ACTION_TYPE_LANDING, ACTION_REQUEST_START_AUTO, ACTION_RESULT_ERR_FAIL);
		return;
	}

	LandingWayPoint landing_data;

	if (!copter.ap.land_complete) {
		// in case of too many request cmd is in sequence, copter will send back same response.
		landing_data.lat = packet_in->lat;
		landing_data.lon = packet_in->lon;
		landing_data.alt = max(_cur_loc.alt,max(500,copter.g.gohome_altitude/2));
		if (_handle_landingMission(&landing_data)) {
		_running_action_type = ACTION_TYPE_LANDING;
		mission_state = MissionState_Running;
		_mode_type = MAP_FENCE;
			send_takeoff_landing_response(ACTION_TYPE_LANDING, ACTION_REQUEST_START_AUTO, ACTION_RESULT_OK);
		} else {
			send_takeoff_landing_response(ACTION_TYPE_LANDING, ACTION_REQUEST_START_AUTO, ACTION_RESULT_ERR_FAIL);
		}
	} else {
		// if the modetype is not right in this process, it will send fail response.
		send_takeoff_landing_response(ACTION_TYPE_LANDING, ACTION_REQUEST_START_AUTO, ACTION_RESULT_ERR_FAIL);
	}
}

bool YP_Radio::_handle_landingMission(LandingWayPoint *landpoint)
{
	AP_Mission::MapWayPointConfig mapWayPointConfig;
	mapWayPointConfig.waypointCount = 2;
	mapWayPointConfig.airline_type = AIRLINE_TYPE_STRAIGHT;
	mapWayPointConfig.airline_ending_action  = AIRLINE_ENDING_LANDING;
	mapWayPointConfig.altitude_type = AltitudeType_RELATIVE;
	mapWayPointConfig.airline_default_speed = 10;
	mapWayPointConfig.pan_mode = Pan_FollowRoute;
	mapWayPointConfig.tilt_mode = Tilt_Manual;
	// init the map_waypoint mode.
	if (_mission.init_map_waypoint(&mapWayPointConfig)) {
		AP_Mission::MapWaypoint map_waypoint;
		//the first point is above the current location
		map_waypoint.waypointIndex = 0;
		map_waypoint.lat = _cur_loc.lat;
		map_waypoint.lon = _cur_loc.lng;
		map_waypoint.alt = landpoint->alt;
		map_waypoint.speed = 20;
		map_waypoint.waitTime = 0;
		map_waypoint.pan_angle = 0;
		map_waypoint.tilt_mode = Tilt_Manual;
		map_waypoint.tilt_angle = 0;
		map_waypoint.doCmdCount = 0;
		map_waypoint.do_cmd = NULL;

		//add first point
		_mission.add_map_waypoint_point(map_waypoint);
		//second point:above the landing location
		map_waypoint.waypointIndex = 1;
		map_waypoint.lat = landpoint->lat;
		map_waypoint.lon = landpoint->lon;
		map_waypoint.alt = landpoint->alt;
		map_waypoint.speed = 25;
		map_waypoint.waitTime = 0;
		map_waypoint.pan_angle = 0;
		map_waypoint.tilt_mode = Tilt_Manual;
		map_waypoint.tilt_angle = 0;
		map_waypoint.doCmdCount = 0;
		map_waypoint.do_cmd = NULL;
		//add second point
		if (_mission.add_map_waypoint_point(map_waypoint)) {
			map_waypoint.waypointIndex = 0;
			map_waypoint.lat = _cur_loc.lat;
			map_waypoint.lon = _cur_loc.lng;
			map_waypoint.alt = _cur_loc.alt;
			map_waypoint.waitTime = 0;
			map_waypoint.pan_angle = 0;
			map_waypoint.tilt_mode = Tilt_Manual;
			map_waypoint.tilt_angle = 0;
			map_waypoint.doCmdCount = 0;
			map_waypoint.do_cmd = NULL;

			_mission.set_start_map_waypoint(map_waypoint);

			_mission.set_yaw_looking_cmd(AP_Mission::LOOK_AT_HOLD);
			_mission.set_map_waypoint_auto_takeoff_enable(false);
			if (copter.set_mode(AUTO)) {
				_running_action_type = ACTION_TYPE_LANDING;
				mission_state = MissionState_Running;

				//Notify user photo mode is running
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_RUNNING;
				return true;
			} else {
				AP_Notify::flying_flags.photo_mode = AP_Notify::Photo_Mode_NONE;
				_mode_type = AUTO_NONES;
				_action_state = ACTION_STATUS_IDLE;
				return false;
			}
		} else {
			return false;
		}
	} else {
		return false;
	}
	return true;
}

void YP_Radio::_handle_wp_mission_feedback_send(void)
{
	uint8_t drone_state_tmp = 0;

	if (copter.ap.land_complete) {
		drone_state_tmp |= 0x01<<DRONE_STATE_LAND_COMPLETE_BIT;
	}

	if (copter.ap.auto_armed) {
		drone_state_tmp |= 0x01<<DRONE_STATE_MOTOR_ARMED_BIT;
	}

	send_action_mission_feedback(_running_action_type, drone_state_tmp, _mission.get_reached_nav_index(), mission_state);

}

/*void YP_Radio::Wifi_MapConfigRequest(MapConfigRequest *packet_in)
{
	_handle_MapConfigRequest(packet_in, map_waypoint_set_type_wifi);
}

void YP_Radio::Wifi_MapSetWaypointRequest(MapWaypointSetRequest *packet_in)
{
	_handle_MapSetWaypointRequest(packet_in, map_waypoint_set_type_wifi);
}

void YP_Radio::Wifi_MapWaypointRequest(MapWaypointRequest *packet_in)
{
	_handle_MapWaypointRequest(packet_in, map_waypoint_set_type_wifi);
}

void YP_Radio::Wifi_MapWayPointResponse(ActionRequest_e request, ActionResult_e result, uint8_t index)
{
	_gimbalcamera.send_waypoint_response(request, result, index);
}

void YP_Radio::Wifi_MapWayPointFeedback(uint8_t index, MissionState state)
{
	_gimbalcamera.send_waypoint_feedback(index, state);
}*/
