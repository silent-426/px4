#include "swarm_node.h"
// mc_control_instance* mc_control_instance::instance = nullptr;
Swarm_Node::Swarm_Node() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::swarm_node)
{
}

Swarm_Node::~Swarm_Node()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool Swarm_Node::init()
{
	// execute Run() on every sensor_accel publication
	//if (!_sensor_accel_sub.registerCallback()) {
	//	PX4_ERR("callback registration failed");
	//	return false;
	//}

	// alternatively, Run on fixed interval
	 ScheduleOnInterval(20000_us); // 2000 us interval, 200 Hz rate

	return true;
}
bool Swarm_Node::takeoff()
{
control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-5);
return false;
}
bool Swarm_Node::arm_offboard()
{
return false;
}
bool Swarm_Node::swarm_node_init()
{

	_vehicle_local_position_sub.copy(&_vehicle_local_position);

        _a01_sub.copy(&_target);
	_a02_sub.copy(&_start_flag);

        //        _target.lat= 47.3978161;
        //        _target.lon=8.5460368;
	    //   PX4_INFO("_start_flag.start_swarm=%d",_start_flag.start_swarm);
        if((_vehicle_local_position.xy_valid)&&_start_flag.start_swarm)
{
	 vehicle_status_s _vehicle_status;
	    if(_vehicle_status_sub.copy(&_vehicle_status))
    {
    vehicle_id=_vehicle_status.system_id;
    if(vehicle_id>1)
    {
        _global_local_proj_ref.initReference(_vehicle_local_position.ref_lat,_vehicle_local_position.ref_lon,hrt_absolute_time());
        _global_local_proj_ref.project(_target.lat,_target.lon,target_x,target_y);
        float dist =sqrtf((_vehicle_local_position.x-target_x)*(_vehicle_local_position.x-target_x)+(_vehicle_local_position.y-target_y)*(_vehicle_local_position.y-target_y));

            //  mavlink_log_info(&_mavlink_log_pub, "与目标的距离:%f",_a02_dist_to_target.dist_to_target);
        if(dist<200)
        {
        //   mavlink_log_info(&_mavlink_log_pub, "与目标的距离:%f",_a02_dist_to_target.dist_to_target);
	   _vehicle_local_position_sub.copy(&_vehicle_local_position);
   	 begin_x=_vehicle_local_position.x;
	begin_y=_vehicle_local_position.y;
   	 begin_z=_vehicle_local_position.z;
   	 _global_local_proj_ref.initReference(_vehicle_local_position.ref_lat,_vehicle_local_position.ref_lon,hrt_absolute_time());


                 return true;
        }
        else
        {
		return false;
        }
}
else if(vehicle_id==1)
{
	time_tick=hrt_absolute_time();
return true;
}
}

}
else
{
return false;
}
return false;
}


void Swarm_Node::start_swarm_node()
{
	if(vehicle_id>1)
	{
// 		float airsp_trim=20.0f;
// 		param_t param_airsp_trim= param_find("FW_AIRSPD_TRIM");
//  param_set(param_airsp_trim, &airsp_trim);
//    updateParams();
	a02_s _a02{};
	_a02_sub.copy(&_a02);
	if(_a02.stop_swarm)
	{
	control_instance::getInstance()->Change_land();
	}
	else
	{
		_vehicle_local_position_sub.copy(&_vehicle_local_position);
        _a01_sub.copy(&_target);
        //        _target.lat= 47.3978161;
        //        _target.lon=8.5460368;
	// PX4_INFO("_target.lat=%lf",_target.lat);
	// PX4_INFO("_target.lon=%lf",_target.lon);
	// PX4_INFO("_target.yaw=%lf",(double)_target.yaw);
        _global_local_proj_ref.project(_target.lat,_target.lon,target_x,target_y);
	PX4_INFO("target_x=%lf",(double)target_x);
	PX4_INFO("target_y=%lf",(double)target_y);
	PX4_INFO("_target.yaw=%lf",(double)_target.yaw);
	fw_x=target_x-(vehicle_id-1)*10*cosf(matrix::wrap_2pi(_target.yaw));
	fw_y=target_y-(vehicle_id-1)*10*sinf(matrix::wrap_2pi(_target.yaw));
	PX4_INFO("fw_x=%lf",(double)fw_x);
	PX4_INFO("fw_y=%lf",(double)fw_y);
	control_instance::getInstance()->Control_posxyz(fw_x,fw_y,begin_z-50);
	}
	}
	if(vehicle_id==1)
	{
		//POINT_STATE=point_state::assemble;
		// if((hrt_absolute_time()-time_tick>0)&&(hrt_absolute_time()-time_tick<time_tick_point1))
		// {
		// control_instance::getInstance()->Control_posxyz(begin_x+100,begin_y,begin_z-50);
		// }
		// if((hrt_absolute_time()-time_tick>time_tick_point1)&&flag==0)
		// {
		// 	flag=1;
		// 	POINT_STATE=point_state::point0;
		// }
		
		switch(POINT_STATE)
		{
		case point_state::point0:
		if(control_instance::getInstance()->Control_posxyz(begin_x+500,begin_y,begin_z-50))
		{
			POINT_STATE=point_state::point1;
		}
		break;
		case point_state::point1: 
		if(control_instance::getInstance()->Control_posxyz(begin_x+500,begin_y-500,begin_z-50))
		{
			POINT_STATE=point_state::point2;
		}
		break;
		case point_state::point2:
		if(control_instance::getInstance()->Control_posxyz(begin_x,begin_y-500,begin_z-50))
		{
			POINT_STATE=point_state::point3;
		}
		break;
		case point_state::point3:
		if(control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-50))
		{
			POINT_STATE=point_state::land;
		}
		break;
		// case point_state::point5:
		// if(control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-50))
		// {
		// 	POINT_STATE=point_state::land;
		// 	// STATE=state::LAND;
		// }
		// break;
		case point_state::land:
		if(control_instance::getInstance()->Change_land())
		{
			POINT_STATE=point_state::end;
					a02_s _a02;
		_a02.stop_swarm=true;
		_a02_pub.publish(_a02);
		}
		break;
		case point_state::end:
		a02_s _a02;
		_a02.stop_swarm=true;
		_a02_pub.publish(_a02);
		break;
	        default:
	        break;
		}
		// if((hrt_absolute_time()-time_tick>0)&&(hrt_absolute_time()-time_tick<time_tick_point1))
		// {
		// control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-50);
		// }
		// if((hrt_absolute_time()-time_tick>time_tick_point1)&&(hrt_absolute_time()-time_tick<time_tick_point2))
		// {
		// control_instance::getInstance()->Control_posxyz(begin_x+500,begin_y,begin_z-50);
		// }
		// if((hrt_absolute_time()-time_tick>time_tick_point2)&&(hrt_absolute_time()-time_tick<time_tick_point3))
		// {
		// control_instance::getInstance()->Control_posxyz(begin_x+500,begin_y+500,begin_z-50);
		// }
		// if((hrt_absolute_time()-time_tick>time_tick_point3)&&(hrt_absolute_time()-time_tick<time_tick_point4))
		// {
		// control_instance::getInstance()->Control_posxyz(begin_x,begin_y+500,begin_z-50);
		// }
		// if((hrt_absolute_time()-time_tick>time_tick_point4)&&(hrt_absolute_time()-time_tick<time_tick_point5))
		// {
		// control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-50);
		// }
		// if(hrt_absolute_time()-time_tick>time_tick_point5)
		// {
		// control_instance::getInstance()->Change_land();
		// a02_s _a02;
		// _a02.stop_swarm=true;
		// _a02_pub.publish(_a02);
		// }
	}

}


void Swarm_Node::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
	switch(STATE)
	{
	case state::INIT:
	// recived_point=0;
	if(swarm_node_init())
	{
	STATE=state::ARM_OFFBOARD;
	}
	//PX4_INFO("INIT");
	break;
	case state::ARM_OFFBOARD:
	//PX4_INFO("ARM_OFFBOARD");
	if(control_instance::getInstance()->Change_offborad()&&control_instance::getInstance()->Arm_vehicle())
	{
	STATE=state::TAKEOFF;
	}
	break;
	case state::TAKEOFF:
	//PX4_INFO("TAKEOFF");
	if(control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-50))
	{
//		_sensor_gps_sub.copy(&_sensor_gps);
// init_gps_time=_sensor_gps.time_utc_usec;
//PX4_INFO("_sensor_gps.time_utc_usec=%lld",_sensor_gps.time_utc_usec);
	STATE=state::MC_TO_FW;
	}
	break;
	case state::MC_TO_FW:
	//PX4_INFO("TAKEOFF");
	if(control_instance::getInstance()->Control_mc_to_fw())
	{
	STATE=state::CONTROL;
	}
	break;
	case state::CONTROL:
	//PX4_INFO("SWARM");
	//mc_control_instance::getInstance()->Control_lat_lon_alt(x,y,z);
	start_swarm_node();
	break;

	case state::LAND:
	//PX4_INFO("LAND");
	control_instance::getInstance()->Change_land();
	break;
	case state::EMERGENCY:
	//PX4_INFO("EMERGENCY");
	break;
	default:
	break;
	}


	perf_end(_loop_perf);
}

int Swarm_Node::task_spawn(int argc, char *argv[])
{
	Swarm_Node *instance = new Swarm_Node();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Swarm_Node::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int Swarm_Node::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Swarm_Node::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int swarm_node_main(int argc, char *argv[])
{
	return Swarm_Node::main(argc, argv);
}
