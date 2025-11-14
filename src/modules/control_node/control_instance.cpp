#include "control_instance.h"
control_instance* control_instance::instance = nullptr;

bool control_instance::Change_offborad()
{
_vehicle_status_sub.copy(&_status);
 _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        ocm.timestamp = hrt_absolute_time();
        _vehicle_local_position_sub.copy(&_vehicle_local_position);
        sp_local.x=_vehicle_local_position.x;
        sp_local.y=_vehicle_local_position.y;
        sp_local.z=_vehicle_local_position.z-5;
        sp_local.timestamp = hrt_absolute_time();
        _trajectory_setpoint_pub.publish(sp_local);
        ocm.position=true;
        ocm.timestamp = hrt_absolute_time();
        _offboard_control_mode_pub.publish(ocm);
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        _command.param1=1.0f;
        _command.param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);

    if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
        {
return true;
        }
        else
        {
        return false;
        }
}

bool control_instance::Arm_vehicle()
{
 _command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        _command.param1 = 1.0f;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);
         if(_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED)
        {
return true;
        }
                else
        {
        return false;
        }
}
bool control_instance::Control_posxyz(float x,float y,float z)
{
 memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=x;
                sp_local.y=y;
                sp_local.z=z;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                ocm.position=true;
                ocm.velocity=false;
                ocm.acceleration=false;
                time_tick=hrt_absolute_time();
                Pubctl();
                 _vehicle_local_position_sub.copy(&_vehicle_local_position);
                 _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);
                                 if(_vtol_vehicle_status.vehicle_vtol_state==vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW)
                                 {
                if((abs(_vehicle_local_position.x-sp_local.x)<fw_xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<fw_xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<fw_z_rad))
                {
                return true;
                }
                else
                {
                return false;
                }
                                 }
else
{
                if((abs(_vehicle_local_position.x-sp_local.x)<xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<z_rad))
                {
                return true;
                }
                else
                {
                return false;
                }
}


}
bool control_instance::Control_posxyz_yaw(float x,float y,float z,float yaw)
{
return false;
}
bool control_instance::Control_posxyz_yawspeed(float x,float y,float z,float yawspeed)
{
return false;
}
bool control_instance::Control_posz_velxy(float vx,float vy,float z)
{
 return false;
}
bool control_instance::Control_posz_accxy(float ax,float ay,float z)
{
 return false;
}
bool control_instance::Change_land()
{
            _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
            _command.param1=1.0f;
            _command.param2=PX4_CUSTOM_MAIN_MODE_AUTO;
            _command.param3=PX4_CUSTOM_SUB_MODE_AUTO_LAND;
            _command.timestamp = hrt_absolute_time();
            _vehicle_status_sub.copy(&_status);
            _vehicle_command_pub.publish(_command);
                if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
        {
return true;
        }
        else
        {
        return false;
        }
}

bool control_instance::Control_lat_lon_alt(float lat,float lon,float alt)
{
        float x,y,z;
        vehicle_local_position_s local_pos{};
			_vehicle_local_position_sub.copy(&local_pos);

			if (!local_pos.xy_global || !local_pos.z_global) {
				return false;
			}

			MapProjection global_local_proj_ref{local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp};

			// global -> local
			// const double lat = target_global_int.lat_int / 1e7;
			// const double lon = target_global_int.lon_int / 1e7;
			global_local_proj_ref.project(lat, lon, x, y);

                       z =alt- local_pos.ref_alt;

 memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=x;
                sp_local.y=y;
                sp_local.z=z;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                ocm.position=true;
                ocm.velocity=false;
                ocm.acceleration=false;
                time_tick=hrt_absolute_time();
                Pubctl();
                 _vehicle_local_position_sub.copy(&_vehicle_local_position);
                if((abs(_vehicle_local_position.x-sp_local.x)<xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<z_rad))
                {
                return true;
                }
                else
                {
                return false;
                }
}



bool control_instance::hold_curr_pos(float time_s)

{
        vehicle_local_position_s local_pos{};
			_vehicle_local_position_sub.copy(&local_pos);
                         memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=local_pos.x;
                sp_local.y=local_pos.y;
                sp_local.z=local_pos.z;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                ocm.position=true;
                ocm.velocity=false;
                ocm.acceleration=false;
                Pubctl();
                if(!time_flag)
                {
                        time_tick=hrt_absolute_time();
                        time_flag=true;
                }
                if((hrt_absolute_time()-time_tick)>time_s*1000000)
                {
                return true;
                time_flag=true;
                }
                else
                {
                return false;
                }
                PX4_INFO("HOLD_CURR_POS");

}
void control_instance::Pubctl()
{
if(ocm.position||ocm.velocity||ocm.acceleration)
{
        ocm.timestamp = hrt_absolute_time();
        _offboard_control_mode_pub.publish(ocm);
        _vehicle_status_sub.copy(&_status);
if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
{
        sp_local.timestamp = hrt_absolute_time();
        _trajectory_setpoint_pub.publish(sp_local);
//        PX4_INFO("sp_local.x=%lf\n",(double)sp_local.x);
//        PX4_INFO("sp_local.y=%lf\n",(double)sp_local.y);
//        PX4_INFO("sp_local.z=%lf\n",(double)sp_local.z);
//        PX4_INFO("sp_local.vx=%lf\n",(double)sp_local.vx);
//        PX4_INFO("sp_local.vy=%lf\n",(double)sp_local.vy);
//        PX4_INFO("sp_local.vz=%lf\n",(double)sp_local.vz);
}
}
}

bool control_instance::Control_mc_to_fw()
{
        // _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        //     _command.param1=1.0f;
        //     _command.param2=PX4_CUSTOM_MAIN_MODE_POSCTL;
        //     _command.timestamp = hrt_absolute_time();
        //     _vehicle_command_pub.publish(_command);
        //     _vehicle_status_sub.copy(&_status);
        //         if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_POSCTL)
        // {
        _vehicle_status_sub.copy(&_status);
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
        _command.param1=(float)(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);
        // }
        _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);
        PX4_INFO("mc_to_fw");
        if(_vtol_vehicle_status.vehicle_vtol_state==vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW)
        {
//                 _vehicle_status_sub.copy(&_status);
//  _command.target_system = _status.system_id;
//         _command.target_component = _status.component_id;
//         ocm.timestamp = hrt_absolute_time();
//         _vehicle_local_position_sub.copy(&_vehicle_local_position);
//         sp_local.x=_vehicle_local_position.x+100;
//         sp_local.y=_vehicle_local_position.y;
//         sp_local.z=_vehicle_local_position.z-50;
//         sp_local.timestamp = hrt_absolute_time();
//         _trajectory_setpoint_pub.publish(sp_local);
//         ocm.position=true;
//         ocm.timestamp = hrt_absolute_time();
//         _offboard_control_mode_pub.publish(ocm);
//          _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
//         _command.param1=1.0f;
//         _command.param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD;
//         _command.timestamp = hrt_absolute_time();
//         _vehicle_command_pub.publish(_command);

//                     if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
//                            {
                                 return true;
                           }
                         else
                                  {
                          return false;
                                 }
                        //   }
                        //         else
                        //  {
                        //  return false;
                        // }
}
void control_instance::SetYaw(float new_yaw_rad)
{
    // 简单实现思路：publish vehicle_attitude_setpoint_s 或 position setpoint with yaw
    vehicle_attitude_setpoint_s att_sp{};
    att_sp.timestamp = hrt_absolute_time();
    // 把 yaw 转为 quaternion（only yaw）
    matrix::Quatf q = matrix::Quatf(matrix::Eulerf(0.0f, 0.0f, new_yaw_rad));
  att_sp.q_d[0] = q(0);
att_sp.q_d[1] = q(1);
att_sp.q_d[2] = q(2);
att_sp.q_d[3] = q(3);

    att_sp.yaw_sp_move_rate = 0.0f;
    // publish 使用 control_instance 内部的 publisher（若没有，可新增）
    _att_sp_pub.publish(att_sp);
}

bool control_instance::Control_fw_to_mc()
{
        _vehicle_status_sub.copy(&_status);
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
        _command.param1=(float)(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);

                _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);
        PX4_INFO("fw_to_mc");
        if(_vtol_vehicle_status.vehicle_vtol_state==vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC)
        {
                        return true;
        }
        else
        {
                         return false;
        }
}
