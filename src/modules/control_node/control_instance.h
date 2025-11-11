#pragma once
#include <geo/geo.h>
#include "commander/px4_custom_mode.h"
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vtol_vehicle_status.h>
using namespace time_literals;


class control_instance{
public:
	bool Change_offborad() ;
	bool Change_land() ;
	bool Change_return() ;
	bool Arm_vehicle() ;
	bool Control_posxyz(float x,float y,float z) ;
	bool Control_posxyz_yaw(float x,float y,float z,float yaw) ;
	bool Control_posxyz_yawspeed(float x,float y,float z,float yawspeed);
        bool Control_posz_velxy(float vx,float vy,float z) ;
	bool Control_posz_accxy(float ax,float ay,float z) ;
	bool Control_lat_lon_alt(float lat,float lon,float alt) ;
	bool Control_mc_to_fw();
	bool Control_fw_to_mc();
	bool hold_curr_pos(float time_s) ;
	void SetYaw(float new_yaw_rad);
	void Pubctl() ;
	static control_instance* getInstance()
	{
	if(instance==nullptr)
	{
	instance = new control_instance();
	}
	return instance;
	}
	struct swarm_point
{
 float x; /*<  */
 float y; /*<  */
 float z; /*<  */
 float r; /*<  */
 float g; /*<  */
 float b; /*<  */
 uint16_t seq; /*<  */
};
swarm_point points[250];
bool start_swarm=false;
private:
    float_t xy_rad=1;
    float_t fw_xy_rad=50;
    float_t z_rad=1;
    float_t fw_z_rad=50;
    float_t yaw_rad=0.1;
    bool time_flag=false;
static control_instance* instance;
control_instance(){};


 vehicle_local_position_s _vehicle_local_position;
 vtol_vehicle_status_s _vtol_vehicle_status;
    vehicle_status_s _status;
     uint64_t time_tick=hrt_absolute_time();
    vehicle_command_s _command = {};
    offboard_control_mode_s ocm{};
    position_setpoint_triplet_s _pos_sp_triplet{};
    vehicle_command_ack_s _ack{};
    vehicle_local_position_setpoint_s sp_local{};

    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
uORB::Publication<vehicle_attitude_setpoint_s>      _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
     uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
       uORB::Publication<vehicle_command_s>		_vehicle_command_pub{ORB_ID(vehicle_command)};
       uORB::Publication<position_setpoint_triplet_s>		_position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
       uORB::Publication<vehicle_local_position_setpoint_s>	_trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
};

