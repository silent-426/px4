#include "swarm_node.h"
#include <cmath>
#include <limits>

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
    ScheduleOnInterval(20000_us); // 20ms 间隔
    PX4_INFO("Swarm_Node initialized");
    return true;
}

bool Swarm_Node::takeoff()
{
    bool ret = control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 5);
    PX4_INFO("takeoff() ret=%d", (int)ret);
    return ret;
}

bool Swarm_Node::arm_offboard()
{
    bool ok = control_instance::getInstance()->Change_offborad();
    ok = ok && control_instance::getInstance()->Arm_vehicle();
    PX4_INFO("arm_offboard() ret=%d", (int)ok);
    return ok;
}

bool Swarm_Node::swarm_node_init()
{
    if (!_vehicle_local_position_sub.copy(&_vehicle_local_position)) {
        PX4_WARN("no local_position");
        return false;
    }

    _a01_sub.copy(&_target);
    _a02_sub.copy(&_start_flag);

    if (!_start_flag.start_swarm) {
        return false;
    }

    vehicle_status_s vehicle_status{};
    if (!_vehicle_status_sub.copy(&vehicle_status)) {
        PX4_WARN("no vehicle_status");
        return false;
    }
    vehicle_id = vehicle_status.system_id;
    PX4_INFO("vehicle_id=%d", vehicle_id);

    begin_x = _vehicle_local_position.x;
    begin_y = _vehicle_local_position.y;
    begin_z = _vehicle_local_position.z;

    _global_local_proj_ref.initReference(_vehicle_local_position.ref_lat,
                                         _vehicle_local_position.ref_lon,
                                         hrt_absolute_time());

    const double lat = _target.lat;
    const double lon = _target.lon;
    const double eps = 1e-6;

    if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && (fabs(lat) > eps || fabs(lon) > eps)) {
        _global_local_proj_ref.project(lat, lon, target_x, target_y);
    } else {
        target_x = begin_x;
        target_y = begin_y;
    }

    float dx = _vehicle_local_position.x - target_x;
    float dy = _vehicle_local_position.y - target_y;
    float dist = sqrtf(dx*dx + dy*dy);
    PX4_INFO("dist_to_target=%.2f m", (double)dist);

    if (vehicle_id > 1 && dist > 200.0f) {
        PX4_WARN("vehicle %d far from target (%.1fm)", vehicle_id, (double)dist);
    }

    return true;
}

void Swarm_Node::start_swarm_node()
{
    // 更新订阅数据
    _vehicle_local_position_sub.copy(&_vehicle_local_position);
    _a01_sub.copy(&_target);
    _a02_sub.copy(&_start_flag);

    // 保护 begin_z（避免与 0.0f 直接比较）
    const float kZeps = 1e-3f;
    if (!PX4_ISFINITE(begin_z) || fabsf(begin_z) < kZeps) {
        begin_z = _vehicle_local_position.z;
    }

    // 处理 stop 命令
    a02_s stop_msg{};
    _a02_sub.copy(&stop_msg);
    if (stop_msg.stop_swarm) {
        control_instance::getInstance()->Change_land();
        PX4_INFO("stop_swarm received id=%d", vehicle_id);
        return;
    }

    // 统一的四边形顶点（与原 leader 的顶点相同）
    // base goal（未加任何偏移）
    float goal_x = begin_x;
    float goal_y = begin_y;
    const float goal_z = begin_z - 50.0f;

    switch (POINT_STATE) {
    case point_state::point0:
        goal_x = begin_x + 500.0f;
        goal_y = begin_y;
        break;
    case point_state::point1:
        goal_x = begin_x + 500.0f;
        goal_y = begin_y - 500.0f;
        break;
    case point_state::point2:
        goal_x = begin_x;
        goal_y = begin_y - 500.0f;
        break;
    case point_state::point3:
        goal_x = begin_x;
        goal_y = begin_y;
        break;
    case point_state::land:
        // land 状态下调用降落命令并发布 stop 标志（与原逻辑一致）
        if (control_instance::getInstance()->Change_land()) {
            POINT_STATE = point_state::end;
            a02_s _a02{};
            _a02.stop_swarm = true;
            _a02_pub.publish(_a02);
        }
        PX4_INFO("id=%d in LAND state", vehicle_id);
        return;
    case point_state::end:
        // 已结束，保持 stop 发布（幂等）
        {
            a02_s _a02{};
            _a02.stop_swarm = true;
            _a02_pub.publish(_a02);
        }
        PX4_INFO("id=%d in END state", vehicle_id);
        return;
    default:
        break;
    }

    // 调试：打印本次目标
    PX4_INFO("SWARM_UNIFORM id=%d POINT_STATE=%d goal=(%.2f, %.2f, %.2f)",
             vehicle_id, (int)POINT_STATE, (double)goal_x, (double)goal_y, (double)goal_z);

    // 调用控制器，若返回 true 表示到达该顶点 -> 推进状态（与 leader 行为一致）
    bool reached = control_instance::getInstance()->Control_posxyz(goal_x, goal_y, goal_z);

    if (reached) {
        switch (POINT_STATE) {
        case point_state::point0:
            POINT_STATE = point_state::point1;
            break;
        case point_state::point1:
            POINT_STATE = point_state::point2;
            break;
        case point_state::point2:
            POINT_STATE = point_state::point3;
            break;
        case point_state::point3:
            POINT_STATE = point_state::land;
            break;
        default:
            break;
        }
        PX4_INFO("SWARM_UNIFORM id=%d reached -> next POINT_STATE=%d", vehicle_id, (int)POINT_STATE);
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

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	switch(STATE)
	{
	case state::INIT:
		if(swarm_node_init()) {
			STATE=state::ARM_OFFBOARD;
		}
		break;
	case state::ARM_OFFBOARD:
		if(control_instance::getInstance()->Change_offborad() && 
		   control_instance::getInstance()->Arm_vehicle()) {
			STATE=state::TAKEOFF;
		}
		break;
	case state::TAKEOFF:
		if(control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 50)) {
			STATE=state::MC_TO_FW;
		}
		break;
	case state::MC_TO_FW:
		if(control_instance::getInstance()->Control_mc_to_fw()) {
			STATE=state::CONTROL;
		}
		break;
	case state::CONTROL:
		start_swarm_node();
		break;
	case state::LAND:
		control_instance::getInstance()->Change_land();
		break;
	default:
		break;
	}

	perf_end(_loop_perf);
}

int Swarm_Node::task_spawn(int argc, char *argv[])
{
	Swarm_Node *instance = new Swarm_Node();
	if (instance && instance->init()) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		return PX4_OK;
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
	if (reason) PX4_WARN("%s", reason);
	PRINT_MODULE_DESCRIPTION("Swarm control module");
	PRINT_MODULE_USAGE_NAME("swarm_node", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	return 0;
}

extern "C" __EXPORT int swarm_node_main(int argc, char *argv[])
{
	return Swarm_Node::main(argc, argv);
}
