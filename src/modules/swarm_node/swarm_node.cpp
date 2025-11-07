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

    // ====== 编队与分组设置 ======
    constexpr float SQUARE_HALF = 250.0f;       // 正方形半边长
    constexpr float TRI_BASE_DIST = 20.0f;      // 三角形顶点相对组中心的基准距离
    constexpr float GROUP_HEIGHT_DIFF = 15.0f;  // A/B组高度差（B组低15m）
    constexpr float DRONE1_HEIGHT_OFFSET = 15.0f; // 1号机比2号机高15m
    constexpr float GROUP_HORIZONTAL_OFFSET = 30.0f; // A/B组水平前后距离

    // 三角形相对偏移
    const float tri_off[3][2] = {
        {0.0f, TRI_BASE_DIST},                              // 前方顶点（2号机）
        {-TRI_BASE_DIST * 0.86602540378f, -TRI_BASE_DIST/2}, // 左后（3号机）
        {TRI_BASE_DIST * 0.86602540378f, -TRI_BASE_DIST/2}   // 右后（4号机）
    };

    // 分组逻辑
    int id = (int)vehicle_id;
    int group = -1; // -1：1号机；0：A组四旋翼（2-4）；1：B组VTOL（5-7）
    int member_idx = 0;
    bool is_drone1 = (id == 1);
    bool is_multicopter = (id >= 1 && id <= 4);  // 1-4号为四旋翼
    bool is_vtol = (id >= 5 && id <= 7);         // 5-7号为VTOL

    if (id >= 2 && id <= 4) {
        group = 0; // A组四旋翼（2-4）
        member_idx = id - 2;
    } else if (id >= 5 && id <= 7) {
        group = 1; // B组VTOL（5-7）
        member_idx = id - 5;
    } else if (id > 7) {
        // 超出7号的ID循环映射
        int rem = (id - 2) % 6;
        group = (rem < 3) ? 0 : 1;
        member_idx = (rem < 3) ? rem : (rem - 3);
        is_multicopter = (group == 0);
        is_vtol = (group == 1);
    }

    // 计算A组中心（正方形轨迹）
    float group_a_center_x = begin_x;
    float group_a_center_y = begin_y;

    switch (POINT_STATE) {
    case point_state::point0:
        group_a_center_x = begin_x + 2*SQUARE_HALF;
        group_a_center_y = begin_y;
        break;
    case point_state::point1:
        group_a_center_x = begin_x + 2*SQUARE_HALF;
        group_a_center_y = begin_y - 2*SQUARE_HALF;
        break;
    case point_state::point2:
        group_a_center_x = begin_x;
        group_a_center_y = begin_y - 2*SQUARE_HALF;
        break;
    case point_state::point3:
        group_a_center_x = begin_x;
        group_a_center_y = begin_y;
        break;
    case point_state::land:
        if (control_instance::getInstance()->Change_land()) {
            POINT_STATE = point_state::end;
            a02_s _a02{};
            _a02.stop_swarm = true;
            _a02_pub.publish(_a02);
        }
        PX4_INFO("id=%d in LAND state", vehicle_id);
        return;
    case point_state::end:
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

    // 计算B组中心（A组前方）
    float group_b_center_x = group_a_center_x;
    float group_b_center_y = group_a_center_y - GROUP_HORIZONTAL_OFFSET;

    // 初始化目标位置（默认值，避免未初始化）
    float goal_x = _vehicle_local_position.x; // 当前位置（安全默认值）
    float goal_y = _vehicle_local_position.y;
    float goal_z = _vehicle_local_position.z;
    const float base_z = begin_z - 50.0f; // A组基准高度

    // 根据分组和机型计算目标位置
    if (is_drone1) {
        // 1号机（四旋翼）：2号机正上方15m
        float drone2_x = group_a_center_x + tri_off[0][0];
        float drone2_y = group_a_center_y + tri_off[0][1];
        goal_x = drone2_x;
        goal_y = drone2_y;
        goal_z = base_z + DRONE1_HEIGHT_OFFSET;

    } else if (group == 0) {
        // A组四旋翼（2-4号）
        goal_x = group_a_center_x + tri_off[member_idx][0];
        goal_y = group_a_center_y + tri_off[member_idx][1];
        goal_z = base_z;

    } else if (group == 1) {
        // B组VTOL（5-7号）
        goal_x = group_b_center_x + tri_off[member_idx][0];
        goal_y = group_b_center_y + tri_off[member_idx][1];
        goal_z = base_z - GROUP_HEIGHT_DIFF;

    } else {
        // 异常ID处理（如id < 1）
        PX4_WARN("Invalid vehicle ID: %d, maintaining current position", id);
        return;
    }

    // 调试信息
    if (is_drone1) {
        PX4_INFO("DRONE1(Quad) id=%d 位置: (%.2f, %.2f, %.2f) [2号机上方15m]",
                 vehicle_id, (double)goal_x, (double)goal_y, (double)goal_z);
    } else if (is_multicopter) {
        PX4_INFO("GROUP_A(Quad) id=%d (成员%d) 位置: (%.2f, %.2f, %.2f)",
                 vehicle_id, member_idx, (double)goal_x, (double)goal_y, (double)goal_z);
    } else if (is_vtol) {
        PX4_INFO("GROUP_B(VTOL) id=%d (成员%d) 位置: (%.2f, %.2f, %.2f)",
                 vehicle_id, member_idx, (double)goal_x, (double)goal_y, (double)goal_z);
    }

    // 位置控制
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
        PX4_INFO("id=%d 到达目标 -> 下一状态=%d", vehicle_id, (int)POINT_STATE);
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
            // 根据机型选择下一状态
            if (vehicle_id >= 5 && vehicle_id <= 7) {
                // VTOL需要转换到固定翼模式
                STATE=state::MC_TO_FW;
            } else {
                // 四旋翼直接进入控制状态
                STATE=state::CONTROL;
            }
        }
        break;
    case state::MC_TO_FW:
        // 只有VTOL（5-7号）执行模式转换
        if (vehicle_id >= 5 && vehicle_id <= 7) {
            if(control_instance::getInstance()->Control_mc_to_fw()) {
                STATE=state::CONTROL;
            }
        } else {
            // 四旋翼跳过此状态
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
