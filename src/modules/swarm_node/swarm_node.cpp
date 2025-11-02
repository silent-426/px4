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

    // ====== Formation and grouping settings ======
    // Vehicles 1-3: group A (triangle)
    // Vehicles 4-6: group B (triangle), group B is HEIGHT_DIFF meters below group A
    // Both groups follow the same square of centers (正方形)，每个顶点为 group center

    constexpr float SQUARE_HALF = 250.0f; // 正方形半边长（原代码使用 500 偏移）
    constexpr float TRI_BASE_DIST = 20.0f; // 三角形顶点相对中心的基准距离（可按需调整）
    constexpr float HEIGHT_DIFF = 10.0f;  // 两组高度差（米）

    // 预设三角形相对偏移（相对于组中心）——等边三角形的三个顶点
    // 顶点间距约为 2*TRI_BASE_DIST
    const float tri_off[3][2] = {
        {0.0f, TRI_BASE_DIST},                              // 前方顶点
        {-TRI_BASE_DIST * 0.86602540378f, -TRI_BASE_DIST/2}, // 左后
        {TRI_BASE_DIST * 0.86602540378f, -TRI_BASE_DIST/2}   // 右后
    };

    // 计算组别与组内索引（保证在 0..2）
    int id = (int)vehicle_id; // 原始 id
    int group = 0; // 0 -> A (1-3), 1 -> B (4-6)
    int member_idx = 0; // 0/1/2

    if (id >= 1 && id <= 3) {
        group = 0; // A
        member_idx = id - 1;
    } else if (id >= 4 && id <= 6) {
        group = 1; // B
        member_idx = id - 4;
    } else {
        // 如果 ID 超出 1..6，使用循环映射
        int rem = (id - 1) % 6; // 0..5
        if (rem < 3) {
            group = 0;
            member_idx = rem;
        } else {
            group = 1;
            member_idx = rem - 3;
        }
    }

    // base group center（不加任何偏移）
    float group_center_x = begin_x;
    float group_center_y = begin_y;

    // 每个 POINT_STATE 对应正方形的一个中心点（四个顶点），保持与原逻辑相同的 500 米 边长
    switch (POINT_STATE) {
    case point_state::point0:
        group_center_x = begin_x + 2*SQUARE_HALF;
        group_center_y = begin_y;
        break;
    case point_state::point1:
        group_center_x = begin_x + 2*SQUARE_HALF;
        group_center_y = begin_y - 2*SQUARE_HALF;
        break;
    case point_state::point2:
        group_center_x = begin_x;
        group_center_y = begin_y - 2*SQUARE_HALF;
        break;
    case point_state::point3:
        group_center_x = begin_x;
        group_center_y = begin_y;
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

    // 如果是组 B，则相对于组 A 下降 HEIGHT_DIFF 米
    float goal_z = (group == 0) ? (begin_z - 50.0f) : (begin_z - 50.0f - HEIGHT_DIFF);

    // 将组中心（group_center_x/y）作为中心，再加上组内三角形偏移
    float goal_x = group_center_x + tri_off[member_idx][0];
    float goal_y = group_center_y + tri_off[member_idx][1];

    // 调试：打印本次目标
    PX4_INFO("SWARM_TRIANGLE id=%d group=%d member=%d POINT_STATE=%d goal=(%.2f, %.2f, %.2f)",
             vehicle_id, group, member_idx, (int)POINT_STATE, (double)goal_x, (double)goal_y, (double)goal_z);

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
        PX4_INFO("SWARM_TRIANGLE id=%d reached -> next POINT_STATE=%d", vehicle_id, (int)POINT_STATE);
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
