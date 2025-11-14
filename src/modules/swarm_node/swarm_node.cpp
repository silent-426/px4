#include "swarm_node.h"
#include <cmath>
#include <limits>

Swarm_Node::Swarm_Node() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::swarm_node),
    POINT_STATE(point0)  // 所有无人机初始状态统一为第1点
{
}

Swarm_Node::~Swarm_Node()
{
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool Swarm_Node::init()
{
    ScheduleOnInterval(20000_us); // 20ms 间隔，所有无人机同步控制频率
    PX4_INFO("Swarm_Node initialized (id=%d)", vehicle_id);
    return true;
}

bool Swarm_Node::takeoff()
{
    bool ret = control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 5);
    PX4_INFO("takeoff() ret=%d (id=%d)", (int)ret, vehicle_id);
    return ret;
}

bool Swarm_Node::arm_offboard()
{
    bool ok = control_instance::getInstance()->Change_offborad();
    ok = ok && control_instance::getInstance()->Arm_vehicle();
    PX4_INFO("arm_offboard() ret=%d (id=%d)", (int)ok, vehicle_id);
    return ok;
}

bool Swarm_Node::swarm_node_init()
{
    if (!_vehicle_local_position_sub.copy(&_vehicle_local_position)) {
        PX4_WARN("no local_position (id=%d)", vehicle_id);
        return false;
    }

    _a01_sub.copy(&_target);
    _a02_sub.copy(&_start_flag);

    if (!_start_flag.start_swarm) {
        return false;
    }

    vehicle_status_s vehicle_status{};
    if (!_vehicle_status_sub.copy(&vehicle_status)) {
        PX4_WARN("no vehicle_status (id=%d)", vehicle_id);
        return false;
    }
    vehicle_id = vehicle_status.system_id;
    PX4_INFO("vehicle_id=%d 初始化完成，悬停点作为第1点", vehicle_id);

    // 记录各自悬停点（第1点和第7点）
    begin_x = _vehicle_local_position.x;
    begin_y = _vehicle_local_position.y;
    begin_z = _vehicle_local_position.z;
    PX4_INFO("id=%d 悬停点坐标: (%.2f, %.2f, %.2f)", 
             vehicle_id, (double)begin_x, (double)begin_y, (double)begin_z);

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
    PX4_INFO("id=%d 到目标距离=%.2f m", vehicle_id, (double)dist);

    return true;
}

void Swarm_Node::start_swarm_node()
{
    // 更新订阅数据（所有无人机同步更新）
    _vehicle_local_position_sub.copy(&_vehicle_local_position);
    _a01_sub.copy(&_target);
    _a02_sub.copy(&_start_flag);

    // 保护 begin_z
    const float kZeps = 1e-3f;
    if (!PX4_ISFINITE(begin_z) || fabsf(begin_z) < kZeps) {
        begin_z = _vehicle_local_position.z;
    }

    // 处理 stop 命令（所有无人机统一响应）
    a02_s stop_msg{};
    _a02_sub.copy(&stop_msg);
    if (stop_msg.stop_swarm) {
        control_instance::getInstance()->Change_land();
        PX4_INFO("id=%d 收到停止命令，开始降落", vehicle_id);
        return;
    }

    // ====== 所有无人机共享7点轨迹逻辑 ======
    constexpr float HEX_RADIUS = 250.0f;         // 统一六边形大小
    constexpr float TRI_BASE_DIST = 20.0f;      // 三角形编队偏移（所有组共用）
    constexpr float GROUP_HEIGHT_DIFF = 15.0f;  // 高度差（不变）
    constexpr float GROUP_HORIZONTAL_OFFSET = 30.0f;  // 组间水平偏移

    // 6个顶点坐标（第1-6点），第7点=第1点（所有无人机共用此轨迹模板）
    const float hex_vertices[6][2] = {
        {HEX_RADIUS * cosf(0),           HEX_RADIUS * sinf(0)},           // 第1点（point0）
        {HEX_RADIUS * cosf(M_PI/3),      HEX_RADIUS * sinf(M_PI/3)},      // 第2点（point1）
        {HEX_RADIUS * cosf(2*M_PI/3),    HEX_RADIUS * sinf(2*M_PI/3)},    // 第3点（point2）
        {HEX_RADIUS * cosf(M_PI),        HEX_RADIUS * sinf(M_PI)},        // 第4点（point3）
        {HEX_RADIUS * cosf(4*M_PI/3),    HEX_RADIUS * sinf(4*M_PI/3)},    // 第5点（point4）
        {HEX_RADIUS * cosf(5*M_PI/3),    HEX_RADIUS * sinf(5*M_PI/3)}     // 第6点（point5）
    };

    // 计算当前无人机的轨迹中心（确保第1点=自身悬停点）
    const float hex_center_x = begin_x - hex_vertices[0][0];
    const float hex_center_y = begin_y - hex_vertices[0][1];

    // 三角形编队偏移（所有组共用，确保编队相对中心位置不变）
    const float tri_off[3][2] = {
        {0.0f, TRI_BASE_DIST},                              // 组内1号成员（原2号/5号机）
        {-TRI_BASE_DIST * 0.86602540378f, -TRI_BASE_DIST/2}, // 组内2号成员（原3号/6号机）
        {TRI_BASE_DIST * 0.86602540378f, -TRI_BASE_DIST/2}   // 组内3号成员（原4号/7号机）
    };

    // 【核心修改：去掉1号机，编号前移，两组各3架】
    int id = (int)vehicle_id;
    int group = -1; 
    int member_idx = 0;
    bool is_multicopter = (id >= 1 && id <= 3);  // A组（原2-4号→新1-3号，四旋翼）
    bool is_vtol = (id >= 4 && id <= 6);         // B组（原5-7号→新4-6号，VTOL）

    if (is_multicopter) {
        group = 0; // A组（四旋翼）
        member_idx = id - 1;  // 新1号→0，新2号→1，新3号→2（对应tri_off的3个顶点）
    } else if (is_vtol) {
        group = 1; // B组（VTOL）
        member_idx = id - 4;  // 新4号→0，新5号→1，新6号→2（对应tri_off的3个顶点）
    } else if (id > 6) {  // 扩展ID逻辑同步前移
        int rem = (id - 1) % 6;  // 原(id-2)%6 → 新(id-1)%6（因去掉1号机）
        group = (rem < 3) ? 0 : 1;
        member_idx = (rem < 3) ? rem : (rem - 3);
        is_multicopter = (group == 0);
        is_vtol = (group == 1);
    } else {
        PX4_WARN("id=%d 无效ID，保持当前位置", id);
        return;
    }

    // 计算组中心（所有组同步跟随7点轨迹）
    float group_center_x = hex_center_x;
    float group_center_y = hex_center_y;

    // 所有组共用同一套7点轨迹切换逻辑（不变）
    switch (POINT_STATE) {
    case point0:  // 第1点（悬停点）
        group_center_x = hex_center_x + hex_vertices[0][0];
        group_center_y = hex_center_y + hex_vertices[0][1];
        break;
    case point1:  // 第2点
        group_center_x = hex_center_x + hex_vertices[1][0];
        group_center_y = hex_center_y + hex_vertices[1][1];
        break;
    case point2:  // 第3点
        group_center_x = hex_center_x + hex_vertices[2][0];
        group_center_y = hex_center_y + hex_vertices[2][1];
        break;
    case point3:  // 第4点
        group_center_x = hex_center_x + hex_vertices[3][0];
        group_center_y = hex_center_y + hex_vertices[3][1];
        break;
    case point4:  // 第5点
        group_center_x = hex_center_x + hex_vertices[4][0];
        group_center_y = hex_center_y + hex_vertices[4][1];
        break;
    case point5:  // 第6点
        group_center_x = hex_center_x + hex_vertices[5][0];
        group_center_y = hex_center_y + hex_vertices[5][1];
        break;
    case point6:  // 第7点（与第1点重合）
        group_center_x = hex_center_x + hex_vertices[0][0];
        group_center_y = hex_center_y + hex_vertices[0][1];
        break;
    case land:
        if (control_instance::getInstance()->Change_land()) {
            POINT_STATE = end;
            a02_s _a02{};
            _a02.stop_swarm = true;
            _a02_pub.publish(_a02);
        }
        PX4_INFO("id=%d 进入降落状态", vehicle_id);
        return;
    case end:
        {
            a02_s _a02{};
            _a02.stop_swarm = true;
            _a02_pub.publish(_a02);
        }
        PX4_INFO("id=%d 任务结束", vehicle_id);
        return;
    default:
        break;
    }

    // B组中心相对于A组偏移（但跟随同一轨迹点，不变）
    float group_b_center_x = group_center_x;
    float group_b_center_y = group_center_y - GROUP_HORIZONTAL_OFFSET;

    // 计算目标位置（去掉1号机逻辑，仅保留两组三角形）
    float goal_x = _vehicle_local_position.x;
    float goal_y = _vehicle_local_position.y;
    float goal_z = _vehicle_local_position.z;
    const float base_z = begin_z - 50.0f; // 基准高度（不变）

    if (group == 0) {
        // A组（新1-3号）：基于A组中心的三角形偏移
        goal_x = group_center_x + tri_off[member_idx][0];
        goal_y = group_center_y + tri_off[member_idx][1];
        goal_z = base_z;

    } else if (group == 1) {
        // B组（新4-6号）：基于B组中心的三角形偏移
        goal_x = group_b_center_x + tri_off[member_idx][0];
        goal_y = group_b_center_y + tri_off[member_idx][1];
        goal_z = base_z - GROUP_HEIGHT_DIFF;

    } else {
        PX4_WARN("id=%d 无效ID，保持当前位置", id);
        return;
    }

    // 调试信息：更新分组描述（去掉1号机）
    PX4_INFO("id=%d 类型=%s 第%d点目标: (%.2f, %.2f, %.2f)",
             vehicle_id,
             is_multicopter ? "A组(四旋翼)" : "B组(VTOL)",
             POINT_STATE + 1,
             (double)goal_x, (double)goal_y, (double)goal_z);

    // 位置控制（所有无人机使用相同的到达判断逻辑，不变）
    bool reached = control_instance::getInstance()->Control_posxyz(goal_x, goal_y, goal_z);

    // 所有无人机同步切换轨迹点（不变）
    if (reached) {
        switch (POINT_STATE) {
        case point0: POINT_STATE = point1; break;
        case point1: POINT_STATE = point2; break;
        case point2: POINT_STATE = point3; break;
        case point3: POINT_STATE = point4; break;
        case point4: POINT_STATE = point5; break;
        case point5: POINT_STATE = point6; break;
        case point6: POINT_STATE = land; break;
        default: break;
        }
        PX4_INFO("id=%d 已到达第%d点，切换到第%d点",
                 vehicle_id, POINT_STATE, POINT_STATE + 1);
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

    // 所有无人机状态机流程统一（调整VTOL组ID范围）
    switch(STATE)
    {
    case state::INIT:
        if(swarm_node_init()) {
            STATE=state::ARM_OFFBOARD;
            PX4_INFO("id=%d 初始化完成，进入解锁模式", vehicle_id);
        }
        break;
    case state::ARM_OFFBOARD:
        if(control_instance::getInstance()->Change_offborad() && 
           control_instance::getInstance()->Arm_vehicle()) {
            STATE=state::TAKEOFF;
            PX4_INFO("id=%d 已解锁并进入offboard模式，准备起飞", vehicle_id);
        }
        break;
    case state::TAKEOFF:
        // 所有无人机起飞到自身悬停点（第1点）
        if(control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 50)) {
            PX4_INFO("id=%d 已到达悬停点（第1点）", vehicle_id);
            if (vehicle_id >= 4 && vehicle_id <= 6) {  // VTOL组ID范围更新为4-6
                STATE=state::MC_TO_FW;
            } else {
                STATE=state::CONTROL;
            }
        }
        break;
    case state::MC_TO_FW:
        if (vehicle_id >= 4 && vehicle_id <= 6) {  // VTOL组ID范围更新为4-6
            if(control_instance::getInstance()->Control_mc_to_fw()) {
                STATE=state::CONTROL;
                PX4_INFO("id=%d 已切换到固定翼模式，开始轨迹飞行", vehicle_id);
            }
        } else {
            STATE=state::CONTROL;
        }
        break;
    case state::CONTROL:
        start_swarm_node(); // 所有无人机进入7点轨迹飞行
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
    PRINT_MODULE_DESCRIPTION("Swarm control module (no id=1, 6-drone version)");
    PRINT_MODULE_USAGE_NAME("swarm_node", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    return 0;
}

extern "C" __EXPORT int swarm_node_main(int argc, char *argv[])
{
    return Swarm_Node::main(argc, argv);
}
