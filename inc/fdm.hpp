#pragma once

#include <array>
#include <cstdint>

// 6DOF 状态（double 精度积分，float 输出给传感器）。
// 坐标约定：
//   机体系（body）：前右下（FRD），与 INAV 机体约定一致。
//   地理系（world)：北东地（NED），速度 (vn, ve, vd)，vd>0 为下降。
//   姿态：roll=横滚（右压杆为正），pitch=俯仰（抬头为正），yaw=偏航（0=正北，顺时针为正）。
struct FdmState
{
    double lat_deg = 30.0;
    double lon_deg = 120.0;
    double alt_m = 10.0; // AMSL
    double vn = 0, ve = 0, vd = 0;
    double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    double p_dps = 0, q_dps = 0, r_dps = 0; // 机体系角速度（FRD）
};

// 注入 SITL 的自洽传感器快照（X-Plane DREF 语义，见 xplane_link）。
struct SensorSnapshot
{
    double lat_deg = 0, lon_deg = 0;
    float alt_m = 0; // elevation（AMSL 米）
    float agl_m = 0; // y_agl（离地米）
    float local_vx = 0; // X-Plane 本地系：东（米/秒）
    float local_vy = 0; // X-Plane 本地系：上为正（米/秒）
    float local_vz = 0; // X-Plane 本地系：南（米/秒）
    float groundspeed = 0; // 地速（米/秒）
    float airspeed = 0; // 空速（多旋翼≈地速+垂直分量）
    float phi_deg = 0; // 横滚
    float theta_deg = 0; // 俯仰（抬头为正，X-Plane theta 语义）
    float psi_deg = 0; // 真航向
    float hpath_deg = 0; // 地面航迹
    float g_axil = 0; // 过载-前向（g）
    float g_side = 0; // 过载-侧向（g）
    float g_nrml = 1; // 过载-法向（g，静止=1）
    float gyro_p_dps = 0; // P（度/秒）
    float gyro_q_dps = 0; // Q（度/秒）
    float gyro_r_dps = 0; // R（度/秒）
    float baro_inhg = 29.92f;
    float mag_x = 1, mag_y = 0, mag_z = 0; // 机体归一化地磁
    // XITL 扩展（fix_type 用 INAV 内部枚举：0=无，1=2D，2=3D；不是 MAVLink 枚举！）
    int num_sats = 16;
    int fix_type = 2;
    int32_t rangefinder_cm = -1;
    float batt_v = 16.8f, batt_a = 0;
    uint16_t rssi = 90;
    bool failsafe = false;
};

// 轻量四旋翼 Newton-Euler 刚体模型（不含完整气动、旋翼动态或地面起落架模型）。
struct FdmParams
{
    double mass_kg = 1.2;
    double max_thrust_n = 30.0;
    double ground_alt_m = 10.0;
    // 转动惯量与力臂
    double ixx = 0.02, iyy = 0.02, izz = 0.04;
    double arm_m = 0.11;
    double yaw_k = 0.02;
    // 电机一阶惯性时间常数；0 表示瞬时响应
    double motor_tau_s = 0.03;
    // 推力曲线：T = Tmax * throttle^thrust_exponent
    double thrust_exponent = 1.0;
    // 气动阻尼：线性项 1/s，二次项 1/m
    double drag_linear = 0.25;
    double drag_quadratic = 0.0;
    // 常值风（NED，m/s，向下为正）
    double wind_n = 0.0, wind_e = 0.0, wind_d = 0.0;
    // 地面水平摩擦时间常数，1/s；0 表示不额外阻尼
    double ground_friction = 5.0;
};

class QuadFdm
{
public:
    explicit QuadFdm(const FdmParams &params);
    QuadFdm(double mass_kg, double max_thrust_n, double ground_alt_m);

    void reset(double lat, double lon, double alt, double yaw_deg = 0);
    // 输⼊：4 路电机归一化油门 0..1（M1..M4，X 构型：M1 右前、M2 左前、M3 左后、M4 右后）。
    // 输出：推进 dt 秒后的自洽传感器快照。
    SensorSnapshot step(const std::array<double, 4> &motors, double dt);

    const FdmState &state() const
    {
        return st_;
    }
    double avg_throttle() const
    {
        return avg_thr_;
    }

private:
    FdmState st_{};
    FdmParams params_{};
    std::array<double, 4> q_body_to_ned_{ 1, 0, 0, 0 };
    std::array<double, 4> motor_state_{ 0, 0, 0, 0 };
    double avg_thr_ = 0;
};
