#include "fdm.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;
constexpr double kEarthR = 6378137.0;

double clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// 气压高度 → 海平面气压英寸汞柱（fl2sitl to_hg 同款公式）。
float alt_to_inhg(double alt_m)
{
    constexpr double k = 44330.0, p0 = 1013.25;
    const double base = (alt_m < k) ? (1.0 - alt_m / k) : 1e-6;
    const double p = p0 * std::pow(std::max(base, 1e-6), 5.255);
    return static_cast<float>(0.0295299837 * p);
}

std::array<double, 4> euler_to_quaternion(double roll_deg, double pitch_deg, double yaw_deg)
{
    const double r = roll_deg * kPi / 360.0, p = pitch_deg * kPi / 360.0, y = yaw_deg * kPi / 360.0;
    const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    return { cr * cp * cy + sr * sp * sy,
             sr * cp * cy - cr * sp * sy,
             cr * sp * cy + sr * cp * sy,
             cr * cp * sy - sr * sp * cy };
}

void normalize_quaternion(std::array<double, 4> &q)
{
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm < 1e-12)
    {
        q = { 1, 0, 0, 0 };
        return;
    }
    for (double &value : q) value /= norm;
}

std::array<double, 9> body_to_ned(const std::array<double, 4> &q)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return { 1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
             2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
             2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y) };
}

std::array<double, 3> quaternion_to_euler(const std::array<double, 4> &q)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const double sin_pitch = clamp(2 * (w * y - z * x), -1, 1);
    const double pitch = std::asin(sin_pitch);
    const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return { roll * 180.0 / kPi, pitch * 180.0 / kPi, yaw * 180.0 / kPi };
}
} // namespace

QuadFdm::QuadFdm(const FdmParams &params)
: params_(params)
{
}

QuadFdm::QuadFdm(double mass_kg, double max_thrust_n, double ground_alt_m)
{
    params_.mass_kg = mass_kg;
    params_.max_thrust_n = max_thrust_n;
    params_.ground_alt_m = ground_alt_m;
}

void QuadFdm::reset(double lat, double lon, double alt, double yaw_deg)
{
    st_ = FdmState{};
    st_.lat_deg = lat;
    st_.lon_deg = lon;
    st_.alt_m = alt;
    st_.yaw_deg = yaw_deg;
    q_body_to_ned_ = euler_to_quaternion(0, 0, yaw_deg);
    motor_state_.fill(0.0);
    avg_thr_ = 0;
}

SensorSnapshot QuadFdm::step(const std::array<double, 4> &motors, double dt)
{
    if (!std::isfinite(dt) || dt <= 0.0) dt = 1e-3;
    dt = clamp(dt, 1e-4, 0.1);

    // 1) 电机一阶惯性 + 推力曲线。电机命令不再瞬间变成推力。
    const double alpha = (params_.motor_tau_s > 0.0) ? (1.0 - std::exp(-dt / params_.motor_tau_s)) : 1.0;
    for (size_t i = 0; i < motor_state_.size(); ++i)
    {
        const double cmd = clamp(motors[i], 0.0, 1.0);
        motor_state_[i] += (cmd - motor_state_[i]) * alpha;
    }
    const double m0 = motor_state_[0], m1 = motor_state_[1];
    const double m2 = motor_state_[2], m3 = motor_state_[3];
    avg_thr_ = (m0 + m1 + m2 + m3) * 0.25;

    const double per_motor_max = params_.max_thrust_n * 0.25;
    const double t0 = per_motor_max * std::pow(m0, params_.thrust_exponent);
    const double t1 = per_motor_max * std::pow(m1, params_.thrust_exponent);
    const double t2 = per_motor_max * std::pow(m2, params_.thrust_exponent);
    const double t3 = per_motor_max * std::pow(m3, params_.thrust_exponent);
    const double thrust = t0 + t1 + t2 + t3;
    const double tau_x = (t1 + t2 - t0 - t3) * params_.arm_m * 0.7071; // 横滚
    const double tau_y = (t0 + t1 - t2 - t3) * params_.arm_m * 0.7071; // 俯仰
    const double tau_z = (-t0 + t1 - t2 + t3) * params_.yaw_k * params_.arm_m * 10.0;

    const auto rot = body_to_ned(q_body_to_ned_);
    const double r11 = rot[0], r12 = rot[1], r13 = rot[2];
    const double r21 = rot[3], r22 = rot[4], r23 = rot[5];
    const double r31 = rot[6], r32 = rot[7], r33 = rot[8];
    // 机体推力轴 (0,0,-thrust) 转到 NED：
    const double t_n = r13 * -thrust;
    const double t_e = r23 * -thrust;
    const double t_d = r33 * -thrust;

    // 2) 平动：线性 + 二次气动阻力，阻力相对空气速度（常值风）。
    const double vn_air = st_.vn - params_.wind_n;
    const double ve_air = st_.ve - params_.wind_e;
    const double vd_air = st_.vd - params_.wind_d;
    const double drag_n = params_.drag_linear * vn_air + params_.drag_quadratic * vn_air * std::abs(vn_air);
    const double drag_e = params_.drag_linear * ve_air + params_.drag_quadratic * ve_air * std::abs(ve_air);
    const double drag_d = 0.6 * (params_.drag_linear * vd_air + params_.drag_quadratic * vd_air * std::abs(vd_air));
    const double an = t_n / params_.mass_kg - drag_n;
    const double ae = t_e / params_.mass_kg - drag_e;
    const double ad = t_d / params_.mass_kg + kG - drag_d;
    st_.vn += an * dt;
    st_.ve += ae * dt;
    st_.vd += ad * dt;
    st_.lat_deg += (st_.vn * dt / kEarthR) * 180.0 / kPi;
    st_.lat_deg = clamp(st_.lat_deg, -89.999999, 89.999999);
    const double cos_lat = std::max(std::cos(st_.lat_deg * kPi / 180.0), 1e-6);
    st_.lon_deg += (st_.ve * dt / (kEarthR * cos_lat)) * 180.0 / kPi;
    st_.lon_deg = std::remainder(st_.lon_deg, 360.0);
    st_.alt_m += -st_.vd * dt;

    // 3) 地面约束：不穿地、不反弹，触地清零垂直速度并施加水平摩擦。
    const double agl_before = st_.alt_m - params_.ground_alt_m;
    if (agl_before <= 0.0)
    {
        st_.alt_m = params_.ground_alt_m;
        if (st_.vd > 0.0) st_.vd = 0.0;
        const double friction = clamp(1.0 - params_.ground_friction * dt, 0.0, 1.0);
        st_.vn *= friction;
        st_.ve *= friction;
    }
    const double agl = std::max(0.0, st_.alt_m - params_.ground_alt_m);

    // 4) 刚体角动量方程 + 角阻尼。
    const double p_rad = st_.p_dps * kPi / 180.0, q_rad = st_.q_dps * kPi / 180.0, r_rad = st_.r_dps * kPi / 180.0;
    const double p_dot = (tau_x + (params_.iyy - params_.izz) * q_rad * r_rad) / params_.ixx - 2.0 * p_rad;
    const double q_dot = (tau_y + (params_.izz - params_.ixx) * r_rad * p_rad) / params_.iyy - 2.0 * q_rad;
    const double r_dot = (tau_z + (params_.ixx - params_.iyy) * p_rad * q_rad) / params_.izz - r_rad;
    double p_new = p_rad + p_dot * dt;
    double q_new = q_rad + q_dot * dt;
    double r_new = r_rad + r_dot * dt;
    if (agl <= 0.01 && thrust < params_.mass_kg * kG * 1.05)
    {
        const double k = clamp(8.0 * dt, 0.0, 1.0);
        st_.roll_deg *= 1.0 - k;
        st_.pitch_deg *= 1.0 - k;
        p_new *= 1.0 - k;
        q_new *= 1.0 - k;
        q_body_to_ned_ = euler_to_quaternion(st_.roll_deg, st_.pitch_deg, st_.yaw_deg);
    }
    st_.p_dps = p_new * 180.0 / kPi;
    st_.q_dps = q_new * 180.0 / kPi;
    st_.r_dps = r_new * 180.0 / kPi;

    const double p_mid = 0.5 * (p_rad + p_new), q_mid = 0.5 * (q_rad + q_new);
    const double r_mid = 0.5 * (r_rad + r_new);
    const auto &a = q_body_to_ned_;
    q_body_to_ned_ = { a[0] - 0.5 * dt * (a[1] * p_mid + a[2] * q_mid + a[3] * r_mid),
                       a[1] + 0.5 * dt * (a[0] * p_mid + a[2] * r_mid - a[3] * q_mid),
                       a[2] + 0.5 * dt * (a[0] * q_mid + a[3] * p_mid - a[1] * r_mid),
                       a[3] + 0.5 * dt * (a[0] * r_mid + a[1] * q_mid - a[2] * p_mid) };
    normalize_quaternion(q_body_to_ned_);
    const auto euler = quaternion_to_euler(q_body_to_ned_);
    st_.roll_deg = euler[0];
    st_.pitch_deg = euler[1];
    st_.yaw_deg = euler[2];
    if (st_.yaw_deg < 0.0) st_.yaw_deg += 360.0;

    // 5) 自洽传感器合成
    SensorSnapshot s;
    s.lat_deg = st_.lat_deg;
    s.lon_deg = st_.lon_deg;
    s.alt_m = static_cast<float>(st_.alt_m);
    s.agl_m = static_cast<float>(agl);
    // X-Plane 本地系：local_vx=东，local_vy=上，local_vz=南。
    // SITL 用 gpsVelD=-local_vy，因此 NED 的 vd（下为正）需取反。
    s.local_vx = static_cast<float>(st_.ve);
    s.local_vy = static_cast<float>(-st_.vd);
    s.local_vz = static_cast<float>(-st_.vn);
    const float gs = static_cast<float>(std::hypot(st_.vn, st_.ve));
    s.groundspeed = gs;
    s.airspeed = static_cast<float>(std::hypot(gs, st_.vd));
    s.phi_deg = static_cast<float>(st_.roll_deg);
    s.theta_deg = static_cast<float>(st_.pitch_deg);
    s.psi_deg = static_cast<float>(st_.yaw_deg);
    s.hpath_deg = static_cast<float>(std::atan2(st_.ve, st_.vn) * 180.0 / kPi);
    if (s.hpath_deg < 0) s.hpath_deg += 360;
    // 过载 = 比力/g（机体系）：比力 = 真实加速度 - 重力矢量(0,0,+g)
    // 触地时位置被钳位（真实加速度≠动力学加速度），比力改用支持力模型：f_d=-g
    const bool on_ground = (agl <= 0.0);
    const double f_n = an, f_e = ae, f_d = on_ground ? -kG : ad - kG;
    const double fx = r11 * f_n + r21 * f_e + r31 * f_d;
    const double fy = r12 * f_n + r22 * f_e + r32 * f_d;
    const double fz = r13 * f_n + r23 * f_e + r33 * f_d;
    // INAV 侧 fakeAccSet(-ax, +ay, +az)；静止悬停 g_nrml=1
    s.g_axil = static_cast<float>(-fx / kG);
    s.g_side = static_cast<float>(fy / kG);
    s.g_nrml = static_cast<float>(-fz / kG);
    s.gyro_p_dps = static_cast<float>(st_.p_dps);
    s.gyro_q_dps = static_cast<float>(st_.q_dps);
    s.gyro_r_dps = static_cast<float>(st_.r_dps);
    s.baro_inhg = alt_to_inhg(st_.alt_m);
    // 地磁：世界系磁北 ≈ 真北（忽略磁偏），转机体
    s.mag_x = static_cast<float>(r11);
    s.mag_y = static_cast<float>(r12);
    s.mag_z = static_cast<float>(r13);
    s.rangefinder_cm = static_cast<int32_t>(agl * 100.0);
    return s;
}
