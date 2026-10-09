#include "fdm.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;
constexpr double kEarthR = 6378137.0;

double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// 气压高度 → 海平面气压英寸汞柱（fl2sitl to_hg 同款公式）。
float alt_to_inhg(double alt_m) {
    constexpr double k = 44330.0, p0 = 1013.25;
    double p = p0 * std::pow(1.0 - alt_m / k, 5.255);
    return static_cast<float>(0.0295299837 * p);
}

std::array<double, 4> euler_to_quaternion(double roll_deg, double pitch_deg, double yaw_deg) {
    const double r = roll_deg * kPi / 360.0, p = pitch_deg * kPi / 360.0, y = yaw_deg * kPi / 360.0;
    const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

void normalize_quaternion(std::array<double, 4>& q) {
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (double& value : q) value /= norm;
}

std::array<double, 9> body_to_ned(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
            2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
            2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
}

std::array<double, 3> quaternion_to_euler(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const double sin_pitch = clamp(2 * (w * y - z * x), -1, 1);
    const double pitch = std::asin(sin_pitch);
    const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return {roll * 180.0 / kPi, pitch * 180.0 / kPi, yaw * 180.0 / kPi};
}
}  // namespace

QuadFdm::QuadFdm(double mass_kg, double max_thrust_n, double ground_alt_m)
    : mass_(mass_kg), max_thrust_(max_thrust_n), ground_alt_(ground_alt_m) {}

void QuadFdm::reset(double lat, double lon, double alt, double yaw_deg) {
    st_ = FdmState{};
    st_.lat_deg = lat;
    st_.lon_deg = lon;
    st_.alt_m = alt;
    st_.yaw_deg = yaw_deg;
    q_body_to_ned_ = euler_to_quaternion(0, 0, yaw_deg);
    avg_thr_ = 0;
}

SensorSnapshot QuadFdm::step(const std::array<double, 4>& motors, double dt) {
    dt = clamp(dt, 1e-4, 0.1);
    // 1) 油门 → 推力/力矩（X 构型：M1 右前、M2 左前、M3 左后、M4 右后）
    const double m0 = clamp(motors[0], 0, 1), m1 = clamp(motors[1], 0, 1);
    const double m2 = clamp(motors[2], 0, 1), m3 = clamp(motors[3], 0, 1);
    avg_thr_ = (m0 + m1 + m2 + m3) * 0.25;
    const double t0 = m0 * max_thrust_ * 0.25, t1 = m1 * max_thrust_ * 0.25;
    const double t2 = m2 * max_thrust_ * 0.25, t3 = m3 * max_thrust_ * 0.25;
    const double thrust = t0 + t1 + t2 + t3;
    const double tau_x = (t1 + t2 - t0 - t3) * arm_m_ * 0.7071;  // 横滚
    const double tau_y = (t0 + t1 - t2 - t3) * arm_m_ * 0.7071;  // 俯仰
    const double tau_z = (-t0 + t1 - t2 + t3) * yaw_k_ * arm_m_ * 10.0;

    const auto rot = body_to_ned(q_body_to_ned_);
    const double r11 = rot[0], r12 = rot[1], r13 = rot[2];
    const double r21 = rot[3], r22 = rot[4], r23 = rot[5];
    const double r31 = rot[6], r32 = rot[7], r33 = rot[8];
    // 机体推力轴 (0,0,-thrust) 转到 NED：
    const double t_n = r13 * -thrust;
    const double t_e = r23 * -thrust;
    const double t_d = r33 * -thrust;

    // 3) 平动积分（含线性阻尼）
    const double drag = 0.25;  // 1/s
    const double an = t_n / mass_ - drag * st_.vn;
    const double ae = t_e / mass_ - drag * st_.ve;
    const double ad = t_d / mass_ + kG - drag * st_.vd * 0.6;
    st_.vn += an * dt;
    st_.ve += ae * dt;
    st_.vd += ad * dt;
    st_.lat_deg += (st_.vn * dt / kEarthR) * 180.0 / kPi;
    st_.lon_deg += (st_.ve * dt / (kEarthR * std::cos(st_.lat_deg * kPi / 180.0))) * 180.0 / kPi;
    st_.alt_m += -st_.vd * dt;

    // 地面约束：不穿地、不反弹，触地清零垂直速度
    const double agl = st_.alt_m - ground_alt_;
    if (agl <= 0) {
        st_.alt_m = ground_alt_;
        if (st_.vd > 0) st_.vd = 0;
        st_.vn *= (1.0 - 5.0 * dt);
        st_.ve *= (1.0 - 5.0 * dt);
    }

    // I*omega_dot = torque - omega x (I*omega)，按刚体角动量方程计算角加速度。
    const double p_rad = st_.p_dps * kPi / 180.0, q_rad = st_.q_dps * kPi / 180.0,
                 r_rad = st_.r_dps * kPi / 180.0;
    const double p_dot = (tau_x + (iyy_ - izz_) * q_rad * r_rad) / ixx_ - 2.0 * p_rad;
    const double q_dot = (tau_y + (izz_ - ixx_) * r_rad * p_rad) / iyy_ - 2.0 * q_rad;
    const double r_dot = (tau_z + (ixx_ - iyy_) * p_rad * q_rad) / izz_ - r_rad;
    double p_new = p_rad + p_dot * dt;
    double q_new = q_rad + q_dot * dt;
    double r_new = r_rad + r_dot * dt;
    if (agl <= 0.01 && thrust < mass_ * kG * 1.05) {
        const double k = clamp(8.0 * dt, 0, 1);
        st_.roll_deg *= 1 - k;
        st_.pitch_deg *= 1 - k;
        p_new *= 1 - k;
        q_new *= 1 - k;
        q_body_to_ned_ = euler_to_quaternion(st_.roll_deg, st_.pitch_deg, st_.yaw_deg);
    }
    st_.p_dps = p_new * 180.0 / kPi;
    st_.q_dps = q_new * 180.0 / kPi;
    st_.r_dps = r_new * 180.0 / kPi;

    const double p_mid = 0.5 * (p_rad + p_new), q_mid = 0.5 * (q_rad + q_new);
    const double r_mid = 0.5 * (r_rad + r_new);
    const auto& a = q_body_to_ned_;
    q_body_to_ned_ = {a[0] - 0.5 * dt * (a[1] * p_mid + a[2] * q_mid + a[3] * r_mid),
                       a[1] + 0.5 * dt * (a[0] * p_mid + a[2] * r_mid - a[3] * q_mid),
                       a[2] + 0.5 * dt * (a[0] * q_mid + a[3] * p_mid - a[1] * r_mid),
                       a[3] + 0.5 * dt * (a[0] * r_mid + a[1] * q_mid - a[2] * p_mid)};
    normalize_quaternion(q_body_to_ned_);
    const auto euler = quaternion_to_euler(q_body_to_ned_);
    st_.roll_deg = euler[0];
    st_.pitch_deg = euler[1];
    st_.yaw_deg = euler[2];
    if (st_.yaw_deg < 0) st_.yaw_deg += 360.0;

    // 5) 自洽传感器合成
    SensorSnapshot s;
    s.lat_deg = st_.lat_deg;
    s.lon_deg = st_.lon_deg;
    s.alt_m = static_cast<float>(st_.alt_m);
    s.agl_m = static_cast<float>(std::max(0.0, agl));
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
    const bool on_ground = (st_.alt_m <= ground_alt_ + 1e-9);
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
    s.rangefinder_cm = agl < 0 ? -1 : static_cast<int32_t>(agl * 100.0);
    return s;
}
