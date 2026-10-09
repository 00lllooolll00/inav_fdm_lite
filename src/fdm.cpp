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
}  // namespace

QuadFdm::QuadFdm(double mass_kg, double max_thrust_n, double ground_alt_m)
    : mass_(mass_kg), max_thrust_(max_thrust_n), ground_alt_(ground_alt_m) {}

void QuadFdm::reset(double lat, double lon, double alt, double yaw_deg) {
    st_ = FdmState{};
    st_.lat_deg = lat;
    st_.lon_deg = lon;
    st_.alt_m = alt;
    st_.yaw_deg = yaw_deg;
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

    // 2) 姿态 → 旋转矩阵（body→world NED，ZYX）
    const double r = st_.roll_deg * kPi / 180.0, p = st_.pitch_deg * kPi / 180.0;
    const double y = st_.yaw_deg * kPi / 180.0;
    const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    const double r11 = cy * cp, r12 = cy * sp * sr - sy * cr, r13 = cy * sp * cr + sy * sr;
    const double r21 = sy * cp, r22 = sy * sp * sr + cy * cr, r23 = sy * sp * cr - cy * sr;
    const double r31 = -sp, r32 = cp * sr, r33 = cp * cr;
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

    // 4) 转动积分（一阶惯量 + 陀螺耦合 + 角阻尼）
    const double p_rad = st_.p_dps * kPi / 180.0, q_rad = st_.q_dps * kPi / 180.0,
                 r_rad = st_.r_dps * kPi / 180.0;
    const double gx = -((iyy_ - izz_) * q_rad * r_rad) / ixx_;
    const double gy = -((izz_ - ixx_) * r_rad * p_rad) / iyy_;
    const double gz = -((ixx_ - iyy_) * p_rad * q_rad) / izz_;
    double p_new = p_rad + ((tau_x / ixx_ + gx) - 2.0 * p_rad) * dt;
    double q_new = q_rad + ((tau_y / iyy_ + gy) - 2.0 * q_rad) * dt;
    double r_new = r_rad + ((tau_z / izz_ + gz) - 1.0 * r_rad) * dt;
    // 触地时姿态回正（起落架效应简化）
    if (agl <= 0.01 && thrust < mass_ * kG * 1.05) {
        const double k = clamp(8.0 * dt, 0, 1);
        st_.roll_deg *= (1 - k);
        st_.pitch_deg *= (1 - k);
        p_new *= (1 - k);
        q_new *= (1 - k);
    }
    st_.p_dps = p_new * 180.0 / kPi;
    st_.q_dps = q_new * 180.0 / kPi;
    st_.r_dps = r_new * 180.0 / kPi;

    // 欧拉角积分（ZYX 运动学，body 角速度投影）
    const double tp = std::tan(p);
    const double sec_p = 1.0 / std::max(cp, 1e-6);
    const double rdot = (p_new + sr * tp * q_new + cr * tp * r_new) * 180.0 / kPi;
    const double pdot_e = (cr * q_new - sr * r_new) * 180.0 / kPi;
    const double ydot = ((sr * sec_p) * q_new + (cr * sec_p) * r_new) * 180.0 / kPi;
    st_.roll_deg += rdot * dt;
    st_.pitch_deg = clamp(st_.pitch_deg + pdot_e * dt, -85, 85);
    st_.yaw_deg += ydot * dt;
    while (st_.yaw_deg < 0) st_.yaw_deg += 360;
    while (st_.yaw_deg >= 360) st_.yaw_deg -= 360;
    while (st_.roll_deg < -180) st_.roll_deg += 360;
    while (st_.roll_deg > 180) st_.roll_deg -= 360;

    // 5) 自洽传感器合成
    SensorSnapshot s;
    s.lat_deg = st_.lat_deg;
    s.lon_deg = st_.lon_deg;
    s.alt_m = static_cast<float>(st_.alt_m);
    s.agl_m = static_cast<float>(std::max(0.0, agl));
    // X-Plane 本地系：local_vx=东，local_vy=地（上为负），local_vz=南
    s.local_vx = static_cast<float>(st_.ve);
    s.local_vy = static_cast<float>(st_.vd);
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
    const double f_n = an, f_e = ae, f_d = ad - kG;
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
