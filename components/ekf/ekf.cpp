#include "ekf.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

static const char* TAG = "EKF";
static constexpr float GRAVITY_MS2 = 9.81f;

static float normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

static Eigen::Matrix3f skew_symmetric(const Eigen::Vector3f& v)
{
    Eigen::Matrix3f m;
    m << 0.0f, -v.z(), v.y(),
        v.z(), 0.0f, -v.x(),
        -v.y(), v.x(), 0.0f;
    return m;
}

EspEKF::EspEKF()
{
    state = NominalState();

    P.setIdentity();
    P *= 0.01f;

    R_accel.setIdentity();
    R_accel *= 0.20f;

    R_mag.setIdentity();
    R_mag *= 0.15f;

    set_process_noise(q_angle_, q_gyro_bias_, q_accel_bias_);

    is_initialized = false;
    ESP_LOGI(TAG, "EKF Initialized. Eigen Version: %d.%d.%d",
        EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
}

void EspEKF::init(const Vector3f& accel_meas)
{
    if (accel_meas.norm() < 1e-3f)
    {
        return;
    }

    // FRD specific force at rest is ~[0,0,-g]. Convert to gravity direction in body.
    const Vector3f acc_norm = accel_meas.normalized();
    const Vector3f g_body = -acc_norm;

    const float pitch = atan2f(-g_body.x(), sqrtf(g_body.y() * g_body.y() + g_body.z() * g_body.z()));
    const float roll = atan2f(g_body.y(), g_body.z());
    const float yaw = 0.0f;

    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    state.q = yawAngle * pitchAngle * rollAngle;
    state.q.normalize();
    state.gyro_bias.setZero();
    state.accel_bias.setZero();

    is_initialized = true;
    ESP_LOGI(TAG, "EKF initialized with Roll: %.2f deg, Pitch: %.2f deg", roll * 57.295f, pitch * 57.295f);
}

void EspEKF::predict(const Vector3f& gyro_meas, float dt)
{
    if (!is_initialized || dt <= 0.0f) return;

    const Vector3f omega = gyro_meas - state.gyro_bias;
    const float omega_norm = omega.norm();

    Eigen::Quaternionf dq;
    if (omega_norm > 1e-5f)
    {
        const float theta_half = omega_norm * dt * 0.5f;
        const float sin_theta_half = std::sin(theta_half);
        dq.w() = std::cos(theta_half);
        dq.vec() = (omega / omega_norm) * sin_theta_half;
    }
    else
    {
        dq.w() = 1.0f;
        dq.vec() = omega * dt * 0.5f;
    }
    state.q = state.q * dq;
    state.q.normalize();

    Eigen::Matrix<float, DIM_ERR, DIM_ERR> Fx;
    Fx.setIdentity();

    const Eigen::Matrix3f omega_skew = skew_symmetric(omega);
    Fx.block<3, 3>(IDX_DTHETA, IDX_DTHETA) = Eigen::Matrix3f::Identity() - omega_skew * dt;
    Fx.block<3, 3>(IDX_DTHETA, IDX_DBG) = -Eigen::Matrix3f::Identity() * dt;
    // dba random walk (identity propagation)

    P = Fx * P * Fx.transpose() + Q * dt;
    P = 0.5f * (P + P.transpose());
    for (int i = 0; i < DIM_ERR; i++)
    {
        P(i, i) = std::max(P(i, i), 1e-9f);
    }
}

void EspEKF::fuse_accel(const Vector3f& accel_meas, float gyro_norm)
{
    if (!is_initialized) return;

    accel_dbg_ = AccelDebug();

    const float acc_norm = accel_meas.norm();
    accel_dbg_.acc_norm = acc_norm;

    if (acc_norm < 2.0f || acc_norm > 30.0f)
    {
        accel_dbg_.used = false;
        return;
    }

    const float acc_dev = std::fabs(acc_norm - GRAVITY_MS2);
    const float acc_ratio = acc_dev / std::max(acc_gate_ms2_, 0.1f);
    const float w_from_acc = 1.0f / (1.0f + acc_ratio * acc_ratio);
    const float gyro_ratio = std::max(0.0f, (gyro_norm - 0.5f) / 5.0f);
    const float w_from_gyro = 1.0f / (1.0f + gyro_ratio * gyro_ratio);
    float acc_weight = std::clamp(w_from_acc * w_from_gyro, 0.05f, 1.0f);

    const Vector3f g_world(0.0f, 0.0f, GRAVITY_MS2);
    const Vector3f f_pred_no_bias = state.q.inverse() * (-g_world);
    const Vector3f f_pred = f_pred_no_bias + state.accel_bias;
    const Vector3f y = accel_meas - f_pred;

    accel_dbg_.used = true;
    accel_dbg_.acc_weight = acc_weight;
    accel_dbg_.z = accel_meas;
    accel_dbg_.g_pred = f_pred;
    accel_dbg_.innov = y;

    Eigen::Matrix<float, 3, DIM_ERR> H;
    H.setZero();
    H.block<3, 3>(0, IDX_DTHETA) = skew_symmetric(f_pred_no_bias);
    H.block<3, 3>(0, IDX_DBA) = Eigen::Matrix3f::Identity();

    const Eigen::Matrix3f R_eff = R_accel * (1.0f / acc_weight);
    Eigen::Matrix3f S = H * P * H.transpose() + R_eff;
    Eigen::Matrix<float, DIM_ERR, 3> K = P * H.transpose() * S.inverse();

    // accel does not observe yaw directly
    K.row(IDX_DTHETA + 2).setZero();
    // avoid unstable z-gyro-bias update from accel-only correction
    K.row(IDX_DBG + 2).setZero();

    const Eigen::Matrix<float, DIM_ERR, 1> delta_x = K * y;

    Vector3f delta_theta = delta_x.segment<3>(IDX_DTHETA);
    const float dtheta_max = 0.20f;
    if (delta_theta.norm() > dtheta_max)
    {
        delta_theta = delta_theta.normalized() * dtheta_max;
    }

    Eigen::Quaternionf dq;
    dq.w() = 1.0f;
    dq.vec() = 0.5f * delta_theta;
    state.q = state.q * dq;
    state.q.normalize();

    Vector3f dbias = delta_x.segment<3>(IDX_DBG);
    const float dbg_max = 0.05f;
    if (dbias.norm() > dbg_max)
    {
        dbias = dbias.normalized() * dbg_max;
    }
    state.gyro_bias += dbias;

    Vector3f dacc_bias = delta_x.segment<3>(IDX_DBA);
    const float dba_max_step = 0.20f;
    if (dacc_bias.norm() > dba_max_step)
    {
        dacc_bias = dacc_bias.normalized() * dba_max_step;
    }
    state.accel_bias += dacc_bias;
    const float abs_acc_bias_max = 3.0f;
    state.accel_bias = state.accel_bias.cwiseMax(Vector3f(-abs_acc_bias_max, -abs_acc_bias_max, -abs_acc_bias_max));
    state.accel_bias = state.accel_bias.cwiseMin(Vector3f(abs_acc_bias_max, abs_acc_bias_max, abs_acc_bias_max));

    accel_dbg_.dtheta = delta_theta;
    accel_dbg_.dbias = dbias;
    accel_dbg_.dacc_bias = dacc_bias;

    const Eigen::Matrix<float, DIM_ERR, DIM_ERR> I = Eigen::Matrix<float, DIM_ERR, DIM_ERR>::Identity();
    const Eigen::Matrix<float, DIM_ERR, DIM_ERR> IKH = (I - K * H);
    P = IKH * P * IKH.transpose() + K * R_eff * K.transpose();
    P = 0.5f * (P + P.transpose());
    for (int i = 0; i < DIM_ERR; i++)
    {
        P(i, i) = std::max(P(i, i), 1e-9f);
    }
}

void EspEKF::fuse_mag(const Vector3f& mag_meas)
{
    if (!is_initialized) return;

    mag_dbg_ = MagDebug();

    const float mag_norm = mag_meas.norm();
    mag_dbg_.mag_norm = mag_norm;

    if (mag_norm < 0.01f || mag_norm > 1000.0f)
    {
        mag_dbg_.used = false;
        return;
    }

    const Vector3f m_body = mag_meas.normalized();

    const float qw = state.q.w();
    const float qx = state.q.x();
    const float qy = state.q.y();
    const float qz = state.q.z();

    const float roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    const float pitch = asinf(std::clamp(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f));

    const float cos_roll = cosf(roll);
    const float sin_roll = sinf(roll);
    const float cos_pitch = cosf(pitch);
    const float sin_pitch = sinf(pitch);

    Matrix3f R_rp;
    R_rp << cos_pitch, sin_roll * sin_pitch, cos_roll * sin_pitch,
        0.0f, cos_roll, -sin_roll,
        -sin_pitch, sin_roll * cos_pitch, cos_roll * cos_pitch;

    const Vector3f m_horizontal = R_rp * m_body;
    float yaw_measured = -atan2f(m_horizontal.y(), m_horizontal.x());
    yaw_measured = normalize_angle(yaw_measured + mag_declination_rad_);

    const Vector3f m_world = state.q * m_body;
    const float yaw_predicted = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    const float yaw_residual = normalize_angle(yaw_measured - yaw_predicted);

    mag_dbg_.used = false;
    mag_dbg_.yaw_measured = yaw_measured;
    mag_dbg_.yaw_predicted = yaw_predicted;
    mag_dbg_.yaw_residual = yaw_residual;
    mag_dbg_.m_world = m_world;
    mag_dbg_.m_body = m_body;

    if (!mag_yaw_aligned)
    {
        const float cy = cosf(yaw_measured * 0.5f);
        const float sy = sinf(yaw_measured * 0.5f);
        const float cp = cosf(pitch * 0.5f);
        const float sp = sinf(pitch * 0.5f);
        const float cr = cosf(roll * 0.5f);
        const float sr = sinf(roll * 0.5f);

        state.q.w() = cr * cp * cy + sr * sp * sy;
        state.q.x() = sr * cp * cy - cr * sp * sy;
        state.q.y() = cr * sp * cy + sr * cp * sy;
        state.q.z() = cr * cp * sy - sr * sp * cy;
        state.q.normalize();

        mag_yaw_aligned = true;
        mag_dbg_.used = true;
        mag_dbg_.mag_weight = 1.0f;
        mag_dbg_.dtheta = Vector3f(0.0f, 0.0f, yaw_residual);
        mag_dbg_.dbias = Vector3f::Zero();
        ESP_LOGI(TAG, "[MAG_INIT] Yaw aligned: %.1f deg", yaw_measured * 57.29578f);
        return;
    }

    const int window_size = 20;
    static float residual_buffer[window_size] = { 0.0f };
    static int residual_index = 0;
    static int residual_count = 0;

    residual_buffer[residual_index] = yaw_residual;
    residual_index = (residual_index + 1) % window_size;
    if (residual_count < window_size) residual_count++;

    float std_dev = 0.0f;
    if (residual_count >= window_size)
    {
        float mean = 0.0f;
        for (int i = 0; i < window_size; i++)
        {
            mean += residual_buffer[i];
        }
        mean /= static_cast<float>(window_size);

        float variance = 0.0f;
        for (int i = 0; i < window_size; i++)
        {
            const float diff = residual_buffer[i] - mean;
            variance += diff * diff;
        }
        variance /= static_cast<float>(window_size);
        std_dev = sqrtf(variance);
    }
    mag_dbg_.yaw_residual_std = std_dev;

    const float residual_abs = std::fabs(yaw_residual);
    const float residual_gate = std::max(mag_gate_rad_, 0.05f);
    const float hard_reject_gate = std::min(2.5f * residual_gate, 2.6f); // <=149 deg

    if (residual_abs > hard_reject_gate)
    {
        mag_dbg_.used = false;
        mag_dbg_.mag_weight = 0.0f;
        return;
    }

    const float residual_weight = std::clamp(1.0f - (residual_abs / residual_gate), 0.05f, 1.0f);
    const float std_weight = 1.0f / (1.0f + (std_dev / 0.35f) * (std_dev / 0.35f));

    // Normal terrestrial field is commonly in 0.25~0.65 Gauss, use smooth penalty outside this zone.
    const float mag_center = 0.45f;
    const float mag_sigma = 0.22f;
    const float mag_diff = (mag_norm - mag_center) / mag_sigma;
    const float norm_weight = std::clamp(expf(-0.5f * mag_diff * mag_diff), 0.10f, 1.0f);

    // Yaw becomes weakly observable near pitch singularity.
    const float abs_pitch_deg = std::fabs(pitch) * 57.29578f;
    float pitch_weight = 1.0f;
    if (abs_pitch_deg > 75.0f)
    {
        pitch_weight = std::clamp((89.0f - abs_pitch_deg) / 14.0f, 0.05f, 1.0f);
    }

    const float w_mag = std::clamp(residual_weight * std_weight * norm_weight * pitch_weight, mag_w_min_, 1.0f);
    mag_dbg_.mag_weight = w_mag;

    Eigen::Matrix<float, 1, DIM_ERR> H;
    H.setZero();
    H(0, IDX_DTHETA + 2) = 1.0f;

    const float R_mag_eff = R_mag(0, 0) / w_mag;
    const float P_yaw = P(IDX_DTHETA + 2, IDX_DTHETA + 2);
    const float S = P_yaw + R_mag_eff;

    Eigen::Matrix<float, DIM_ERR, 1> K = P.col(IDX_DTHETA + 2) * (1.0f / S);

    K(IDX_DTHETA + 0) = 0.0f;
    K(IDX_DTHETA + 1) = 0.0f;
    K(IDX_DBG + 0) = 0.0f;
    K(IDX_DBG + 1) = 0.0f;
    K(IDX_DBA + 0) = 0.0f;
    K(IDX_DBA + 1) = 0.0f;
    K(IDX_DBA + 2) = 0.0f;

    last_mag_k_yaw_bias_z = K(IDX_DBG + 2);

    const Eigen::Matrix<float, DIM_ERR, 1> delta_x = K * yaw_residual;

    Vector3f delta_theta = delta_x.segment<3>(IDX_DTHETA);
    Eigen::Quaternionf dq;
    dq.w() = 1.0f;
    dq.vec() = 0.5f * delta_theta;
    state.q = state.q * dq;
    state.q.normalize();

    Vector3f dbias = delta_x.segment<3>(IDX_DBG);
    state.gyro_bias += dbias;

    const float max_bias = 0.05f;
    state.gyro_bias = state.gyro_bias.cwiseMax(Vector3f(-max_bias, -max_bias, -max_bias));
    state.gyro_bias = state.gyro_bias.cwiseMin(Vector3f(max_bias, max_bias, max_bias));

    mag_dbg_.used = true;
    mag_dbg_.dtheta = delta_theta;
    mag_dbg_.dbias = dbias;

    const Eigen::Matrix<float, DIM_ERR, DIM_ERR> I = Eigen::Matrix<float, DIM_ERR, DIM_ERR>::Identity();
    const Eigen::Matrix<float, DIM_ERR, DIM_ERR> IKH = (I - K * H);
    P = IKH * P * IKH.transpose() + (K * R_mag_eff) * K.transpose();
    P = 0.5f * (P + P.transpose());
    for (int i = 0; i < DIM_ERR; i++)
    {
        P(i, i) = std::max(P(i, i), 1e-9f);
    }
}

Eigen::Vector3f EspEKF::get_euler_angles()
{
    const float qw = state.q.w();
    const float qx = state.q.x();
    const float qy = state.q.y();
    const float qz = state.q.z();

    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    const float roll = atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (qw * qy - qz * qx);
    float pitch;
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI / 2.0f, sinp);
    else
        pitch = asinf(sinp);

    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    const float yaw = atan2f(siny_cosp, cosy_cosp);

    return Vector3f(roll, pitch, yaw);
}

Eigen::Vector3f EspEKF::get_gyro_bias()
{
    return state.gyro_bias;
}

void EspEKF::set_process_noise(float q_angle, float q_gyro_bias, float q_accel_bias)
{
    q_angle_ = std::max(q_angle, 1e-9f);
    q_gyro_bias_ = std::max(q_gyro_bias, 1e-9f);
    q_accel_bias_ = std::max(q_accel_bias, 1e-9f);

    Q.setZero();
    Q.block<3, 3>(IDX_DTHETA, IDX_DTHETA) = Matrix3f::Identity() * q_angle_;
    Q.block<3, 3>(IDX_DBG, IDX_DBG) = Matrix3f::Identity() * q_gyro_bias_;
    Q.block<3, 3>(IDX_DBA, IDX_DBA) = Matrix3f::Identity() * q_accel_bias_;
}

void EspEKF::set_measure_noise_accel(float r_accel)
{
    R_accel.setIdentity();
    R_accel *= std::max(r_accel, 1e-6f);
}

void EspEKF::set_measure_noise_mag(float r_mag)
{
    R_mag.setIdentity();
    R_mag *= std::max(r_mag, 1e-6f);
}

void EspEKF::set_mag_declination(float declination_deg)
{
    mag_declination_rad_ = declination_deg * (M_PI / 180.0f);
}

void EspEKF::set_adaptive_fusion_config(float mag_w_min, float mag_gate_deg, float acc_gate_ms2)
{
    mag_w_min_ = std::clamp(mag_w_min, 0.01f, 1.0f);
    mag_gate_rad_ = std::clamp(mag_gate_deg, 3.0f, 120.0f) * (M_PI / 180.0f);
    acc_gate_ms2_ = std::clamp(acc_gate_ms2, 0.5f, 12.0f);
}
