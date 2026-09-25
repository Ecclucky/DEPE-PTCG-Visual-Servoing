#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

namespace
{
double clampValue(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

double safeVariance(double value)
{
    return std::max(1.0e-9, value);
}

constexpr int kAttackPacketLength = 5;
} // namespace

struct AxisState
{
    double e = 0.0;
    double v = 0.0;
    double d = 0.0;
};

class SecureKalmanEstimator
{
public:
    void configure(double dt, double q_e, double q_v, double q_d,
                   double r_e, double r_v, double d_limit,
                   double dos_r_scale, double fdi_r_add)
    {
        dt_ = std::max(1.0e-4, dt);
        q_[0] = std::max(1.0e-12, q_e);
        q_[1] = std::max(1.0e-12, q_v);
        q_[2] = std::max(1.0e-12, q_d);
        r_e_ = std::max(1.0e-9, r_e);
        r_v_ = std::max(1.0e-9, r_v);
        d_limit_ = std::max(0.01, d_limit);
        dos_r_scale_ = std::max(1.0, dos_r_scale);
        fdi_r_add_ = std::max(0.0, fdi_r_add);
    }

    void initialize(double e, double v)
    {
        x_.e = e;
        x_.v = v;
        x_.d = 0.0;
        resetCovariance();
        innovation_norm_ = 0.0;
        y_bar_e_ = e;
        y_bar_v_ = v;
        used_prediction_ = false;
        samples_used_ = 0;
    }

    void reset()
    {
        x_ = AxisState();
        resetCovariance();
        innovation_norm_ = 0.0;
        y_bar_e_ = 0.0;
        y_bar_v_ = 0.0;
        used_prediction_ = true;
        samples_used_ = 0;
    }

    void update(double measured_e, double measured_v, bool has_velocity,
                bool measurement_available, bool dos_active, bool fdi_active,
                double fdi_noise, double previous_u)
    {
        AxisState x_pred = propagate(x_, previous_u);
        predictCovariance();

        const bool use_prediction = dos_active || !measurement_available;
        used_prediction_ = use_prediction;
        samples_used_ = 0;
        x_ = x_pred;

        if (use_prediction)
        {
            innovation_norm_ = 0.0;
            y_bar_e_ = x_pred.e;
            y_bar_v_ = x_pred.v;
            return;
        }

        double y_e = measured_e;
        double y_v = measured_v;
        double r_e = r_e_;
        double r_v = r_v_;

        if (fdi_active)
        {
            y_e += fdi_noise;
            y_v += 0.5 * fdi_noise;
            r_e += fdi_r_add_;
            r_v += 0.25 * fdi_r_add_;
        }
        const double innovation_e = y_e - x_.e;
        scalarUpdate(1.0, 0.0, 0.0, y_e, r_e);
        ++samples_used_;

        double innovation_v = 0.0;
        if (has_velocity)
        {
            innovation_v = y_v - x_.v;
            scalarUpdate(0.0, 1.0, 0.0, y_v, r_v);
            ++samples_used_;
        }

        x_.d = clampValue(x_.d, d_limit_);
        y_bar_e_ = y_e;
        y_bar_v_ = y_v;
        innovation_norm_ = std::sqrt(innovation_e * innovation_e + innovation_v * innovation_v);
    }

    const AxisState &state() const { return x_; }
    double variance(int i) const { return p_[i][i]; }
    double innovationNorm() const { return innovation_norm_; }
    double yBarE() const { return y_bar_e_; }
    double yBarV() const { return y_bar_v_; }
    bool usedPrediction() const { return used_prediction_; }
    int samplesUsed() const { return samples_used_; }

private:
    AxisState propagate(const AxisState &x, double u) const
    {
        AxisState xp = x;
        const double a = -u + x.d;
        xp.e = x.e + dt_ * x.v + 0.5 * dt_ * dt_ * a;
        xp.v = x.v + dt_ * a;
        xp.d = x.d;
        return xp;
    }

    void predictCovariance()
    {
        const double dt2 = dt_ * dt_;
        const double a[3][3] = {
            {1.0, dt_, 0.5 * dt2},
            {0.0, 1.0, dt_},
            {0.0, 0.0, 1.0}};
        double predicted[3][3] = {};
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        predicted[r][c] += a[r][i] * p_[i][j] * a[c][j];
                    }
                }
            }
        }
        for (int i = 0; i < 3; ++i)
        {
            predicted[i][i] += q_[i];
        }
        copyMatrix(predicted, p_);
    }

    void scalarUpdate(double h0, double h1, double h2, double y, double r)
    {
        const double h[3] = {h0, h1, h2};
        const double hx = h0 * x_.e + h1 * x_.v + h2 * x_.d;
        const double residual = y - hx;
        double ph[3] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                ph[i] += p_[i][j] * h[j];
            }
        }
        double s = r;
        for (int i = 0; i < 3; ++i)
        {
            s += h[i] * ph[i];
        }
        s = std::max(1.0e-9, s);

        const double k[3] = {ph[0] / s, ph[1] / s, ph[2] / s};
        x_.e += k[0] * residual;
        x_.v += k[1] * residual;
        x_.d += k[2] * residual;

        double updated[3][3] = {};
        for (int r_i = 0; r_i < 3; ++r_i)
        {
            for (int c_i = 0; c_i < 3; ++c_i)
            {
                updated[r_i][c_i] = p_[r_i][c_i] - k[r_i] * ph[c_i];
            }
        }
        copyMatrix(updated, p_);
        symmetrizeCovariance();
    }

    void resetCovariance()
    {
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                p_[r][c] = (r == c) ? 0.05 : 0.0;
            }
        }
    }

    void symmetrizeCovariance()
    {
        for (int r = 0; r < 3; ++r)
        {
            for (int c = r + 1; c < 3; ++c)
            {
                const double avg = 0.5 * (p_[r][c] + p_[c][r]);
                p_[r][c] = avg;
                p_[c][r] = avg;
            }
            p_[r][r] = safeVariance(p_[r][r]);
        }
    }

    void copyMatrix(const double src[3][3], double dst[3][3])
    {
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                dst[r][c] = src[r][c];
            }
        }
    }

    AxisState x_;
    double p_[3][3] = {};
    double q_[3] = {1.0e-4, 1.0e-3, 5.0e-3};
    double dt_ = 0.01;
    double r_e_ = 2.0e-3;
    double r_v_ = 1.0e-2;
    double d_limit_ = 2.0;
    double dos_r_scale_ = 200.0;
    double fdi_r_add_ = 1.0e-2;
    double innovation_norm_ = 0.0;
    double y_bar_e_ = 0.0;
    double y_bar_v_ = 0.0;
    bool used_prediction_ = true;
    int samples_used_ = 0;
};

class AxisMultiRateFusionController
{
public:
    void configure(double dt, int horizon, double kp, double kv, double u_max,
                   double metric_q_e, double metric_q_v, double metric_q_d,
                   double process_q_e, double process_q_v, double process_q_d,
                   double visual_re, double local_re, double local_rv,
                   double fusion_delta, double dos_r_scale, double fdi_amp,
                   double d_limit, double disturbance_comp_gain)
    {
        dt_ = std::max(1.0e-4, dt);
        horizon_ = std::max(1, horizon);
        kp_ = kp;
        kv_ = kv;
        u_max_ = u_max;
        q_e_ = metric_q_e;
        q_v_ = metric_q_v;
        q_d_ = metric_q_d;
        process_q_e_ = process_q_e;
        process_q_v_ = process_q_v;
        process_q_d_ = process_q_d;
        visual_re_ = visual_re;
        local_re_ = local_re;
        local_rv_ = local_rv;
        fusion_delta_ = fusion_delta;
        fdi_amp_ = fdi_amp;
        d_limit_ = d_limit;
        disturbance_comp_gain_ = std::max(0.0, std::min(1.0, disturbance_comp_gain));
        visual_estimator_.configure(dt_, process_q_e_, process_q_v_, process_q_d_, visual_re_, local_rv_, d_limit_, dos_r_scale, fdi_amp_ * fdi_amp_);
        local_estimator_.configure(dt_, process_q_e_, process_q_v_, process_q_d_, local_re_, local_rv_, d_limit_, dos_r_scale, fdi_amp_ * fdi_amp_);
    }

    void initialize(double visual_e)
    {
        visual_estimator_.initialize(visual_e, 0.0);
        local_estimator_.initialize(visual_e, 0.0);
        fused_.e = visual_e;
        fused_.v = 0.0;
        fused_.d = 0.0;
        last_u_ = 0.0;
        candidate_u_ = 0.0;
        fusion_metric_ = 0.0;
        fusion_pass_ = true;
        used_prediction_ = false;
        packet_compute_time_ = 0.0;
    }

    void reset()
    {
        visual_estimator_.reset();
        local_estimator_.reset();
        fused_ = AxisState();
        last_u_ = 0.0;
        candidate_u_ = 0.0;
        fusion_metric_ = 0.0;
        fusion_pass_ = false;
        used_prediction_ = true;
        packet_compute_time_ = 0.0;
    }

    double update(double visual_e, bool has_visual_measurement,
                  double local_visual_v, bool has_local_velocity,
                  bool dos_active, bool fdi_active, double fdi_noise)
    {
        const auto tic = std::chrono::steady_clock::now();
        const double bounded_noise = clampValue(fdi_noise, fdi_amp_);

        visual_estimator_.update(visual_e, 0.0, false, has_visual_measurement,
                                 dos_active, fdi_active, bounded_noise, last_u_);
        local_estimator_.update(visual_e, local_visual_v, has_local_velocity,
                                has_visual_measurement,
                                false, false, 0.0, last_u_);
        fuseEstimates();

        candidate_u_ = controlLaw(fused_);
        last_u_ = candidate_u_;

        const auto toc = std::chrono::steady_clock::now();
        packet_compute_time_ = std::chrono::duration<double>(toc - tic).count();
        return last_u_;
    }

    const AxisState &remote() const { return visual_estimator_.state(); }
    const AxisState &smart() const { return local_estimator_.state(); }
    const AxisState &fused() const { return fused_; }
    double lastControl() const { return last_u_; }
    double candidateControl() const { return candidate_u_; }
    double fusionMetric() const { return fusion_metric_; }
    bool fusionPass() const { return fusion_pass_; }
    bool usedPrediction() const { return used_prediction_; }
    int candidateLen() const { return visual_estimator_.samplesUsed(); }
    int bufferLen() const { return local_estimator_.samplesUsed(); }
    double packetComputeTime() const { return packet_compute_time_; }
    double localSafetyControl() const { return controlLaw(local_estimator_.state()); }

private:
    double controlLaw(const AxisState &x) const
    {
        return clampValue(kp_ * x.e + kv_ * x.v, u_max_);
    }

    void fuseEstimates()
    {
        const AxisState &x1 = visual_estimator_.state();
        const AxisState &x2 = local_estimator_.state();
        const double s1[3] = {x1.e, x1.v, x1.d};
        const double s2[3] = {x2.e, x2.v, x2.d};
        double fused_values[3] = {};

        for (int i = 0; i < 3; ++i)
        {
            const double w1 = 1.0 / safeVariance(visual_estimator_.variance(i));
            const double w2 = 1.0 / safeVariance(local_estimator_.variance(i));
            fused_values[i] = (w1 * s1[i] + w2 * s2[i]) / (w1 + w2);
        }

        fused_.e = fused_values[0];
        fused_.v = fused_values[1];
        fused_.d = clampValue(fused_values[2], d_limit_);

        const double de = x1.e - x2.e;
        const double dv = x1.v - x2.v;
        const double dd = x1.d - x2.d;
        fusion_metric_ = std::sqrt(q_e_ * de * de + q_v_ * dv * dv + q_d_ * dd * dd);
        fusion_pass_ = fusion_metric_ <= fusion_delta_;
        used_prediction_ = visual_estimator_.usedPrediction() || local_estimator_.usedPrediction();
    }

    SecureKalmanEstimator visual_estimator_;
    SecureKalmanEstimator local_estimator_;
    AxisState fused_;
    double dt_ = 0.01;
    int horizon_ = 20;
    double kp_ = 4.5;
    double kv_ = 2.0;
    double u_max_ = 3.0;
    double q_e_ = 1.0e-4;
    double q_v_ = 1.0e-3;
    double q_d_ = 1.0e-5;
    double process_q_e_ = 1.0e-4;
    double process_q_v_ = 1.0e-3;
    double process_q_d_ = 1.0e-5;
    double visual_re_ = 5.0e-2;
    double local_re_ = 2.0e-3;
    double local_rv_ = 2.0e-2;
    double fusion_delta_ = 0.75;
    double fdi_amp_ = 0.10;
    double d_limit_ = 0.25;
    double disturbance_comp_gain_ = 0.20;
    double last_u_ = 0.0;
    double candidate_u_ = 0.0;
    double fusion_metric_ = 0.0;
    double packet_compute_time_ = 0.0;
    bool fusion_pass_ = false;
    bool used_prediction_ = true;
};

class MultiRateHybridAttacksController
{
public:
    MultiRateHybridAttacksController() : nh_("~"), rng_(7), uniform_(0.0, 1.0)
    {
        loadParams();
        configureAxes();

        point_sub_ = nh_.subscribe("/point_with_fixed_delay", 1, &MultiRateHybridAttacksController::pointCallback, this, ros::TransportHints().tcpNoDelay());
        px4_state_sub_ = nh_.subscribe("/px4_state_pub", 1, &MultiRateHybridAttacksController::px4StateCallback, this, ros::TransportHints().tcpNoDelay());

        ground_truth_vel_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/velocity", 10, &MultiRateHybridAttacksController::groundTruthVelCallback, this);
        ground_truth_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/pose", 10, &MultiRateHybridAttacksController::groundTruthPoseCallback, this);
        target_ground_truth_vel_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/velocity", 10, &MultiRateHybridAttacksController::targetGroundTruthVelCallback, this);
        target_ground_truth_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/pose", 10, &MultiRateHybridAttacksController::targetGroundTruthPoseCallback, this);

        acc_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
        debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);

        timer_ = nh_.createTimer(ros::Duration(dt_), &MultiRateHybridAttacksController::timerCallback, this);
        ROS_INFO("MultiRate_Hybrid_attacks secure fusion visual servo controller started.");
    }

private:
    void loadParams()
    {
        nh_.param("dt", dt_, 0.01);
        nh_.param("horizon", horizon_, 20);
        nh_.param("x_bias", axis_bias_[0], -0.9);
        nh_.param("y_bias", axis_bias_[1], 0.0);
        nh_.param("z_bias", axis_bias_[2], 0.0);
        nh_.param("target_mocap_x_comp", target_mocap_comp_[0], -0.026);
        nh_.param("target_mocap_y_comp", target_mocap_comp_[1], 0.127);
        nh_.param("target_mocap_z_comp", target_mocap_comp_[2], -0.117);
        nh_.param("uav_mocap_log_x_comp", uav_mocap_log_comp_[0], 0.0);
        nh_.param("uav_mocap_log_y_comp", uav_mocap_log_comp_[1], -0.122);
        nh_.param("uav_mocap_log_z_comp", uav_mocap_log_comp_[2], 0.038);
        double common_u_max = 2.0;
        nh_.param("u_max", common_u_max, 2.0);
        nh_.param("u_max_x", u_max_[0], 0.3);
        nh_.param("u_max_y", u_max_[1], common_u_max);
        nh_.param("u_max_z", u_max_[2], common_u_max);
        nh_.param("kp", kp_, 4.5);
        nh_.param("kv", kv_, 2.0);
        nh_.param("observer_l0", l0_, 0.60);
        nh_.param("observer_l1", l1_, 6.60);
        nh_.param("observer_l2", l2_, 0.0);
        nh_.param("gate_qe", q_e_, 1.0e-4);
        nh_.param("gate_qv", q_v_, 1.0e-3);
        nh_.param("gate_qd", q_d_, 1.0e-5);
        nh_.param("gate_mu", mu_, 0.55);
        nh_.param("gate_delta", delta_, 0.75);
        nh_.param("fdi_amp", fdi_amp_, 0.10);
        nh_.param("smart_qe", smart_qe_, 1.0e-4);
        nh_.param("smart_qv", smart_qv_, 1.0e-3);
        nh_.param("smart_qd", smart_qd_, 1.0e-5);
        nh_.param("smart_re", smart_re_, 2.0e-3);
        nh_.param("smart_rv", smart_rv_, 2.0e-2);
        nh_.param("visual_re", visual_re_, 5.0e-2);
        nh_.param("visual_velocity_alpha", visual_velocity_alpha_, 0.35);
        nh_.param("visual_velocity_limit", visual_velocity_limit_, 2.0);
        nh_.param("visual_velocity_min_dt", visual_velocity_min_dt_, 0.02);
        nh_.param("visual_velocity_max_dt", visual_velocity_max_dt_, 0.20);
        nh_.param("dos_r_scale", dos_r_scale_, 200.0);
        nh_.param("disturbance_limit", d_limit_, 0.25);
        nh_.param("disturbance_comp_gain", disturbance_comp_gain_, 0.20);
        nh_.param("attack_sim_enabled", attack_sim_enabled_, true);
        nh_.param("fdi_prob", fdi_prob_, 0.02);
        nh_.param("dos_prob", dos_prob_, 0.05);
        nh_.param("dos_period_steps", dos_period_steps_, 80);
        nh_.param("dos_active_steps", dos_active_steps_, 4);
        nh_.param("target_loss_hold_steps", target_loss_hold_steps_, 10);
        nh_.param("attack_seed", attack_seed_, 7);
        fdi_prob_ = std::max(0.0, std::min(1.0, fdi_prob_));
        dos_prob_ = std::max(0.0, std::min(1.0, dos_prob_));
        visual_velocity_alpha_ = std::max(0.0, std::min(1.0, visual_velocity_alpha_));
        visual_velocity_limit_ = std::max(0.1, visual_velocity_limit_);
        visual_velocity_min_dt_ = std::max(1.0e-3, visual_velocity_min_dt_);
        visual_velocity_max_dt_ = std::max(visual_velocity_min_dt_, visual_velocity_max_dt_);
        target_loss_hold_steps_ = std::max(1, std::min(10, target_loss_hold_steps_));
        rng_.seed(static_cast<unsigned int>(attack_seed_));
    }

    void configureAxes()
    {
        for (int i = 0; i < 3; ++i)
        {
            axes_[i].configure(dt_, horizon_, kp_, kv_, u_max_[i],
                               q_e_, q_v_, q_d_,
                               smart_qe_, smart_qv_, smart_qd_,
                               visual_re_, smart_re_, smart_rv_,
                               delta_, dos_r_scale_, fdi_amp_, d_limit_, disturbance_comp_gain_);
        }
    }

    void pointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        const bool was_lost = target_lost_;
        target_lost_ = msg->data.size() < 5 || msg->data[4] > 0.5;
        external_fdi_request_ = msg->data.size() >= 7 && std::fabs(msg->data[6]) > 0.5;

        if (msg->data.size() >= 6)
        {
            des_yaw_ = msg->data[5];
        }

        if (target_lost_ || msg->data.size() < 3)
        {
            visual_velocity_valid_ = false;
            visual_velocity_initialized_ = false;
            last_visual_timestamp_ = 0.0;
            return;
        }

        std::array<double, 3> current_visual_error = {{0.0, 0.0, 0.0}};
        for (int i = 0; i < 3; ++i)
        {
            vision_raw_[i] = msg->data[i];
            current_visual_error[i] = vision_raw_[i] + axis_bias_[i];
        }

        double visual_timestamp = ros::Time::now().toSec();
        if (msg->data.size() >= 4 && std::isfinite(msg->data[3]))
        {
            visual_timestamp = msg->data[3];
        }

        const double visual_dt = visual_timestamp - last_visual_timestamp_;
        const bool velocity_sample_valid =
            !was_lost &&
            last_visual_timestamp_ > 0.0 &&
            visual_dt >= visual_velocity_min_dt_ &&
            visual_dt <= visual_velocity_max_dt_;

        if (velocity_sample_valid)
        {
            for (int i = 0; i < 3; ++i)
            {
                const double raw_velocity =
                    (current_visual_error[i] - last_visual_error_[i]) / visual_dt;
                const double bounded_velocity =
                    clampValue(raw_velocity, visual_velocity_limit_);
                visual_velocity_[i] = visual_velocity_initialized_
                                          ? visual_velocity_alpha_ * bounded_velocity +
                                                (1.0 - visual_velocity_alpha_) * visual_velocity_[i]
                                          : bounded_velocity;
            }
            visual_velocity_valid_ = true;
            visual_velocity_initialized_ = true;
        }
        else
        {
            visual_velocity_valid_ = false;
            visual_velocity_initialized_ = false;
        }

        vision_error_ = current_visual_error;
        last_visual_error_ = current_visual_error;
        last_visual_timestamp_ = visual_timestamp;
        target_lost_steps_ = 0;
        valid_visual_seen_ = true;
        new_visual_ = true;

        if (!controller_initialized_)
        {
            initializeAxesFromCurrentState();
            controller_initialized_ = true;
        }

        if (was_lost)
        {
            ROS_INFO("MultiRate_Hybrid_attacks target reacquired: secure fusion estimator receives visual correction.");
        }
    }

    void px4StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        px4_state_ = msg->data;
        if (px4_state_ != 3)
        {
            resetAxes();
            valid_visual_seen_ = false;
        }
    }

    void groundTruthVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        gt_vel_[0] = msg->twist.linear.x;
        gt_vel_[1] = msg->twist.linear.y;
        gt_vel_[2] = msg->twist.linear.z;
    }

    void groundTruthPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        gt_pos_[0] = msg->pose.position.x;
        gt_pos_[1] = msg->pose.position.y;
        gt_pos_[2] = msg->pose.position.z;
        gt_pos_log_[0] = gt_pos_[0] - target_mocap_comp_[0] - uav_mocap_log_comp_[0];
        gt_pos_log_[1] = gt_pos_[1] - target_mocap_comp_[1] - uav_mocap_log_comp_[1];
        gt_pos_log_[2] = gt_pos_[2] - target_mocap_comp_[2] - uav_mocap_log_comp_[2];
    }

    void targetGroundTruthVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        target_gt_vel_[0] = msg->twist.linear.x;
        target_gt_vel_[1] = msg->twist.linear.y;
        target_gt_vel_[2] = msg->twist.linear.z;
    }

    void targetGroundTruthPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        target_gt_raw_pos_[0] = msg->pose.position.x;
        target_gt_raw_pos_[1] = msg->pose.position.y;
        target_gt_raw_pos_[2] = msg->pose.position.z;
    }

    void timerCallback(const ros::TimerEvent &)
    {
        const auto tic = std::chrono::steady_clock::now();
        std::array<double, 3> u = {{0.0, 0.0, 0.0}};
        std::array<double, 3> packet_compute_time = {{0.0, 0.0, 0.0}};

        if (px4_state_ != 3)
        {
            resetAxes();
            valid_visual_seen_ = false;
            publishAccCommand(u);
            publishTimedDebug(tic, u, true, false, 0, 0.0, packet_compute_time);
            ++step_;
            return;
        }

        if (!valid_visual_seen_)
        {
            publishAccCommand(u);
            publishTimedDebug(tic, u, true, false, 0, 0.0, packet_compute_time);
            ++step_;
            return;
        }

        if (!controller_initialized_)
        {
            initializeAxesFromCurrentState();
            controller_initialized_ = true;
        }

        if (target_lost_)
        {
            ++target_lost_steps_;
            if (target_lost_steps_ > target_loss_hold_steps_)
            {
                resetAxes();
                valid_visual_seen_ = false;
                ROS_WARN_THROTTLE(1.0, "MultiRate_Hybrid_attacks target lost too long: zero command protection is active.");
                publishAccCommand(u);
                publishTimedDebug(tic, u, true, false, 0, 0.0, packet_compute_time);
                ++step_;
                return;
            }
        }

        const bool new_attack_packet = attack_packet_slot_ == 0;
        if (new_attack_packet)
        {
            updateAttackFlags();
            packet_fdi_noise_ = fdi_active_ ? randomSigned(fdi_amp_) : 0.0;
        }

        bool any_reject = false;
        bool all_fresh = true;
        int min_samples_used = 2;
        const bool has_visual_measurement = new_visual_ && !target_lost_;
        const bool has_local_velocity =
            has_visual_measurement && visual_velocity_valid_;

        for (int i = 0; i < 3; ++i)
        {
            u[i] = axes_[i].update(vision_error_[i], has_visual_measurement,
                                   visual_velocity_[i], has_local_velocity,
                                   dos_active_, fdi_active_, packet_fdi_noise_);
            packet_compute_time[i] = axes_[i].packetComputeTime();
            any_reject = any_reject || !axes_[i].fusionPass();
            all_fresh = all_fresh && !axes_[i].usedPrediction();
            min_samples_used = std::min(min_samples_used, axes_[i].candidateLen() + axes_[i].bufferLen());
        }

        new_visual_ = false;
        publishAccCommand(u);
        publishTimedDebug(tic, u, any_reject, all_fresh, min_samples_used, packet_fdi_noise_, packet_compute_time);
        ++attack_packet_slot_;
        if (attack_packet_slot_ >= kAttackPacketLength)
        {
            attack_packet_slot_ = 0;
            ++attack_packet_sequence_;
            ++attack_packet_interval_;
        }
        ++step_;
    }

    void updateAttackFlags()
    {
        dos_active_ = false;
        fdi_active_ = false;

        if (!attack_sim_enabled_)
        {
            fdi_active_ = external_fdi_request_;
            return;
        }

        dos_active_ = uniform_(rng_) < dos_prob_;
        fdi_active_ = external_fdi_request_ || (uniform_(rng_) < fdi_prob_);
    }

    double randomSigned(double amplitude)
    {
        return amplitude * (2.0 * uniform_(rng_) - 1.0);
    }

    void initializeAxesFromCurrentState()
    {
        for (int i = 0; i < 3; ++i)
        {
            axes_[i].initialize(vision_error_[i]);
        }
    }

    void resetAxes()
    {
        for (auto &axis : axes_)
        {
            axis.reset();
        }
        controller_initialized_ = false;
        dos_active_ = false;
        fdi_active_ = false;
        attack_packet_slot_ = 0;
        attack_packet_sequence_ = 1;
        attack_packet_interval_ = 0;
        packet_fdi_noise_ = 0.0;
        target_lost_steps_ = 0;
        new_visual_ = false;
        visual_velocity_.fill(0.0);
        last_visual_error_.fill(0.0);
        visual_velocity_valid_ = false;
        visual_velocity_initialized_ = false;
        last_visual_timestamp_ = 0.0;
    }

    void publishTimedDebug(const std::chrono::steady_clock::time_point &tic,
                           const std::array<double, 3> &u, bool any_reject,
                           bool all_fresh, int min_samples_used, double fdi_noise,
                           const std::array<double, 3> &packet_compute_time) const
    {
        const auto toc = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(toc - tic).count();
        publishDebug(u, elapsed, any_reject, all_fresh, min_samples_used, fdi_noise, packet_compute_time);
    }

    void publishAccCommand(const std::array<double, 3> &u)
    {
        quadrotor_msgs::PositionCommand acc_msg;
        acc_msg.header.stamp = ros::Time::now();
        acc_msg.header.frame_id = "world";
        acc_msg.position.x = 0.0;
        acc_msg.position.y = 0.0;
        acc_msg.position.z = 0.0;
        acc_msg.velocity.x = 0.0;
        acc_msg.velocity.y = 0.0;
        acc_msg.velocity.z = 0.0;
        acc_msg.acceleration.x = u[0];
        acc_msg.acceleration.y = u[1];
        acc_msg.acceleration.z = u[2];
        acc_msg.jerk.x = 0.0;
        acc_msg.jerk.y = 0.0;
        acc_msg.jerk.z = 0.0;
        acc_msg.yaw = 0.0;
        acc_msg.yaw_dot = 0.0;
        acc_cmd_pub_.publish(acc_msg);
    }

    void fillAxisDebug(std::vector<double> &data, int base, int axis, double u) const
    {
        const AxisState &remote = axes_[axis].remote();
        const AxisState &smart = axes_[axis].smart();
        const AxisState &fused = axes_[axis].fused();
        data[base + 0] = remote.e;
        data[base + 1] = remote.v;
        data[base + 2] = u;
        data[base + 3] = vision_error_[axis];
        data[base + 4] = gt_pos_[axis];
        data[base + 5] = gt_vel_[axis];
        data[base + 6] = target_gt_raw_pos_[axis];
        data[base + 7] = target_gt_vel_[axis];
        data[base + 8] = smart.e;
        data[base + 9] = smart.v;
        data[base + 10] = smart.d;
        data[base + 11] = remote.d;
        data[base + 12] = axes_[axis].fusionMetric();
        data[base + 13] = axes_[axis].fusionPass() ? 1.0 : 0.0;
        data[base + 14] = axes_[axis].candidateControl();
        data[base + 15] = fused.e;
        data[base + 16] = fused.v;
        data[base + 17] = fused.d;
        data[base + 18] = dos_active_ ? 1.0 : 0.0;
        data[base + 19] = fdi_active_ ? 1.0 : 0.0;
        data[base + 20] = static_cast<double>(axes_[axis].candidateLen());
        data[base + 21] = static_cast<double>(axes_[axis].bufferLen());
        data[base + 22] = axes_[axis].usedPrediction() ? 1.0 : 0.0;
        data[base + 23] = axes_[axis].lastControl();
    }

    void publishDebug(const std::array<double, 3> &u, double elapsed, bool any_reject,
                      bool all_fresh, int min_samples_used, double fdi_noise,
                      const std::array<double, 3> &packet_compute_time) const
    {
        std_msgs::Float64MultiArray dbg;
        dbg.data.assign(155, 0.0);
        fillAxisDebug(dbg.data, 0, 0, u[0]);
        fillAxisDebug(dbg.data, 24, 1, u[1]);
        fillAxisDebug(dbg.data, 48, 2, u[2]);

        dbg.data[72] = static_cast<double>(step_);
        dbg.data[73] = elapsed;
        dbg.data[74] = (target_lost_ || any_reject || dos_active_ || fdi_active_) ? 1.0 : 0.0;
        dbg.data[75] = static_cast<double>(horizon_);
        dbg.data[76] = delta_;
        dbg.data[77] = mu_;
        dbg.data[78] = static_cast<double>(min_samples_used);
        dbg.data[79] = static_cast<double>(px4_state_);
        dbg.data[80] = all_fresh ? 1.0 : 0.0;
        dbg.data[81] = (!any_reject) ? 1.0 : 0.0;
        dbg.data[82] = target_lost_ ? 1.0 : 0.0;
        dbg.data[83] = valid_visual_seen_ ? 1.0 : 0.0;
        dbg.data[84] = controller_initialized_ ? 1.0 : 0.0;
        dbg.data[85] = static_cast<double>(target_lost_steps_);
        dbg.data[86] = static_cast<double>(target_loss_hold_steps_);
        dbg.data[87] = attack_sim_enabled_ ? 1.0 : 0.0;
        dbg.data[88] = external_fdi_request_ ? 1.0 : 0.0;
        dbg.data[89] = fdi_noise;
        dbg.data[90] = static_cast<double>(dos_period_steps_);
        dbg.data[91] = static_cast<double>(dos_active_steps_);
        dbg.data[92] = fdi_prob_;
        dbg.data[93] = fdi_amp_;
        dbg.data[94] = des_yaw_;
        dbg.data[95] = dt_;
        dbg.data[96] = u_max_[0];
        dbg.data[97] = u_max_[1];
        dbg.data[98] = u_max_[2];
        dbg.data[99] = kp_;
        dbg.data[100] = kv_;
        dbg.data[101] = l0_;
        dbg.data[102] = l1_;
        dbg.data[103] = l2_;
        dbg.data[104] = q_e_;
        dbg.data[105] = q_v_;
        dbg.data[106] = q_d_;
        dbg.data[107] = smart_qe_;
        dbg.data[108] = smart_qv_;
        dbg.data[109] = smart_qd_;
        dbg.data[110] = smart_re_;
        dbg.data[111] = smart_rv_;
        dbg.data[112] = packet_compute_time[0];
        dbg.data[113] = packet_compute_time[1];
        dbg.data[114] = packet_compute_time[2];
        dbg.data[115] = packet_compute_time[0] + packet_compute_time[1] + packet_compute_time[2];
        dbg.data[116] = axis_bias_[0];
        dbg.data[117] = axis_bias_[1];
        dbg.data[118] = axis_bias_[2];
        dbg.data[119] = target_mocap_comp_[0];
        dbg.data[120] = target_mocap_comp_[1];
        dbg.data[121] = target_mocap_comp_[2];
        dbg.data[122] = target_gt_raw_pos_[0] - gt_pos_[0];
        dbg.data[123] = target_gt_raw_pos_[1] - gt_pos_[1];
        dbg.data[124] = target_gt_raw_pos_[2] - gt_pos_[2];
        dbg.data[125] = target_gt_vel_[0] - gt_vel_[0];
        dbg.data[126] = target_gt_vel_[1] - gt_vel_[1];
        dbg.data[127] = target_gt_vel_[2] - gt_vel_[2];
        dbg.data[128] = static_cast<double>(attack_seed_);
        dbg.data[129] = 3.0;
        dbg.data[130] = 1.0;
        dbg.data[131] = dos_prob_;
        dbg.data[132] = 2.0;
        dbg.data[133] = ros::Time::now().toSec();
        dbg.data[134] = vision_raw_[0];
        dbg.data[135] = vision_raw_[1];
        dbg.data[136] = vision_raw_[2];
        dbg.data[137] = target_gt_raw_pos_[0] - gt_pos_[0] + target_mocap_comp_[0] + uav_mocap_log_comp_[0];
        dbg.data[138] = target_gt_raw_pos_[1] - gt_pos_[1] + target_mocap_comp_[1] + uav_mocap_log_comp_[1];
        dbg.data[139] = target_gt_raw_pos_[2] - gt_pos_[2] + target_mocap_comp_[2] + uav_mocap_log_comp_[2];
        dbg.data[140] = axes_[0].candidateControl();
        dbg.data[141] = axes_[1].candidateControl();
        dbg.data[142] = axes_[2].candidateControl();
        dbg.data[143] = axes_[0].localSafetyControl();
        dbg.data[144] = axes_[1].localSafetyControl();
        dbg.data[145] = axes_[2].localSafetyControl();
        dbg.data[146] = u[0] - axes_[0].candidateControl();
        dbg.data[147] = u[1] - axes_[1].candidateControl();
        dbg.data[148] = u[2] - axes_[2].candidateControl();
        dbg.data[149] = static_cast<double>(attack_packet_slot_);
        dbg.data[150] = static_cast<double>(attack_packet_sequence_);
        dbg.data[151] = static_cast<double>(attack_packet_interval_);
        dbg.data[152] = uav_mocap_log_comp_[0];
        dbg.data[153] = uav_mocap_log_comp_[1];
        dbg.data[154] = uav_mocap_log_comp_[2];

        debug_pub_.publish(dbg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Subscriber px4_state_sub_;
    ros::Subscriber ground_truth_vel_sub_;
    ros::Subscriber ground_truth_pose_sub_;
    ros::Subscriber target_ground_truth_vel_sub_;
    ros::Subscriber target_ground_truth_pose_sub_;
    ros::Publisher acc_cmd_pub_;
    ros::Publisher debug_pub_;
    ros::Timer timer_;

    std::array<AxisMultiRateFusionController, 3> axes_;
    std::array<double, 3> vision_error_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> vision_raw_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> visual_velocity_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> last_visual_error_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> gt_pos_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> gt_pos_log_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> gt_vel_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> target_gt_raw_pos_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> target_gt_vel_ = {{0.0, 0.0, 0.0}};

    double dt_ = 0.01;
    int horizon_ = 20;
    std::array<double, 3> u_max_ = {{0.3, 2.0, 2.0}};
    std::array<double, 3> axis_bias_ = {{-0.9, 0.0, 0.0}};
    std::array<double, 3> target_mocap_comp_ = {{-0.026, 0.127, -0.117}};
    std::array<double, 3> uav_mocap_log_comp_ = {{0.0, -0.122, 0.038}};
    double kp_ = 4.5;
    double kv_ = 2.0;
    double l0_ = 0.60;
    double l1_ = 6.60;
    double l2_ = 0.0;
    double q_e_ = 1.0e-4;
    double q_v_ = 1.0e-3;
    double q_d_ = 1.0e-5;
    double mu_ = 0.55;
    double delta_ = 0.75;
    double fdi_amp_ = 0.10;
    double smart_qe_ = 1.0e-4;
    double smart_qv_ = 1.0e-3;
    double smart_qd_ = 1.0e-5;
    double smart_re_ = 2.0e-3;
    double smart_rv_ = 2.0e-2;
    double visual_re_ = 5.0e-2;
    double visual_velocity_alpha_ = 0.35;
    double visual_velocity_limit_ = 2.0;
    double visual_velocity_min_dt_ = 0.02;
    double visual_velocity_max_dt_ = 0.20;
    double last_visual_timestamp_ = 0.0;
    double dos_r_scale_ = 200.0;
    double d_limit_ = 0.25;
    double disturbance_comp_gain_ = 0.20;
    double des_yaw_ = 0.0;
    double fdi_prob_ = 0.02;
    double dos_prob_ = 0.05;
    double packet_fdi_noise_ = 0.0;
    bool attack_sim_enabled_ = true;
    bool target_lost_ = true;
    bool valid_visual_seen_ = false;
    bool controller_initialized_ = false;
    bool dos_active_ = false;
    bool fdi_active_ = false;
    bool external_fdi_request_ = false;
    bool new_visual_ = false;
    bool visual_velocity_valid_ = false;
    bool visual_velocity_initialized_ = false;
    int px4_state_ = 0;
    int dos_period_steps_ = 80;
    int dos_active_steps_ = 4;
    int target_loss_hold_steps_ = 10;
    int target_lost_steps_ = 0;
    int attack_seed_ = 7;
    int attack_packet_slot_ = 0;
    unsigned long long attack_packet_sequence_ = 1;
    unsigned long long attack_packet_interval_ = 0;
    unsigned long long step_ = 0;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MultiRate_Hybrid_attacks");
    MultiRateHybridAttacksController controller;
    ros::spin();
    return 0;
}
