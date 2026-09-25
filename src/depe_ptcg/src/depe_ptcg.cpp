#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#ifndef DEPE_PTCG_CORE_ONLY
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#endif

namespace
{
const int kPacketLength = 5;
const double kFastPeriod = 0.01;

double clampValue(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

double signNonzero(double value)
{
    return value >= 0.0 ? 1.0 : -1.0;
}

bool decodeVisualMessage(const std::vector<double> &data,
                         const std::array<double, 3> &axis_bias,
                         std::array<double, 3> &visual_error,
                         bool &target_lost,
                         bool &external_fdi_request)
{
    target_lost = true;
    external_fdi_request = false;
    if (data.size() < 5)
    {
        return false;
    }

    for (std::size_t i = 0; i < 5; ++i)
    {
        if (!std::isfinite(data[i]))
        {
            return false;
        }
    }
    if (data[4] > 0.5)
    {
        return false;
    }

    std::array<double, 3> decoded = {{0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; ++i)
    {
        const double biased = data[i] + axis_bias[i];
        if (!std::isfinite(biased))
        {
            return false;
        }
        decoded[i] = biased;
    }

    visual_error = decoded;
    target_lost = false;
    if (data.size() >= 7 && std::isfinite(data[6]))
    {
        external_fdi_request = std::fabs(data[6]) > 0.5;
    }
    return true;
}
} // namespace

struct AxisState
{
    double e = 0.0;
    double v = 0.0;
    double d = 0.0;
};

struct AxisPacket
{
    std::vector<AxisState> x;
    std::vector<double> u;
    int len = 0;
    bool fresh = false;
    unsigned long long sequence = 0;
    unsigned long long interval = 0;
    double timestamp = 0.0;
};

class AxisSecureController
{
public:
    AxisSecureController()
    {
        resetCovariance();
    }

    void configure(double dt, int horizon, double kp, double kv, double u_max,
                   double l0, double l1, double l2,
                   double q_e, double q_v, double q_d,
                   double mu, double delta, double fdi_amp,
                   double smart_qe, double smart_qv, double smart_qd,
                   double smart_re, double smart_rv)
    {
        (void)dt;
        dt_ = kFastPeriod;
        (void)horizon;
        horizon_ = kPacketLength;
        kp_ = kp;
        kv_ = kv;
        u_max_ = u_max;
        l0_ = l0;
        l1_ = l1;
        l2_ = l2;
        q_e_ = q_e;
        q_v_ = q_v;
        q_d_ = q_d;
        mu_ = mu;
        delta_ = delta;
        fdi_amp_ = fdi_amp;
        smart_qe_ = smart_qe;
        smart_qv_ = smart_qv;
        smart_qd_ = smart_qd;
        smart_re_ = smart_re;
        smart_rv_ = smart_rv;
        clearActivePacket();
        actual_input_history_.fill(0.0);
        history_count_ = 0;
        remaining_packet_len_ = 0;
        last_processed_sequence_ = 0;
        packet_valid_ = false;
        gate_latched_ = true;
        resetCovariance();
    }

    void setVisualMeasurement(double y)
    {
        visual_y_ = y;
        has_visual_ = true;
        new_visual_ = true;
        smart_visual_y_ = y;
        has_smart_visual_ = true;
        new_smart_visual_ = true;
    }

    void discardPendingVisualMeasurement()
    {
        new_visual_ = false;
        new_smart_visual_ = false;
    }

    void initialize(double visual_e)
    {
        remote_.e = visual_e;
        remote_.v = 0.0;
        remote_.d = 0.0;
        smart_.e = visual_e;
        smart_.v = 0.0;
        smart_.d = 0.0;
        visual_y_ = visual_e;
        smart_visual_y_ = visual_e;
        has_visual_ = true;
        new_visual_ = false;
        has_smart_visual_ = true;
        new_smart_visual_ = false;
        last_u_ = 0.0;
        local_safety_u_ = 0.0;
        packet_compute_time_ = 0.0;
        actual_input_history_.fill(0.0);
        history_count_ = 0;
        remaining_packet_len_ = 0;
        last_processed_sequence_ = 0;
        packet_valid_ = false;
        gate_latched_ = true;
        clearActivePacket();
        resetCovariance();
    }

    void reset()
    {
        remote_ = AxisState();
        smart_ = AxisState();
        candidate_x_ = AxisState();
        clearActivePacket();
        actual_input_history_.fill(0.0);
        visual_y_ = 0.0;
        has_visual_ = false;
        new_visual_ = false;
        smart_visual_y_ = 0.0;
        has_smart_visual_ = false;
        new_smart_visual_ = false;
        last_u_ = 0.0;
        local_safety_u_ = 0.0;
        candidate_u_ = 0.0;
        gate_metric_ = 0.0;
        packet_compute_time_ = 0.0;
        gate_pass_ = false;
        fresh_packet_ = false;
        used_hold_ = true;
        candidate_len_ = 0;
        history_count_ = 0;
        remaining_packet_len_ = 0;
        last_processed_sequence_ = 0;
        packet_valid_ = false;
        gate_latched_ = true;
        resetCovariance();
    }

    double updateControl(bool new_interval, int slot,
                         bool dos_active, bool fdi_active, double fdi_noise,
                         unsigned long long sequence,
                         unsigned long long interval,
                         double timestamp)
    {
        if (new_interval)
        {
            beginPacketInterval(dos_active, fdi_active, fdi_noise,
                                sequence, interval, timestamp);
        }

        if (slot < 0 || slot >= kPacketLength)
        {
            gate_metric_ = 1.0e9;
            gate_pass_ = false;
            gate_latched_ = true;
            used_hold_ = true;
            last_u_ = local_safety_u_;
            return last_u_;
        }

        const bool has_candidate = packet_valid_ &&
                                   active_packet_.len == kPacketLength &&
                                   static_cast<int>(active_packet_.u.size()) == kPacketLength &&
                                   static_cast<int>(active_packet_.x.size()) == kPacketLength;
        if (has_candidate)
        {
            candidate_u_ = active_packet_.u[slot];
            candidate_x_ = active_packet_.x[slot];
            candidate_len_ = active_packet_.len;
        }
        else
        {
            candidate_u_ = 0.0;
            candidate_x_ = AxisState();
            candidate_len_ = 0;
        }

        if (has_candidate)
        {
            gate_metric_ = std::fabs(candidate_u_ - local_safety_u_);
            gate_pass_ = !gate_latched_ && gate_metric_ <= delta_;
        }
        else
        {
            gate_metric_ = 1.0e9;
            gate_pass_ = false;
        }

        if (!gate_pass_)
        {
            gate_latched_ = true;
        }

        last_u_ = gate_pass_ ? candidate_u_ : local_safety_u_;
        actual_input_history_[slot] = last_u_;
        history_count_ = std::max(history_count_, slot + 1);
        remaining_packet_len_ = has_candidate ? kPacketLength - slot - 1 : 0;
        used_hold_ = !gate_pass_;
        return last_u_;
    }

    const AxisState &remote() const { return remote_; }
    const AxisState &smart() const { return smart_; }
    const AxisState &candidateState() const { return candidate_x_; }
    double lastControl() const { return last_u_; }
    double candidateControl() const { return candidate_u_; }
    double gateMetric() const { return gate_metric_; }
    bool gatePass() const { return gate_pass_; }
    bool freshPacket() const { return fresh_packet_; }
    bool usedHold() const { return used_hold_; }
    int candidateLen() const { return candidate_len_; }
    int bufferLen() const { return remaining_packet_len_; }
    double packetComputeTime() const { return packet_compute_time_; }
    double nominalCandidateControl() const { return controlLaw(candidate_x_); }
    double localSafetyControl() const { return local_safety_u_; }

private:
    double controlLaw(const AxisState &x) const
    {
        return clampValue(kp_ * x.e + kv_ * x.v + x.d, u_max_);
    }

    AxisState propagateObserver(const AxisState &x) const
    {
        AxisState xp = x;
        const double error_acc = x.d;
        xp.e = x.e + dt_ * x.v + 0.5 * dt_ * dt_ * error_acc;
        xp.v = x.v + dt_ * error_acc;
        xp.d = x.d;
        return xp;
    }

    AxisState propagatePlant(const AxisState &x, double u) const
    {
        AxisState xp = x;
        const double error_acc = -u + x.d;
        xp.e = x.e + dt_ * x.v + 0.5 * dt_ * dt_ * error_acc;
        xp.v = x.v + dt_ * error_acc;
        xp.d = x.d;
        return xp;
    }

    void beginPacketInterval(bool dos_active, bool fdi_active, double fdi_noise,
                             unsigned long long sequence,
                             unsigned long long interval,
                             double timestamp)
    {
        const bool has_complete_history = history_count_ == kPacketLength;
        stepRemoteEstimatorSlow(has_complete_history);
        stepSmartSensorSlow(has_complete_history);
        local_safety_u_ = controlLaw(smart_);

        const auto packet_tic = std::chrono::steady_clock::now();
        const AxisPacket controller_packet = makeRemotePacket(sequence, interval, timestamp);
        AxisPacket candidate;
        if (!dos_active)
        {
            candidate = controller_packet;
            if (fdi_active)
            {
                applyDeception(candidate, fdi_noise);
            }
        }

        packet_valid_ = !dos_active &&
                        validatePacket(candidate, sequence, interval, timestamp);
        if (packet_valid_)
        {
            last_processed_sequence_ = candidate.sequence;
            active_packet_ = candidate;
            remaining_packet_len_ = kPacketLength;
        }
        else
        {
            clearActivePacket();
            remaining_packet_len_ = 0;
        }

        const auto packet_toc = std::chrono::steady_clock::now();
        packet_compute_time_ = std::chrono::duration<double>(packet_toc - packet_tic).count();
        fresh_packet_ = packet_valid_;
        gate_latched_ = !packet_valid_;
        gate_metric_ = packet_valid_ ? 0.0 : 1.0e9;
        gate_pass_ = false;
        used_hold_ = true;
        candidate_len_ = packet_valid_ ? kPacketLength : 0;
        candidate_u_ = 0.0;
        candidate_x_ = AxisState();
        actual_input_history_.fill(0.0);
        history_count_ = 0;
    }

    void stepRemoteEstimatorSlow(bool has_complete_history)
    {
        AxisState predicted = remote_;
        if (has_complete_history)
        {
            for (int j = 0; j < kPacketLength; ++j)
            {
                predicted = propagateObserver(predicted);
            }
        }

        const double residual = (new_visual_ && has_visual_) ? visual_y_ - predicted.e : 0.0;
        remote_ = predicted;
        remote_.e += l0_ * residual;
        remote_.v += l1_ * residual;
        remote_.d = clampValue(remote_.d + l2_ * residual, d_limit_);
        new_visual_ = false;
    }

    void stepSmartSensorSlow(bool has_complete_history)
    {
        const double dt2 = dt_ * dt_;
        const double a[3][3] = {
            {1.0, dt_, 0.5 * dt2},
            {0.0, 1.0, dt_},
            {0.0, 0.0, 1.0}};

        AxisState x_pred = smart_;
        double p_pred[3][3] = {};
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                p_pred[r][c] = smart_p_[r][c];
            }
        }

        if (has_complete_history)
        {
            for (int step = 0; step < kPacketLength; ++step)
            {
                x_pred = propagateObserver(x_pred);
                double p_next[3][3] = {};
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            for (int j = 0; j < 3; ++j)
                            {
                                p_next[r][c] += a[r][i] * p_pred[i][j] * a[c][j];
                            }
                        }
                    }
                }
                p_next[0][0] += smart_qe_;
                p_next[1][1] += smart_qv_;
                p_next[2][2] += smart_qd_;
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        p_pred[r][c] = p_next[r][c];
                    }
                }
            }
        }

        smart_ = x_pred;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                smart_p_[r][c] = p_pred[r][c];
            }
        }

        if (new_smart_visual_ && has_smart_visual_)
        {
            const double residual = smart_visual_y_ - x_pred.e;
            const double innovation = std::max(1.0e-9, p_pred[0][0] + smart_re_);
            double k[3] = {};
            for (int r = 0; r < 3; ++r)
            {
                k[r] = p_pred[r][0] / innovation;
            }

            smart_.e = x_pred.e + k[0] * residual;
            smart_.v = x_pred.v + k[1] * residual;
            smart_.d = clampValue(x_pred.d + k[2] * residual, d_limit_);

            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    smart_p_[r][c] = p_pred[r][c] - k[r] * p_pred[0][c];
                }
            }
        }
        new_smart_visual_ = false;
    }

    AxisPacket makeRemotePacket(unsigned long long sequence,
                                unsigned long long interval,
                                double timestamp) const
    {
        AxisPacket packet;
        packet.x.resize(kPacketLength);
        packet.u.resize(kPacketLength);
        packet.len = kPacketLength;
        packet.fresh = true;
        packet.sequence = sequence;
        packet.interval = interval;
        packet.timestamp = timestamp;

        AxisState xp = remote_;
        for (int i = 0; i < kPacketLength; ++i)
        {
            const double u = controlLaw(xp);
            packet.x[i] = xp;
            packet.u[i] = u;
            xp = propagatePlant(xp, u);
        }
        return packet;
    }

    bool validatePacket(const AxisPacket &packet,
                        unsigned long long expected_sequence,
                        unsigned long long expected_interval,
                        double expected_timestamp) const
    {
        if (!packet.fresh ||
            packet.len != kPacketLength ||
            static_cast<int>(packet.x.size()) != kPacketLength ||
            static_cast<int>(packet.u.size()) != kPacketLength ||
            packet.sequence != expected_sequence ||
            packet.sequence <= last_processed_sequence_ ||
            packet.interval != expected_interval ||
            !std::isfinite(packet.timestamp) ||
            std::fabs(packet.timestamp - expected_timestamp) > 0.5 * dt_)
        {
            return false;
        }

        for (int i = 0; i < kPacketLength; ++i)
        {
            if (!std::isfinite(packet.u[i]) || std::fabs(packet.u[i]) > u_max_)
            {
                return false;
            }
        }
        return true;
    }

    void applyDeception(AxisPacket &packet, double noise) const
    {
        if (packet.len <= 0)
        {
            return;
        }
        const double attack = clampValue(noise, fdi_amp_);
        const double dir = signNonzero(packet.x.front().e + 0.25 * packet.x.front().v + 1.0e-6);
        for (int i = 0; i < packet.len; ++i)
        {
            const double shape = 1.0 + 0.15 * std::sin(0.47 * static_cast<double>(i));
            const double forged = dir * attack * shape;
            packet.u[i] += forged;
        }
        packet.fresh = true;
    }

    void clearActivePacket()
    {
        active_packet_.x.clear();
        active_packet_.u.clear();
        active_packet_.len = 0;
        active_packet_.fresh = false;
        active_packet_.sequence = 0;
        active_packet_.interval = 0;
        active_packet_.timestamp = 0.0;
    }

    void resetCovariance()
    {
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                smart_p_[r][c] = (r == c) ? 0.05 : 0.0;
            }
        }
    }

    double dt_ = kFastPeriod;
    int horizon_ = kPacketLength;
    double kp_ = 4.5;
    double kv_ = 2.0;
    double u_max_ = 3.0;
    double l0_ = 0.60;
    double l1_ = 6.60;
    double l2_ = 0.0;
    double q_e_ = 1.8;
    double q_v_ = 0.45;
    double q_d_ = 0.04;
    double mu_ = 0.55;
    double delta_ = 0.35;
    double fdi_amp_ = 0.10;
    double d_limit_ = 2.0;
    double smart_qe_ = 1.0e-4;
    double smart_qv_ = 1.0e-3;
    double smart_qd_ = 5.0e-3;
    double smart_re_ = 2.0e-3;
    double smart_rv_ = 1.0e-2;

    AxisState remote_;
    AxisState smart_;
    AxisState candidate_x_;
    AxisPacket active_packet_;
    std::array<double, kPacketLength> actual_input_history_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
    double smart_p_[3][3] = {};
    double visual_y_ = 0.0;
    bool has_visual_ = false;
    bool new_visual_ = false;
    double smart_visual_y_ = 0.0;
    bool has_smart_visual_ = false;
    bool new_smart_visual_ = false;
    double last_u_ = 0.0;
    double local_safety_u_ = 0.0;
    double candidate_u_ = 0.0;
    double gate_metric_ = 0.0;
    double packet_compute_time_ = 0.0;
    bool gate_pass_ = false;
    bool fresh_packet_ = false;
    bool used_hold_ = true;
    bool packet_valid_ = false;
    bool gate_latched_ = true;
    int candidate_len_ = 0;
    int history_count_ = 0;
    int remaining_packet_len_ = 0;
    unsigned long long last_processed_sequence_ = 0;
};

#ifndef DEPE_PTCG_CORE_ONLY
class DepePtcgController
{
public:
    DepePtcgController() : nh_("~"), rng_(7), uniform_(0.0, 1.0)
    {
        loadParams();
        configureAxes();

        point_sub_ = nh_.subscribe("/point_with_fixed_delay", 1, &DepePtcgController::pointCallback, this, ros::TransportHints().tcpNoDelay());
        px4_state_sub_ = nh_.subscribe("/px4_state_pub", 1, &DepePtcgController::px4StateCallback, this, ros::TransportHints().tcpNoDelay());

        ground_truth_vel_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/velocity", 10, &DepePtcgController::groundTruthVelCallback, this);
        ground_truth_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/pose", 10, &DepePtcgController::groundTruthPoseCallback, this);
        target_ground_truth_vel_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/velocity", 10, &DepePtcgController::targetGroundTruthVelCallback, this);
        target_ground_truth_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/pose", 10, &DepePtcgController::targetGroundTruthPoseCallback, this);

        acc_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
        debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);

        timer_ = nh_.createTimer(ros::Duration(dt_), &DepePtcgController::timerCallback, this);
        ROS_INFO("depe_ptcg DEPE-PTCG flight-experiment controller started.");
    }

private:
    void loadParams()
    {
        nh_.param("dt", dt_, kFastPeriod);
        if (std::fabs(dt_ - kFastPeriod) > 1.0e-9)
        {
            ROS_WARN("depe_ptcg uses a fixed 0.01 s fast period; dt=%.6f is forced to 0.01.", dt_);
            dt_ = kFastPeriod;
        }
        nh_.param("horizon", horizon_, kPacketLength);
        if (horizon_ != kPacketLength)
        {
            ROS_WARN("depe_ptcg uses a fixed five-element packet; horizon=%d is forced to 5.", horizon_);
            horizon_ = kPacketLength;
        }
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
        nh_.param("gate_qe", q_e_, 1.80);
        nh_.param("gate_qv", q_v_, 0.45);
        nh_.param("gate_qd", q_d_, 0.04);
        nh_.param("gate_mu", mu_, 0.55);
        nh_.param("gate_delta", delta_, 0.35);
        nh_.param("fdi_amp", fdi_amp_, 0.10);
        nh_.param("smart_qe", smart_qe_, 1.0e-4);
        nh_.param("smart_qv", smart_qv_, 1.0e-3);
        nh_.param("smart_qd", smart_qd_, 5.0e-3);
        nh_.param("smart_re", smart_re_, 2.0e-3);
        nh_.param("smart_rv", smart_rv_, 1.0e-2);
        nh_.param("attack_sim_enabled", attack_sim_enabled_, true);
        nh_.param("fdi_prob", fdi_prob_, 0.02);
        nh_.param("dos_prob", dos_prob_, 0.05);
        nh_.param("dos_period_steps", dos_period_steps_, 80);
        nh_.param("dos_active_steps", dos_active_steps_, 4);
        nh_.param("target_loss_hold_steps", target_loss_hold_steps_, 120);
        nh_.param("attack_seed", attack_seed_, 7);
        fdi_prob_ = std::max(0.0, std::min(1.0, fdi_prob_));
        dos_prob_ = std::max(0.0, std::min(1.0, dos_prob_));
        rng_.seed(static_cast<unsigned int>(attack_seed_));
    }

    void configureAxes()
    {
        for (int i = 0; i < 3; ++i)
        {
            axes_[i].configure(dt_, horizon_, kp_, kv_, u_max_[i], l0_, l1_, l2_,
                               q_e_, q_v_, q_d_, mu_, delta_, fdi_amp_,
                               smart_qe_, smart_qv_, smart_qd_, smart_re_, smart_rv_);
        }
    }

    void pointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() >= 3)
        {
            vision_raw_[0] = msg->data[0];
            vision_raw_[1] = msg->data[1];
            vision_raw_[2] = msg->data[2];
        }

        const bool was_lost = target_lost_;
        std::array<double, 3> decoded_visual = {{0.0, 0.0, 0.0}};
        if (!decodeVisualMessage(msg->data, axis_bias_, decoded_visual,
                                 target_lost_, external_fdi_request_))
        {
            for (auto &axis : axes_)
            {
                axis.discardPendingVisualMeasurement();
            }
            return;
        }

        target_lost_steps_ = 0;
        valid_visual_seen_ = true;
        vision_error_ = decoded_visual;

        if (!controller_initialized_)
        {
            initializeAxesFromCurrentState();
            controller_initialized_ = true;
        }
        else
        {
            for (int i = 0; i < 3; ++i)
            {
                axes_[i].setVisualMeasurement(vision_error_[i]);
            }
        }

        if (was_lost)
        {
            ROS_INFO("depe_ptcg target reacquired: valid visual frame accepted.");
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
        target_gt_pos_[0] = target_gt_raw_pos_[0] + target_mocap_comp_[0];
        target_gt_pos_[1] = target_gt_raw_pos_[1] + target_mocap_comp_[1];
        target_gt_pos_[2] = target_gt_raw_pos_[2] + target_mocap_comp_[2];
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
                ROS_WARN_THROTTLE(1.0, "depe_ptcg target lost too long: zero command protection is active.");
                publishAccCommand(u);
                publishTimedDebug(tic, u, true, false, 0, 0.0, packet_compute_time);
                ++step_;
                return;
            }
        }

        const bool new_interval = packet_slot_ == 0;
        if (new_interval)
        {
            updateAttackFlags();
            packet_fdi_noise_ = fdi_active_ ? randomSigned(fdi_amp_) : 0.0;
        }

        bool any_reject = false;
        bool all_fresh = true;
        int min_buffer_len = horizon_;
        const double packet_timestamp =
            static_cast<double>(packet_interval_) * static_cast<double>(kPacketLength) * dt_;

        for (int i = 0; i < 3; ++i)
        {
            u[i] = axes_[i].updateControl(new_interval, packet_slot_,
                                          dos_active_, fdi_active_, packet_fdi_noise_,
                                          packet_sequence_, packet_interval_, packet_timestamp);
            packet_compute_time[i] = axes_[i].packetComputeTime();
            any_reject = any_reject || !axes_[i].gatePass();
            all_fresh = all_fresh && axes_[i].freshPacket();
            min_buffer_len = std::min(min_buffer_len, axes_[i].bufferLen());
        }

        publishAccCommand(u);
        publishTimedDebug(tic, u, any_reject, all_fresh, min_buffer_len, packet_fdi_noise_, packet_compute_time);
        ++packet_slot_;
        if (packet_slot_ >= kPacketLength)
        {
            packet_slot_ = 0;
            ++packet_sequence_;
            ++packet_interval_;
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
        packet_slot_ = 0;
        packet_sequence_ = 1;
        packet_interval_ = 0;
        packet_fdi_noise_ = 0.0;
        target_lost_steps_ = 0;
    }

    void publishTimedDebug(const std::chrono::steady_clock::time_point &tic,
                           const std::array<double, 3> &u, bool any_reject,
                           bool all_fresh, int min_buffer_len, double fdi_noise,
                           const std::array<double, 3> &packet_compute_time) const
    {
        const auto toc = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(toc - tic).count();
        publishDebug(u, elapsed, any_reject, all_fresh, min_buffer_len, fdi_noise, packet_compute_time);
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
        const AxisState &candidate = axes_[axis].candidateState();
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
        data[base + 12] = axes_[axis].gateMetric();
        data[base + 13] = axes_[axis].gatePass() ? 1.0 : 0.0;
        data[base + 14] = axes_[axis].candidateControl();
        data[base + 15] = candidate.e;
        data[base + 16] = candidate.v;
        data[base + 17] = candidate.d;
        data[base + 18] = dos_active_ ? 1.0 : 0.0;
        data[base + 19] = fdi_active_ ? 1.0 : 0.0;
        data[base + 20] = static_cast<double>(axes_[axis].candidateLen());
        data[base + 21] = static_cast<double>(axes_[axis].bufferLen());
        data[base + 22] = axes_[axis].usedHold() ? 1.0 : 0.0;
        data[base + 23] = axes_[axis].lastControl();
    }

    void publishDebug(const std::array<double, 3> &u, double elapsed, bool any_reject,
                      bool all_fresh, int min_buffer_len, double fdi_noise,
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
        dbg.data[78] = static_cast<double>(min_buffer_len);
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
        dbg.data[94] = 0.0;
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
        dbg.data[132] = 1.0;
        dbg.data[133] = ros::Time::now().toSec();
        dbg.data[134] = vision_raw_[0];
        dbg.data[135] = vision_raw_[1];
        dbg.data[136] = vision_raw_[2];
        dbg.data[137] = dbg.data[122] + target_mocap_comp_[0] + uav_mocap_log_comp_[0];
        dbg.data[138] = dbg.data[123] + target_mocap_comp_[1] + uav_mocap_log_comp_[1];
        dbg.data[139] = dbg.data[124] + target_mocap_comp_[2] + uav_mocap_log_comp_[2];
        for (int i = 0; i < 3; ++i)
        {
            const double nominal_u = axes_[i].nominalCandidateControl();
            dbg.data[140 + i] = nominal_u;
            dbg.data[143 + i] = axes_[i].localSafetyControl();
            dbg.data[146 + i] = dos_active_ ? 0.0 : axes_[i].candidateControl() - nominal_u;
        }
        dbg.data[149] = static_cast<double>(packet_slot_);
        dbg.data[150] = static_cast<double>(packet_sequence_);
        dbg.data[151] = static_cast<double>(packet_interval_);
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

    std::array<AxisSecureController, 3> axes_;
    std::array<double, 3> vision_raw_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> vision_error_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> gt_pos_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> gt_pos_log_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> gt_vel_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> target_gt_raw_pos_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> target_gt_pos_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> target_gt_vel_ = {{0.0, 0.0, 0.0}};

    double dt_ = kFastPeriod;
    int horizon_ = kPacketLength;
    std::array<double, 3> u_max_ = {{0.3, 2.0, 2.0}};
    std::array<double, 3> axis_bias_ = {{-0.9, 0.0, 0.0}};
    std::array<double, 3> target_mocap_comp_ = {{-0.026, 0.127, -0.117}};
    std::array<double, 3> uav_mocap_log_comp_ = {{0.0, -0.122, 0.038}};
    double kp_ = 4.5;
    double kv_ = 2.0;
    double l0_ = 0.60;
    double l1_ = 6.60;
    double l2_ = 0.0;
    double q_e_ = 1.80;
    double q_v_ = 0.45;
    double q_d_ = 0.04;
    double mu_ = 0.55;
    double delta_ = 0.35;
    double fdi_amp_ = 0.10;
    double smart_qe_ = 1.0e-4;
    double smart_qv_ = 1.0e-3;
    double smart_qd_ = 5.0e-3;
    double smart_re_ = 2.0e-3;
    double smart_rv_ = 1.0e-2;
    double fdi_prob_ = 0.02;
    double dos_prob_ = 0.05;
    bool attack_sim_enabled_ = true;
    bool target_lost_ = true;
    bool valid_visual_seen_ = false;
    bool controller_initialized_ = false;
    bool dos_active_ = false;
    bool fdi_active_ = false;
    bool external_fdi_request_ = false;
    int px4_state_ = 0;
    int dos_period_steps_ = 80;
    int dos_active_steps_ = 4;
    int target_loss_hold_steps_ = 120;
    int target_lost_steps_ = 0;
    int attack_seed_ = 7;
    int packet_slot_ = 0;
    unsigned long long packet_sequence_ = 1;
    unsigned long long packet_interval_ = 0;
    unsigned long long step_ = 0;
    double packet_fdi_noise_ = 0.0;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depe_ptcg");
    DepePtcgController controller;
    ros::spin();
    return 0;
}
#endif
