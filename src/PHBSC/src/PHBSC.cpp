#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

#include <quadrotor_msgs/PositionCommand.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <cmath>
#include <array>
#include <deque>
#include <vector>
#include <algorithm>
#include <chrono>
#include <random>

namespace
{
double clampValue(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

double signNonzero(double value)
{
    return value >= 0.0 ? 1.0 : -1.0;
}

constexpr int kAttackPacketLength = 5;
} // namespace

// ==========================================================================================
// PHBSC prediction and feedback core
// ==========================================================================================
class PHBSCController
{
private:
    // Parameters
    const double T_c = 0.01;     // Control period (100Hz)
    const int Np_steps = 5;      // Prediction horizon
    const int DoS_Period_X = 80; // Internal periodic attack option; disabled for these experiments
    const int DoS_Peace_a = 76;  // 80-step period, 4-step active window
    const bool enable_internal_dos_ = false;

    // Bias
    double x_bias = -0.9;
    double y_bias = 0.0;
    double z_bias = 0.0;

    Eigen::Matrix2d Ad;      // System Matrix A
    Eigen::Vector2d Bd;      // Input Matrix B
    Eigen::Vector2d L;       // Observer Gain L
    Eigen::RowVector2d K;    // Controller Gain K
    Eigen::RowVector2d C_aug;// Output Matrix C

    // State Variables
    std::deque<double> u_history; // History of applied control inputs

    // Control Queues
    std::vector<double> Uc;       // Candidate control sequence
    std::vector<double> Ua;       // Actual applied sequence (under DoS)

    std::vector<Eigen::Vector2d> Xc;
    std::vector<Eigen::Vector2d> Xa;

    Eigen::Vector2d xhat_tau_;    // Observer state [pos, vel]

    double debug_x_used_for_u_ = 0.0;
    double debug_v_used_for_u_ = 0.0;

public:
    PHBSCController()
    {
        // 1. Initialize Matrices
        Ad << 1.0, 0.01,
              0.0, 1.0;

        Bd <<0.0000, 0.0;

        C_aug << 1.0, 0.0;

        K << 4.5, 2.0;
        L << 0.6, 6.6;

        // Initialize buffers
        Uc.assign(Np_steps, 0.0);
        Ua.assign(Np_steps, 0.0);
        Xc.assign(Np_steps, Eigen::Vector2d::Zero());
        Xa.assign(Np_steps, Eigen::Vector2d::Zero());

        xhat_tau_.setZero();
    }

    void setBias(double xb, double yb, double zb)
    {
        x_bias = xb; y_bias = yb; z_bias = zb;
    }

    double adjustBias(double value, int which_axis, bool use_bias) const
    {
        if (!use_bias) return value;
        if (which_axis == 0) return value + x_bias;
        if (which_axis == 1) return value + y_bias;
        return value + z_bias;
    }

    double limitControl(double controlValue, int which_axis) const
    {
        double limit = (which_axis == 0) ? 0.3 : 6.0;
        if (std::abs(controlValue) >= limit)
            controlValue = limit * controlValue / std::abs(controlValue);
        return controlValue;
    }

    double getDebugXUsedForU() const { return debug_x_used_for_u_; }
    double getDebugVUsedForU() const { return debug_v_used_for_u_; }
    int getPredictionHorizon() const { return Np_steps; }
    double getKp() const { return K(0); }
    double getKv() const { return K(1); }
    double getObserverL0() const { return L(0); }
    double getObserverL1() const { return L(1); }

    double getAttackState(long long k_step) const
    {
        const long long step_in_cycle = ((k_step > 0) ? (k_step - 1) : 0) % DoS_Period_X;
        return (enable_internal_dos_ && step_in_cycle >= DoS_Peace_a) ? 1.0 : 0.0;
    }

    void reset()
    {
        u_history.clear();
        std::fill(Uc.begin(), Uc.end(), 0.0);
        std::fill(Ua.begin(), Ua.end(), 0.0);
        std::fill(Xc.begin(), Xc.end(), Eigen::Vector2d::Zero());
        std::fill(Xa.begin(), Xa.end(), Eigen::Vector2d::Zero());
        xhat_tau_.setZero();
    }

    // Reinitialize the measured state while retaining input and prediction queues.
    void softReset(double current_pos, double last_vel)
    {
        xhat_tau_(0) = current_pos;
        xhat_tau_(1) = last_vel;

    }

    // Main Compute Function
    void computeControl(Eigen::Vector2d &hat_x_aug,
                        double y_meas,
                        long long k_step,
                        bool sample_available,
                        bool use_bias,
                        int which_axis,
                        double u_applied,
                        double &u_out)
    {
        // 1) Apply bias
        const double y_biased = adjustBias(y_meas, which_axis, use_bias);

        const long long hist_idx = (k_step > 0) ? (k_step - 1) : 0;
        if (u_history.size() <= static_cast<size_t>(hist_idx))
            u_history.push_back(u_applied);
        else
            u_history[hist_idx] = u_applied;

        // 2) Observer & Prediction
        if (sample_available)
        {
            double y_bar = y_biased;
            std::vector<Eigen::Vector2d> temp_xhat_for_pred(Np_steps, Eigen::Vector2d::Zero());

            for (int n = 0; n < Np_steps; ++n)
            {
                const long long idx = (k_step > 0 ? (k_step - 1) : 0) + n;
                const double u_h = (idx < static_cast<long long>(u_history.size())) ? u_history[idx] : 0.0;
                const Eigen::Vector2d xhat_tau_bef = xhat_tau_;

                // Standard Observer
                const double y_est = (C_aug * xhat_tau_)(0);
                xhat_tau_ = Ad * xhat_tau_ + Bd * u_h + L * (y_bar - y_est);

                // Predict Measurement
                y_bar = (C_aug * (Ad * xhat_tau_bef + Bd * u_h))(0);
                temp_xhat_for_pred[n] = xhat_tau_;
            }

            hat_x_aug = temp_xhat_for_pred[0];

            // Compute candidate sequence Uc AND Xc
            for (int n = 0; n < Np_steps; ++n)
            {
                const Eigen::Vector2d &x_pred_n = temp_xhat_for_pred[n];
                Uc[n] = (K * x_pred_n)(0);
                Xc[n] = x_pred_n;
            }
        }
        else
        {
            if (!Uc.empty())
            {
                // Shift Uc
                const double last_uc = Uc.back();
                for (size_t i = 0; i + 1 < Uc.size(); ++i) Uc[i] = Uc[i + 1];
                Uc.back() = last_uc;

                // Shift Xc
                const Eigen::Vector2d last_xc = Xc.back();
                for (size_t i = 0; i + 1 < Xc.size(); ++i) Xc[i] = Xc[i + 1];
                Xc.back() = last_xc;
            }
        }

        // 3) DoS Logic
        const long long step_in_cycle = ((k_step > 0) ? (k_step - 1) : 0) % DoS_Period_X;
        const bool is_attack = enable_internal_dos_ && (step_in_cycle >= DoS_Peace_a);

        if (!is_attack)
        {
            Ua = Uc; Xa = Xc; // Peace
        }
        else
        {
            // Attack
            if (!Ua.empty())
            {
                const double last_ua = Ua.back();
                for (size_t i = 0; i + 1 < Ua.size(); ++i) Ua[i] = Ua[i + 1];
                Ua.back() = last_ua;

                const Eigen::Vector2d last_xa = Xa.back();
                for (size_t i = 0; i + 1 < Xa.size(); ++i) Xa[i] = Xa[i + 1];
                Xa.back() = last_xa;
            }
        }

        // 4) Output
        const double raw_u = Ua.empty() ? 0.0 : Ua[0];

        if (!Xa.empty())
        {
            debug_x_used_for_u_ = Xa[0](0);
            debug_v_used_for_u_ = Xa[0](1);
        }
        else
        {
            debug_x_used_for_u_ = 0.0;
            debug_v_used_for_u_ = 0.0;
        }

        u_out = limitControl(raw_u, which_axis);
    }
};

// ==========================================================================================
// Low Pass Filter
// ==========================================================================================
class LowPassFilter
{
private:
    double alpha_;
    double last_output_;
public:
    explicit LowPassFilter(double alpha_value) : alpha_(alpha_value), last_output_(0.0) {}
    double filter(double input)
    {
        const double output = alpha_ * input + (1.0 - alpha_) * last_output_;
        last_output_ = output;
        return output;
    }
};

// ==========================================================================================
// MyController (Single Axis State Machine)
// ==========================================================================================
class MyController
{
private:
    Eigen::Vector2d hat_x_aug_;
    PHBSCController phbsc_controller_;

    double u_;
    double u_last_;
    double y_real_;

    long long global_k_step_;
    int timer_count_;

    bool first_time_in_fun_;
    bool loss_target_;
    bool use_bias_;
    double loss_or_not_;
    int which_axis_;

    ros::NodeHandle nh_;
    ros::Subscriber px4_state_sub_;
    ros::Timer timer_;
    LowPassFilter filter_for_img_;

    int px4_state_;

    friend class TripleAxisController;

public:
    MyController()
        : hat_x_aug_(Eigen::Vector2d::Zero()), u_(0.0), u_last_(0.0), y_real_(0.0),
          global_k_step_(1), timer_count_(0), first_time_in_fun_(true), loss_target_(true),
          use_bias_(true), loss_or_not_(1.0), which_axis_(0), nh_("~"),
          filter_for_img_(0.95), px4_state_(1)
    {
        px4_state_sub_ = nh_.subscribe("/px4_state_pub", 1, &MyController::StateCallback, this, ros::TransportHints().tcpNoDelay());
    }

    double getDebugUsedX() const { return phbsc_controller_.getDebugXUsedForU(); }
    double getDebugUsedV() const { return phbsc_controller_.getDebugVUsedForU(); }

    double getAttackState() const
    {

        long long k = (global_k_step_ > 1) ? (global_k_step_ - 1) : 1;
        return phbsc_controller_.getAttackState(k);
    }

    void cal_single_axis_ctrl_input(double measure_single_axis, double loss_or_not, bool use_bias, int which_axis)
    {
        loss_or_not_ = loss_or_not;
        use_bias_ = use_bias;
        which_axis_ = which_axis;

        y_real_ = filter_for_img_.filter(measure_single_axis);

        timer_count_ = 0;
        function(loss_or_not_, use_bias_, which_axis_);

        timer_.stop();
        timer_ = nh_.createTimer(ros::Duration(0.01), &MyController::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        timer_count_++;
        if (timer_count_ >= 5) timer_.stop();
        else function(loss_or_not_, use_bias_, which_axis_);
    }

    void function(double loss_or_not, bool use_bias, int which_axis)
    {
        if (loss_or_not == 1.0 && loss_target_ == false) loss_target_ = true;
        if (loss_or_not == 0.0 && loss_target_ == true) { loss_target_ = false; first_time_in_fun_ = true; }

        if (px4_state_ != 3)
        {
            u_ = 0.0; u_last_ = 0.0;
            hat_x_aug_.setZero();
            phbsc_controller_.reset();
            global_k_step_ = 1;
            return;
        }

        if (first_time_in_fun_)
        {
            first_time_in_fun_ = false;

            double last_velocity = hat_x_aug_(1);

            const double y_for_state = phbsc_controller_.adjustBias(y_real_, which_axis, use_bias);
            hat_x_aug_(0) = y_for_state;    // biased error coordinate
            hat_x_aug_(1) = last_velocity;

            phbsc_controller_.softReset(y_for_state, last_velocity);

            global_k_step_ = 1;

            phbsc_controller_.computeControl(hat_x_aug_, y_real_, global_k_step_, true, use_bias, which_axis, u_last_, u_);
        }
        else
        {
            const bool sample_available = (timer_count_ == 0);
            phbsc_controller_.computeControl(hat_x_aug_, y_real_, global_k_step_, sample_available, use_bias, which_axis, u_last_, u_);
        }

        global_k_step_++;
        u_last_ = u_;
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        px4_state_ = msg->data;
        if (px4_state_ != 3)
        {
            u_ = 0.0; u_last_ = 0.0;
            hat_x_aug_.setZero();
            phbsc_controller_.reset();
            global_k_step_ = 1;
            first_time_in_fun_ = true;
        }
    }
};

// ==========================================================================================
// Three-axis controller with a fixed-rate command publisher
// ==========================================================================================
class TripleAxisController
{
private:
    MyController controllerX_, controllerY_, controllerZ_;
    ros::NodeHandle nh_;

    ros::Subscriber sub_;
    ros::Subscriber ground_truth_vel_sub_, ground_truth_pose_sub_;
    ros::Subscriber target_ground_truth_vel_sub_, target_ground_truth_pose_sub_;

    ros::Publisher pub_hat_x_, acc_cmd_pub_;
    quadrotor_msgs::PositionCommand acc_msg_;
    ros::Timer control_update_timer_;

    double gt_vx_ = 0, gt_vy_ = 0, gt_vz_ = 0;
    double gt_x_ = 0, gt_y_ = 0, gt_z_ = 0;
    double target_gt_vx_ = 0, target_gt_vy_ = 0, target_gt_vz_ = 0;
    double target_gt_x_ = 0, target_gt_y_ = 0, target_gt_z_ = 0;
    double vision_error_x_ = 0, vision_error_y_ = 0, vision_error_z_ = 0;

    std::array<double, 3> hold_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> last_sent_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> candidate_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> nominal_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> packet_compute_time_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> u_max_ = {{0.3, 6.0, 6.0}};
    std::array<double, 3> axis_bias_ = {{-0.9, 0.0, 0.0}};
    std::array<double, 3> target_mocap_comp_ = {{-0.026, 0.127, -0.117}};
    std::array<double, 3> uav_mocap_log_comp_ = {{0.0, -0.122, 0.038}};

    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_;

    double dt_ = 0.01;
    double fdi_amp_ = 0.10;
    double fdi_prob_ = 0.02;
    double dos_prob_ = 0.05;
    double fdi_noise_ = 0.0;
    double delta_ = 0.75;
    double mu_ = 0.55;
    double gate_qe_ = 1.80;
    double gate_qv_ = 0.45;
    double gate_qd_ = 0.04;
    double smart_qe_ = 0.0;
    double smart_qv_ = 0.0;
    double smart_qd_ = 0.0;
    double smart_re_ = 0.0;
    double smart_rv_ = 0.0;
    bool attack_sim_enabled_ = true;
    bool dos_active_ = false;
    bool fdi_active_ = false;
    bool external_fdi_request_ = false;
    bool target_lost_ = true;
    bool valid_visual_seen_ = false;
    bool controller_initialized_ = false;
    bool used_hold_ = true;
    int dos_period_steps_ = 80;
    int dos_active_steps_ = 4;
    int horizon_ = 5;
    int target_loss_hold_steps_ = 120;
    int target_lost_steps_ = 0;
    int attack_seed_ = 7;
    int attack_packet_slot_ = 0;
    unsigned long long attack_packet_sequence_ = 1;
    unsigned long long attack_packet_interval_ = 0;
    unsigned long long step_ = 0;

public:
    TripleAxisController() : nh_("~"), rng_(7), uniform_(0.0, 1.0)
    {
        loadParams();
        sub_ = nh_.subscribe("/point_with_fixed_delay", 1, &TripleAxisController::callback, this, ros::TransportHints().tcpNoDelay());

        ground_truth_vel_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/velocity", 10, &TripleAxisController::ground_truth_vel_callback, this);
        ground_truth_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/pose", 10, &TripleAxisController::ground_truth_pose_callback, this);
        target_ground_truth_vel_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/velocity", 10, &TripleAxisController::target_ground_truth_vel_callback, this);
        target_ground_truth_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/pose", 10, &TripleAxisController::target_ground_truth_pose_callback, this);

        pub_hat_x_ = nh_.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        acc_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);

        control_update_timer_ = nh_.createTimer(ros::Duration(0.01), &TripleAxisController::controlUpdate, this);
    }

    void loadParams()
    {
        nh_.param("dt", dt_, 0.01);
        nh_.param("horizon", horizon_, 5);
        nh_.param("fdi_amp", fdi_amp_, 0.10);
        nh_.param("fdi_prob", fdi_prob_, 0.02);
        nh_.param("dos_prob", dos_prob_, 0.05);
        nh_.param("attack_sim_enabled", attack_sim_enabled_, true);
        nh_.param("dos_period_steps", dos_period_steps_, 80);
        nh_.param("dos_active_steps", dos_active_steps_, 4);
        nh_.param("target_loss_hold_steps", target_loss_hold_steps_, 120);
        nh_.param("u_max_x", u_max_[0], 0.3);
        nh_.param("u_max_y", u_max_[1], 6.0);
        nh_.param("u_max_z", u_max_[2], 6.0);
        nh_.param("gate_delta", delta_, 0.75);
        nh_.param("gate_mu", mu_, 0.55);
        nh_.param("gate_qe", gate_qe_, 1.80);
        nh_.param("gate_qv", gate_qv_, 0.45);
        nh_.param("gate_qd", gate_qd_, 0.04);
        nh_.param("target_mocap_x_comp", target_mocap_comp_[0], -0.026);
        nh_.param("target_mocap_y_comp", target_mocap_comp_[1], 0.127);
        nh_.param("target_mocap_z_comp", target_mocap_comp_[2], -0.117);
        nh_.param("uav_mocap_log_x_comp", uav_mocap_log_comp_[0], 0.0);
        nh_.param("uav_mocap_log_y_comp", uav_mocap_log_comp_[1], -0.122);
        nh_.param("uav_mocap_log_z_comp", uav_mocap_log_comp_[2], 0.038);
        nh_.param("attack_seed", attack_seed_, 7);
        fdi_prob_ = std::max(0.0, std::min(1.0, fdi_prob_));
        dos_prob_ = std::max(0.0, std::min(1.0, dos_prob_));
        rng_.seed(static_cast<unsigned int>(attack_seed_));
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

    double deceptionCommand(double nominal_u, double error, double velocity, int axis) const
    {
        const double attack = clampValue(fdi_noise_, fdi_amp_);
        const double dir = signNonzero(error + 0.25 * velocity + 1.0e-6);
        return clampValue(nominal_u + dir * attack, u_max_[axis]);
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() < 5)
        {
            target_lost_ = true;
            return;
        }

        target_lost_ = msg->data[4] > 0.5;
        external_fdi_request_ = msg->data.size() >= 7 && std::fabs(msg->data[6]) > 0.5;

        if (target_lost_)
        {
            return;
        }

        valid_visual_seen_ = true;
        controller_initialized_ = true;
        target_lost_steps_ = 0;
        vision_error_x_ = msg->data[0];
        vision_error_y_ = msg->data[1];
        vision_error_z_ = msg->data[2];
        controllerX_.cal_single_axis_ctrl_input(msg->data[0], msg->data[4], true, 0);
        controllerY_.cal_single_axis_ctrl_input(msg->data[1], msg->data[4], false, 1);
        controllerZ_.cal_single_axis_ctrl_input(msg->data[2], msg->data[4], true, 2);
    }

    void fillAxisDebug(std::vector<double> &data, int base, int axis,
                       const MyController &controller, double vision_error,
                       double gt_pos, double gt_vel, double target_pos, double target_vel) const
    {
        const double local_e = target_pos - gt_pos;
        const double local_v = target_vel - gt_vel;
        data[base + 0] = controller.hat_x_aug_(0);
        data[base + 1] = controller.hat_x_aug_(1);
        data[base + 2] = last_sent_u_[axis];
        data[base + 3] = vision_error + axis_bias_[axis];
        data[base + 4] = gt_pos;
        data[base + 5] = gt_vel;
        data[base + 6] = target_pos;
        data[base + 7] = target_vel;
        data[base + 8] = local_e;
        data[base + 9] = local_v;
        data[base + 10] = 0.0;
        data[base + 11] = 0.0;
        data[base + 12] = std::fabs(candidate_u_[axis] - nominal_u_[axis]);
        data[base + 13] = dos_active_ ? 0.0 : 1.0;
        data[base + 14] = candidate_u_[axis];
        data[base + 15] = controller.getDebugUsedX();
        data[base + 16] = controller.getDebugUsedV();
        data[base + 17] = 0.0;
        data[base + 18] = dos_active_ ? 1.0 : 0.0;
        data[base + 19] = fdi_active_ ? 1.0 : 0.0;
        data[base + 20] = static_cast<double>(horizon_);
        data[base + 21] = static_cast<double>(horizon_);
        data[base + 22] = used_hold_ ? 1.0 : 0.0;
        data[base + 23] = last_sent_u_[axis];
    }

    void publishDebug(double elapsed)
    {
        std_msgs::Float64MultiArray dbg;
        dbg.data.assign(155, 0.0);

        fillAxisDebug(dbg.data, 0, 0, controllerX_, vision_error_x_, gt_x_, gt_vx_, target_gt_x_, target_gt_vx_);
        fillAxisDebug(dbg.data, 24, 1, controllerY_, vision_error_y_, gt_y_, gt_vy_, target_gt_y_, target_gt_vy_);
        fillAxisDebug(dbg.data, 48, 2, controllerZ_, vision_error_z_, gt_z_, gt_vz_, target_gt_z_, target_gt_vz_);

        dbg.data[72] = static_cast<double>(step_);
        dbg.data[73] = elapsed;
        dbg.data[74] = (target_lost_ || dos_active_ || fdi_active_) ? 1.0 : 0.0;
        dbg.data[75] = static_cast<double>(horizon_);
        dbg.data[76] = delta_;
        dbg.data[77] = mu_;
        dbg.data[78] = static_cast<double>(horizon_);
        dbg.data[79] = static_cast<double>(controllerX_.px4_state_);
        dbg.data[80] = used_hold_ ? 0.0 : 1.0;
        dbg.data[81] = dos_active_ ? 0.0 : 1.0;
        dbg.data[82] = target_lost_ ? 1.0 : 0.0;
        dbg.data[83] = valid_visual_seen_ ? 1.0 : 0.0;
        dbg.data[84] = controller_initialized_ ? 1.0 : 0.0;
        dbg.data[85] = static_cast<double>(target_lost_steps_);
        dbg.data[86] = static_cast<double>(target_loss_hold_steps_);
        dbg.data[87] = attack_sim_enabled_ ? 1.0 : 0.0;
        dbg.data[88] = external_fdi_request_ ? 1.0 : 0.0;
        dbg.data[89] = fdi_noise_;
        dbg.data[90] = static_cast<double>(dos_period_steps_);
        dbg.data[91] = static_cast<double>(dos_active_steps_);
        dbg.data[92] = fdi_prob_;
        dbg.data[93] = fdi_amp_;
        dbg.data[94] = 0.0;
        dbg.data[95] = dt_;
        dbg.data[96] = u_max_[0];
        dbg.data[97] = u_max_[1];
        dbg.data[98] = u_max_[2];
        dbg.data[99] = controllerX_.phbsc_controller_.getKp();
        dbg.data[100] = controllerX_.phbsc_controller_.getKv();
        dbg.data[101] = controllerX_.phbsc_controller_.getObserverL0();
        dbg.data[102] = controllerX_.phbsc_controller_.getObserverL1();
        dbg.data[103] = 0.0;
        dbg.data[104] = gate_qe_;
        dbg.data[105] = gate_qv_;
        dbg.data[106] = gate_qd_;
        dbg.data[107] = smart_qe_;
        dbg.data[108] = smart_qv_;
        dbg.data[109] = smart_qd_;
        dbg.data[110] = smart_re_;
        dbg.data[111] = smart_rv_;
        dbg.data[112] = packet_compute_time_[0];
        dbg.data[113] = packet_compute_time_[1];
        dbg.data[114] = packet_compute_time_[2];
        dbg.data[115] = packet_compute_time_[0] + packet_compute_time_[1] + packet_compute_time_[2];
        dbg.data[116] = axis_bias_[0];
        dbg.data[117] = axis_bias_[1];
        dbg.data[118] = axis_bias_[2];
        dbg.data[119] = target_mocap_comp_[0];
        dbg.data[120] = target_mocap_comp_[1];
        dbg.data[121] = target_mocap_comp_[2];
        dbg.data[122] = target_gt_x_ - gt_x_;
        dbg.data[123] = target_gt_y_ - gt_y_;
        dbg.data[124] = target_gt_z_ - gt_z_;
        dbg.data[125] = target_gt_vx_ - gt_vx_;
        dbg.data[126] = target_gt_vy_ - gt_vy_;
        dbg.data[127] = target_gt_vz_ - gt_vz_;
        dbg.data[128] = static_cast<double>(attack_seed_);
        dbg.data[129] = 3.0;
        dbg.data[130] = 1.0;
        dbg.data[131] = dos_prob_;
        dbg.data[132] = 3.0;
        dbg.data[133] = ros::Time::now().toSec();
        dbg.data[134] = vision_error_x_;
        dbg.data[135] = vision_error_y_;
        dbg.data[136] = vision_error_z_;
        dbg.data[137] = target_gt_x_ - gt_x_ + target_mocap_comp_[0] + uav_mocap_log_comp_[0];
        dbg.data[138] = target_gt_y_ - gt_y_ + target_mocap_comp_[1] + uav_mocap_log_comp_[1];
        dbg.data[139] = target_gt_z_ - gt_z_ + target_mocap_comp_[2] + uav_mocap_log_comp_[2];
        dbg.data[140] = nominal_u_[0];
        dbg.data[141] = nominal_u_[1];
        dbg.data[142] = nominal_u_[2];
        dbg.data[143] = hold_u_[0];
        dbg.data[144] = hold_u_[1];
        dbg.data[145] = hold_u_[2];
        dbg.data[146] = dos_active_ ? 0.0 : candidate_u_[0] - nominal_u_[0];
        dbg.data[147] = dos_active_ ? 0.0 : candidate_u_[1] - nominal_u_[1];
        dbg.data[148] = dos_active_ ? 0.0 : candidate_u_[2] - nominal_u_[2];
        dbg.data[149] = static_cast<double>(attack_packet_slot_);
        dbg.data[150] = static_cast<double>(attack_packet_sequence_);
        dbg.data[151] = static_cast<double>(attack_packet_interval_);
        dbg.data[152] = uav_mocap_log_comp_[0];
        dbg.data[153] = uav_mocap_log_comp_[1];
        dbg.data[154] = uav_mocap_log_comp_[2];

        pub_hat_x_.publish(dbg);
    }

    void controlUpdate(const ros::TimerEvent &)
    {
        const auto attack_tic = std::chrono::high_resolution_clock::now();

        const bool new_attack_packet = attack_packet_slot_ == 0;
        if (new_attack_packet)
        {
            updateAttackFlags();
            fdi_noise_ = fdi_active_ ? randomSigned(fdi_amp_) : 0.0;
        }

        nominal_u_[0] = controllerX_.u_;
        nominal_u_[1] = controllerY_.u_;
        nominal_u_[2] = controllerZ_.u_;
        candidate_u_ = nominal_u_;

        const auto packet_tic = std::chrono::high_resolution_clock::now();
        if (fdi_active_ && !dos_active_)
        {
            candidate_u_[0] = deceptionCommand(nominal_u_[0], vision_error_x_, controllerX_.hat_x_aug_(1), 0);
            candidate_u_[1] = deceptionCommand(nominal_u_[1], vision_error_y_, controllerY_.hat_x_aug_(1), 1);
            candidate_u_[2] = deceptionCommand(nominal_u_[2], vision_error_z_, controllerZ_.hat_x_aug_(1), 2);
        }
        const auto packet_toc = std::chrono::high_resolution_clock::now();
        const double packet_elapsed = std::chrono::duration<double>(packet_toc - packet_tic).count();
        packet_compute_time_[0] = packet_elapsed / 3.0;
        packet_compute_time_[1] = packet_elapsed / 3.0;
        packet_compute_time_[2] = packet_elapsed / 3.0;

        if (target_lost_)
        {
            ++target_lost_steps_;
        }

        const bool controls_enabled =
            controllerX_.px4_state_ == 3 && valid_visual_seen_ &&
            !(target_lost_ && target_lost_steps_ > target_loss_hold_steps_);
        if (!controls_enabled)
        {
            fdi_noise_ = 0.0;
            dos_active_ = false;
            fdi_active_ = false;
            attack_packet_slot_ = 0;
            attack_packet_sequence_ = 1;
            attack_packet_interval_ = 0;
            candidate_u_ = {{0.0, 0.0, 0.0}};
            last_sent_u_ = candidate_u_;
            hold_u_ = candidate_u_;
            used_hold_ = true;
            if (controllerX_.px4_state_ != 3)
            {
                controller_initialized_ = false;
                valid_visual_seen_ = false;
            }
        }
        else if (dos_active_)
        {
            last_sent_u_ = hold_u_;
            used_hold_ = true;
        }
        else
        {
            last_sent_u_ = candidate_u_;
            hold_u_ = last_sent_u_;
            used_hold_ = false;
        }

        acc_msg_.header.stamp = ros::Time::now();
        acc_msg_.header.frame_id = "world";
        acc_msg_.position.x = 0.0;
        acc_msg_.position.y = 0.0;
        acc_msg_.position.z = 0.0;
        acc_msg_.velocity.x = 0.0;
        acc_msg_.velocity.y = 0.0;
        acc_msg_.velocity.z = 0.0;
        acc_msg_.acceleration.x = last_sent_u_[0];
        acc_msg_.acceleration.y = last_sent_u_[1];
        acc_msg_.acceleration.z = last_sent_u_[2];
        acc_msg_.jerk.x = 0.0;
        acc_msg_.jerk.y = 0.0;
        acc_msg_.jerk.z = 0.0;
        acc_msg_.yaw = 0.0;
        acc_msg_.yaw_dot = 0.0;

        acc_cmd_pub_.publish(acc_msg_);

        const auto attack_toc = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<double> elapsed = attack_toc - attack_tic;
        publishDebug(elapsed.count());
        if (controls_enabled)
        {
            ++attack_packet_slot_;
            if (attack_packet_slot_ >= kAttackPacketLength)
            {
                attack_packet_slot_ = 0;
                ++attack_packet_sequence_;
                ++attack_packet_interval_;
            }
        }
        ++step_;
        return;

    }
    void ground_truth_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        gt_vx_ = msg->twist.linear.x; gt_vy_ = msg->twist.linear.y; gt_vz_ = msg->twist.linear.z;
    }
    void ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        gt_x_ = msg->pose.position.x; gt_y_ = msg->pose.position.y; gt_z_ = msg->pose.position.z;
    }
    void target_ground_truth_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        target_gt_vx_ = msg->twist.linear.x; target_gt_vy_ = msg->twist.linear.y; target_gt_vz_ = msg->twist.linear.z;
    }
    void target_ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        target_gt_x_ = msg->pose.position.x; target_gt_y_ = msg->pose.position.y; target_gt_z_ = msg->pose.position.z;
    }

    void spin() { ros::spin(); }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_controller_node");
    TripleAxisController controller;
    controller.spin();
    return 0;
}
