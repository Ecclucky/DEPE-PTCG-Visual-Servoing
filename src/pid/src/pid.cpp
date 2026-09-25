#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <cmath>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <Iir.h>
#include <chrono>
#include <algorithm>
#include <array>
#include <random>

using namespace std;

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

// ==========================================================================================
// PID baseline used in the comparative flight experiments.
class PID2Controller
{
private:
    const double k_i = 0.3;
    const double k_p = 4;
    const double k_d = 2.9;
    const double T_c = 0.05;
    double x_bias = -0.9;
    double y_bias = 0;
    double z_bias = -0.1;
    double y1_real_bias = 0;
    ros::NodeHandle nh;

public:
    PID2Controller() {}

    double adjustBias(double value, int which_axis, bool use_bias)
    {
        if (use_bias)
        {
            if (which_axis == 0) return value + x_bias;
            else if (which_axis == 1) return value + y_bias;
            else return value + z_bias;
        }
        return value;
    }

    double limitControl(double controlValue, int which_axis)
    {
        double limit = 5;
        if (abs(controlValue) >= limit)
        {
            controlValue = limit * controlValue / abs(controlValue);
        }
        return controlValue;
    }

    double limitIntegral(double IntegralValue, int which_axis)
    {
        double limit = 1;
        if (abs(IntegralValue) >= limit)
        {
            IntegralValue = limit * IntegralValue / abs(IntegralValue);
        }
        return IntegralValue;
    }

    void computeControl(double y1_real_slow, double y2_derivative_sampling, double &u, double &integral_error, bool use_bias, int which_axis)
    {
        y1_real_bias = adjustBias(y1_real_slow, which_axis, use_bias);
        integral_error = integral_error + y1_real_bias * T_c;
        integral_error = limitIntegral(integral_error, which_axis);
        u = k_i * integral_error + k_p * y1_real_bias + k_d * y2_derivative_sampling;
        u = limitControl(u, which_axis);
    }
};

class LowPassFilter
{
private:
    double alpha;
    double last_output;
public:
    LowPassFilter(double alpha_value) : alpha(alpha_value), last_output(0.0) {}
    double filter(double input)
    {
        double output = alpha * input + (1 - alpha) * last_output;
        last_output = output;
        return output;
    }
};

// ==========================================================================================

// ==========================================================================================
class MyController
{
private:
    double u, integral_error;
    double y_real, y_real_last;
    double y_real_derivative, y_filtered_deri;
    double time_now, time_last, time_pass;
    bool first_time_in_fun, loss_target, use_bias_;
    ros::NodeHandle nh;

    LowPassFilter filter_for_img;
    PID2Controller pid2controller;
    ros::Subscriber px4_state_sub;
    friend // Publish the most recent command at the fixed actuator update rate.
class TripleAxisController;
    double loss_or_not_;
    int which_axis_;
    int px4_state;
    Iir::Butterworth::LowPass<2> filter_4_for_deri;

    long long global_k_step_;

public:
    MyController()
        : nh("~"), px4_state(1), y_real(0.0), u(0.0), y_real_last(0.0), y_real_derivative(0.0),
          time_now(0.0), time_last(0.0), time_pass(0.0), filter_for_img(0.95), y_filtered_deri(0),
          integral_error(0), loss_target(true), loss_or_not_(1), use_bias_(1), which_axis_(0),
          first_time_in_fun(true), global_k_step_(1)
    {
        px4_state_sub = nh.subscribe("/px4_state_pub", 1, &MyController::StateCallback, this);
        filter_4_for_deri.setup(20, 2.7);
    }

    void cal_single_axis_ctrl_input(double measure_single_axis, double loss_or_not, bool use_bias, int which_axis)
    {
        loss_or_not_ = loss_or_not;
        use_bias_ = use_bias;
        which_axis_ = which_axis;
        y_real = measure_single_axis;
        y_real = filter_for_img.filter(y_real);
        time_now = ros::Time::now().toSec();
        time_pass = time_now - time_last;
        if (time_pass <= 0) time_pass = 0.05;

        y_real_derivative = (y_real - y_real_last) / time_pass;
        y_filtered_deri = filter_4_for_deri.filter(y_real_derivative);

        function(loss_or_not_, use_bias_, which_axis_);

        y_real_last = y_real;
        time_last = time_now;
    }

    void function(double loss_or_not, bool use_bias, int which_axis)
    {
        if (loss_or_not == 1 && loss_target == false) loss_target = true;
        if (loss_or_not == 0 && loss_target == true) { loss_target = false; first_time_in_fun = true; }

        if (first_time_in_fun)
        {
            time_pass = 0.05;
            y_filtered_deri = 0;
            first_time_in_fun = false;
            pid2controller.computeControl(y_real, y_filtered_deri, u, integral_error, use_bias, which_axis);
            global_k_step_ = 1;
        }
        else
        {
            pid2controller.computeControl(y_real, y_filtered_deri, u, integral_error, use_bias, which_axis);
        }
        global_k_step_++;
    }

    void StateCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        px4_state = msg->data;
        if (msg->data != 3) { u = 0; integral_error = 0; global_k_step_ = 1; }
    }
};

// ==========================================================================================

// ==========================================================================================
// Publish the most recent command at the fixed actuator update rate.
class TripleAxisController
{
private:
    MyController controllerX, controllerY, controllerZ;
    ros::NodeHandle nh;
    ros::Subscriber sub, ground_truth_sub, ground_truth_pose_sub;
    ros::Subscriber target_ground_truth_vel_sub, target_ground_truth_pose_sub;
    ros::Publisher pub_hat_x, acc_cmd_pub, x_pub;
    quadrotor_msgs::PositionCommand acc_msg;
    ros::Timer control_update_timer;
    double des_yaw;
    double ground_truth_first_deri_x = 0, ground_truth_first_deri_y = 0, ground_truth_first_deri_z = 0;
    double ground_truth_x = 0, ground_truth_y = 0, ground_truth_z = 0;
    double target_gt_vx_ = 0, target_gt_vy_ = 0, target_gt_vz_ = 0;
    double target_gt_x_ = 0, target_gt_y_ = 0, target_gt_z_ = 0;
    double vision_error_x_ = 0, vision_error_y_ = 0, vision_error_z_ = 0;
    std::array<double, 3> hold_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> last_sent_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> candidate_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> nominal_u_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> u_max_ = {{5.0, 5.0, 5.0}};
    std::array<double, 3> axis_bias_ = {{-0.9, 0.0, -0.1}};
    std::array<double, 3> target_mocap_comp_ = {{-0.026, 0.127, -0.117}};
    std::array<double, 3> uav_mocap_log_comp_ = {{0.0, -0.122, 0.038}};
    std::array<double, 3> packet_compute_time_ = {{0.0, 0.0, 0.0}};
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_;
    double dt_ = 0.01;
    double fdi_amp_ = 0.10;
    double fdi_prob_ = 0.02;
    double dos_prob_ = 0.05;
    double fdi_noise_ = 0.0;
    double delta_ = 0.75;
    double mu_ = 0.55;
    double kp_ = 4.0;
    double kv_ = 2.9;
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
    int horizon_ = 20;
    int target_loss_hold_steps_ = 120;
    int target_lost_steps_ = 0;
    int attack_seed_ = 7;
    int attack_packet_slot_ = 0;
    unsigned long long attack_packet_sequence_ = 1;
    unsigned long long attack_packet_interval_ = 0;
    unsigned long long step_ = 0;

public:
    TripleAxisController() : nh("~"), des_yaw(0), rng_(7), uniform_(0.0, 1.0)
    {
        loadParams();
        x_pub = nh.advertise<std_msgs::Float64>("/input_x_axis", 100);

        sub = nh.subscribe("/point_with_fixed_delay", 1, &TripleAxisController::callback, this);

        ground_truth_sub = nh.subscribe("/vrpn_client_node/YZU0/0/velocity", 10, &TripleAxisController::ground_truth_callback, this);
        ground_truth_pose_sub = nh.subscribe("/vrpn_client_node/YZU0/0/pose", 10, &TripleAxisController::ground_truth_pose_callback, this);
        target_ground_truth_vel_sub = nh.subscribe("/vrpn_client_node/YZU1/0/velocity", 10, &TripleAxisController::target_ground_truth_vel_callback, this);
        target_ground_truth_pose_sub = nh.subscribe("/vrpn_client_node/YZU1/0/pose", 10, &TripleAxisController::target_ground_truth_pose_callback, this);

        pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);

        control_update_timer = nh.createTimer(ros::Duration(0.01), &TripleAxisController::controlUpdate, this);
    }

    void loadParams()
    {
        nh.param("dt", dt_, 0.01);
        nh.param("horizon", horizon_, 20);
        nh.param("fdi_amp", fdi_amp_, 0.10);
        nh.param("fdi_prob", fdi_prob_, 0.02);
        nh.param("dos_prob", dos_prob_, 0.05);
        nh.param("attack_sim_enabled", attack_sim_enabled_, true);
        nh.param("dos_period_steps", dos_period_steps_, 80);
        nh.param("dos_active_steps", dos_active_steps_, 4);
        nh.param("target_loss_hold_steps", target_loss_hold_steps_, 120);
        nh.param("u_max_x", u_max_[0], 5.0);
        nh.param("u_max_y", u_max_[1], 5.0);
        nh.param("u_max_z", u_max_[2], 5.0);
        nh.param("target_mocap_x_comp", target_mocap_comp_[0], -0.026);
        nh.param("target_mocap_y_comp", target_mocap_comp_[1], 0.127);
        nh.param("target_mocap_z_comp", target_mocap_comp_[2], -0.117);
        nh.param("uav_mocap_log_x_comp", uav_mocap_log_comp_[0], 0.0);
        nh.param("uav_mocap_log_y_comp", uav_mocap_log_comp_[1], -0.122);
        nh.param("uav_mocap_log_z_comp", uav_mocap_log_comp_[2], 0.038);
        nh.param("attack_seed", attack_seed_, 7);
        fdi_prob_ = std::max(0.0, std::min(1.0, fdi_prob_));
        dos_prob_ = std::max(0.0, std::min(1.0, dos_prob_));
        rng_.seed(static_cast<unsigned int>(attack_seed_));
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() < 5)
        {
            target_lost_ = true;
            return;
        }

        des_yaw = 0;
        if (msg->data.size() >= 6)
        {
            des_yaw = msg->data[5];
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

        controllerX.cal_single_axis_ctrl_input(msg->data[0], msg->data[4], 1, 0);
        controllerY.cal_single_axis_ctrl_input(msg->data[1], msg->data[4], 0, 1);
        controllerZ.cal_single_axis_ctrl_input(msg->data[2], msg->data[4], 1, 2);
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

    void fillAxisDebug(std::vector<double> &data, int base, int axis,
                       const MyController &controller, double vision_error,
                       double gt_pos, double gt_vel, double target_pos, double target_vel) const
    {
        const double local_e = target_pos - gt_pos;
        const double local_v = target_vel - gt_vel;
        data[base + 0] = controller.y_real;
        data[base + 1] = controller.y_filtered_deri;
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
        data[base + 13] = 1.0;
        data[base + 14] = candidate_u_[axis];
        data[base + 15] = controller.y_real;
        data[base + 16] = controller.y_filtered_deri;
        data[base + 17] = 0.0;
        data[base + 18] = dos_active_ ? 1.0 : 0.0;
        data[base + 19] = fdi_active_ ? 1.0 : 0.0;
        data[base + 20] = 1.0;
        data[base + 21] = 1.0;
        data[base + 22] = used_hold_ ? 1.0 : 0.0;
        data[base + 23] = last_sent_u_[axis];
    }

    void publishDebug(double elapsed)
    {
        std_msgs::Float64MultiArray dbg;
        dbg.data.assign(155, 0.0);

        fillAxisDebug(dbg.data, 0, 0, controllerX, vision_error_x_, ground_truth_x, ground_truth_first_deri_x, target_gt_x_, target_gt_vx_);
        fillAxisDebug(dbg.data, 24, 1, controllerY, vision_error_y_, ground_truth_y, ground_truth_first_deri_y, target_gt_y_, target_gt_vy_);
        fillAxisDebug(dbg.data, 48, 2, controllerZ, vision_error_z_, ground_truth_z, ground_truth_first_deri_z, target_gt_z_, target_gt_vz_);

        dbg.data[72] = static_cast<double>(step_);
        dbg.data[73] = elapsed;
        dbg.data[74] = (target_lost_ || dos_active_ || fdi_active_) ? 1.0 : 0.0;
        dbg.data[75] = static_cast<double>(horizon_);
        dbg.data[76] = delta_;
        dbg.data[77] = mu_;
        dbg.data[78] = 1.0;
        dbg.data[79] = static_cast<double>(controllerX.px4_state);
        dbg.data[80] = dos_active_ ? 0.0 : 1.0;
        dbg.data[81] = 1.0;
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
        dbg.data[94] = des_yaw;
        dbg.data[95] = dt_;
        dbg.data[96] = u_max_[0];
        dbg.data[97] = u_max_[1];
        dbg.data[98] = u_max_[2];
        dbg.data[99] = kp_;
        dbg.data[100] = kv_;
        dbg.data[101] = 0.0;
        dbg.data[102] = 0.0;
        dbg.data[103] = 0.0;
        dbg.data[104] = 0.0;
        dbg.data[105] = 0.0;
        dbg.data[106] = 0.0;
        dbg.data[107] = 0.0;
        dbg.data[108] = 0.0;
        dbg.data[109] = 0.0;
        dbg.data[110] = 0.0;
        dbg.data[111] = 0.0;
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
        dbg.data[122] = target_gt_x_ - ground_truth_x;
        dbg.data[123] = target_gt_y_ - ground_truth_y;
        dbg.data[124] = target_gt_z_ - ground_truth_z;
        dbg.data[125] = target_gt_vx_ - ground_truth_first_deri_x;
        dbg.data[126] = target_gt_vy_ - ground_truth_first_deri_y;
        dbg.data[127] = target_gt_vz_ - ground_truth_first_deri_z;
        dbg.data[128] = static_cast<double>(attack_seed_);
        dbg.data[129] = 3.0;
        dbg.data[130] = 1.0;
        dbg.data[131] = dos_prob_;
        dbg.data[132] = 4.0;
        dbg.data[133] = ros::Time::now().toSec();
        dbg.data[134] = vision_error_x_;
        dbg.data[135] = vision_error_y_;
        dbg.data[136] = vision_error_z_;
        dbg.data[137] = target_gt_x_ - ground_truth_x + target_mocap_comp_[0] + uav_mocap_log_comp_[0];
        dbg.data[138] = target_gt_y_ - ground_truth_y + target_mocap_comp_[1] + uav_mocap_log_comp_[1];
        dbg.data[139] = target_gt_z_ - ground_truth_z + target_mocap_comp_[2] + uav_mocap_log_comp_[2];
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

        pub_hat_x.publish(dbg);
    }

    // Apply the configured packet attack and publish the held or current command.
    void controlUpdate(const ros::TimerEvent &)
    {

        auto start_time = std::chrono::high_resolution_clock::now();

        std_msgs::Float64 input_data_msg;
        input_data_msg.data = controllerX.u;
        x_pub.publish(input_data_msg);

        const bool new_attack_packet = attack_packet_slot_ == 0;
        if (new_attack_packet)
        {
            updateAttackFlags();
            fdi_noise_ = fdi_active_ ? randomSigned(fdi_amp_) : 0.0;
        }
        nominal_u_[0] = controllerX.u;
        nominal_u_[1] = controllerY.u;
        nominal_u_[2] = controllerZ.u;
        candidate_u_ = nominal_u_;

        if (target_lost_)
        {
            ++target_lost_steps_;
        }

        const bool controls_enabled = controllerX.px4_state == 3;
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
        }
        else
        {
            if (fdi_active_ && !dos_active_)
            {
                candidate_u_[0] = deceptionCommand(nominal_u_[0], vision_error_x_, controllerX.y_filtered_deri, 0);
                candidate_u_[1] = deceptionCommand(nominal_u_[1], vision_error_y_, controllerY.y_filtered_deri, 1);
                candidate_u_[2] = deceptionCommand(nominal_u_[2], vision_error_z_, controllerZ.y_filtered_deri, 2);
            }

            if (dos_active_)
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
        }

        acc_msg.acceleration.x = last_sent_u_[0];
        acc_msg.acceleration.y = last_sent_u_[1];
        acc_msg.acceleration.z = last_sent_u_[2];
        acc_msg.header.frame_id = "world";
        acc_msg.header.stamp = ros::Time::now();
        acc_cmd_pub.publish(acc_msg);
        auto aligned_debug_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> aligned_debug_elapsed = aligned_debug_end_time - start_time;
        publishDebug(aligned_debug_elapsed.count());
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

    void ground_truth_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        ground_truth_first_deri_x = msg->twist.linear.x;
        ground_truth_first_deri_y = msg->twist.linear.y;
        ground_truth_first_deri_z = msg->twist.linear.z;
    }

    void ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ground_truth_x = msg->pose.position.x;
        ground_truth_y = msg->pose.position.y;
        ground_truth_z = msg->pose.position.z;
    }

    void target_ground_truth_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        target_gt_vx_ = msg->twist.linear.x;
        target_gt_vy_ = msg->twist.linear.y;
        target_gt_vz_ = msg->twist.linear.z;
    }

    void target_ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        target_gt_x_ = msg->pose.position.x;
        target_gt_y_ = msg->pose.position.y;
        target_gt_z_ = msg->pose.position.z;
    }

    void spin() { while (ros::ok()) { ros::spin(); } }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_controller_node");
    TripleAxisController controller;
    controller.spin();
    return 0;
}
