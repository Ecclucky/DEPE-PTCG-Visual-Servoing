#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <string>
using namespace std;

namespace
{
enum
{
    kDebugDataSize = 155
};

const char *const kHeaders[kDebugDataSize] = {
    "x_remote_e",
    "x_remote_v",
    "x_u",
    "x_vision_error",
    "x_gt_pos",
    "x_gt_vel",
    "x_target_gt_pos",
    "x_target_gt_vel",
    "x_smart_e",
    "x_smart_v",
    "x_smart_d",
    "x_remote_d",
    "x_gate_metric",
    "x_gate_pass",
    "x_candidate_u",
    "x_candidate_e",
    "x_candidate_v",
    "x_candidate_d",
    "x_dos_active",
    "x_fdi_active",
    "x_candidate_len",
    "x_buffer_len",
    "x_used_hold",
    "x_last_control",
    "y_remote_e",
    "y_remote_v",
    "y_u",
    "y_vision_error",
    "y_gt_pos",
    "y_gt_vel",
    "y_target_gt_pos",
    "y_target_gt_vel",
    "y_smart_e",
    "y_smart_v",
    "y_smart_d",
    "y_remote_d",
    "y_gate_metric",
    "y_gate_pass",
    "y_candidate_u",
    "y_candidate_e",
    "y_candidate_v",
    "y_candidate_d",
    "y_dos_active",
    "y_fdi_active",
    "y_candidate_len",
    "y_buffer_len",
    "y_used_hold",
    "y_last_control",
    "z_remote_e",
    "z_remote_v",
    "z_u",
    "z_vision_error",
    "z_gt_pos",
    "z_gt_vel",
    "z_target_gt_pos",
    "z_target_gt_vel",
    "z_smart_e",
    "z_smart_v",
    "z_smart_d",
    "z_remote_d",
    "z_gate_metric",
    "z_gate_pass",
    "z_candidate_u",
    "z_candidate_e",
    "z_candidate_v",
    "z_candidate_d",
    "z_dos_active",
    "z_fdi_active",
    "z_candidate_len",
    "z_buffer_len",
    "z_used_hold",
    "z_last_control",
    "step",
    "elapsed",
    "abnormal_flag",
    "horizon",
    "gate_delta",
    "gate_mu",
    "min_buffer_len",
    "px4_state",
    "all_fresh",
    "all_gate_pass",
    "target_lost",
    "valid_visual_seen",
    "controller_initialized",
    "target_lost_steps",
    "target_loss_hold_steps",
    "attack_sim_enabled",
    "external_fdi_request",
    "fdi_noise",
    "dos_period_steps",
    "dos_active_steps",
    "fdi_prob",
    "fdi_amp",
    "des_yaw",
    "dt",
    "x_u_max",
    "y_u_max",
    "z_u_max",
    "kp",
    "kv",
    "observer_l0",
    "observer_l1",
    "observer_l2",
    "gate_qe",
    "gate_qv",
    "gate_qd",
    "smart_qe",
    "smart_qv",
    "smart_qd",
    "smart_re",
    "smart_rv",
    "x_packet_compute_time",
    "y_packet_compute_time",
    "z_packet_compute_time",
    "packet_compute_time_total",
    "x_axis_bias",
    "y_axis_bias",
    "z_axis_bias",
    "x_target_mocap_comp",
    "y_target_mocap_comp",
    "z_target_mocap_comp",
    "x_relative_pos_mocap",
    "y_relative_pos_mocap",
    "z_relative_pos_mocap",
    "x_relative_vel_mocap",
    "y_relative_vel_mocap",
    "z_relative_vel_mocap",
    "attack_seed",
    "debug_schema_version",
    "dos_model",
    "dos_prob",
    "method_id",
    "ros_time",
    "x_vision_raw",
    "y_vision_raw",
    "z_vision_raw",
    "x_relative_pos_mocap_aligned",
    "y_relative_pos_mocap_aligned",
    "z_relative_pos_mocap_aligned",
    "x_nominal_u",
    "y_nominal_u",
    "z_nominal_u",
    "x_local_safety_u",
    "y_local_safety_u",
    "z_local_safety_u",
    "x_attack_delta_u",
    "y_attack_delta_u",
    "z_attack_delta_u",
    "attack_packet_slot",
    "attack_packet_sequence",
    "attack_packet_interval",
    "x_uav_mocap_log_comp",
    "y_uav_mocap_log_comp",
    "z_uav_mocap_log_comp",
};
} // namespace

class DataRecorder
{
public:
    DataRecorder()
    {
        // Initialize ROS node handle
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        std::string output_file;
        private_nh.param<std::string>("output_file", output_file, "data1.csv");

        // Subscribe to the /hat_x_topic
        subscriber_ = nh.subscribe("/hat_x_topic", 0, &DataRecorder::callback, this);

        // Create or overwrite the CSV file
        file_.open(output_file, std::ofstream::out | std::ofstream::trunc);
        if (!file_.is_open())
        {
            ROS_FATAL_STREAM("Failed to open telemetry CSV: " << output_file);
            ros::shutdown();
            return;
        }
        for (int i = 0; i < kDebugDataSize; ++i)
        {
            if (i > 0)
            {
                file_ << ",";
            }
            file_ << kHeaders[i];
        }
        file_ << "\n";
    }

    ~DataRecorder()
    {
        // Close the file on shutdown
        file_.close();
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() != kDebugDataSize)
        {
            ROS_ERROR_THROTTLE(1.0, "Debug schema mismatch: expected %d values, received %zu.",
                               kDebugDataSize, msg->data.size());
            return;
        }

        if (file_.is_open())
        {
            for (int i = 0; i < kDebugDataSize; ++i)
            {
                if (i > 0)
                {
                    file_ << ",";
                }
                file_ << msg->data[i];
            }
            file_ << "\n";
            file_.flush();
        }
    }

private:
    ros::Subscriber subscriber_;
    std::ofstream file_;
};

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "data_recorder");
    DataRecorder recorder;
    // Enter ROS event loop
    ros::spin();
    return 0;
}
