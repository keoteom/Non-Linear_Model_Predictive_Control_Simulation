/**
 * @file odom_publisher.cpp
 * @brief PX4 → nav_msgs/Odometry converter.
 *
 * Subscribes to actual PX4 drone state:
 *   /fmu/out/vehicle_local_position  (px4_msgs/VehicleLocalPosition)
 *   /fmu/out/vehicle_attitude        (px4_msgs/VehicleAttitude)
 *
 * Publishes:
 *   /odom  (nav_msgs/Odometry)
 *
 * PX4 uses NED frame. This node converts to ENU (ROS convention)
 * if use_enu parameter is true (default), or keeps NED if false.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <chrono>
#include <mutex>

class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher() : Node("odom_publisher") {
        // Parameters
        this->declare_parameter<bool>("use_enu", true);
        this->declare_parameter<double>("rate", 20.0);

        use_enu_ = this->get_parameter("use_enu").as_bool();
        double rate = this->get_parameter("rate").as_double();

        // QoS for PX4 topics
        rmw_qos_profile_t qos_raw = rmw_qos_profile_sensor_data;
        auto qos_px4 = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_raw.history, 5), qos_raw);

        // Subscribers – PX4 actual state
        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos_px4,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                local_pos_ = *msg;
                pos_received_ = true;
            });

        sub_attitude_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos_px4,
            [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                attitude_ = *msg;
                att_received_ = true;
            });

        // Publisher
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_nmpc", 10);

        // Timer
        int period_ms = static_cast<int>(1000.0 / rate);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&OdomPublisher::publish_odom, this));

        RCLCPP_INFO(this->get_logger(),
                    "PX4 -> /odom converter started | rate=%.0f Hz | frame=%s",
                    rate, use_enu_ ? "ENU" : "NED");
    }

private:
    void publish_odom() {
        std::lock_guard<std::mutex> lock(mtx_);

        if (!pos_received_ || !att_received_) return;

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";

        if (use_enu_) {
            // PX4 NED -> ROS ENU conversion
            // Position: x_enu = x_ned, y_enu = -y_ned, z_enu = -z_ned
            msg.pose.pose.position.x =  local_pos_.x;
            msg.pose.pose.position.y =  local_pos_.y;
            msg.pose.pose.position.z = -local_pos_.z;

            // Velocity
            msg.twist.twist.linear.x =  local_pos_.vx;
            msg.twist.twist.linear.y =  local_pos_.vy;
            msg.twist.twist.linear.z = -local_pos_.vz;

            // Quaternion NED -> ENU: q_enu = [q.w, q.x, -q.y, -q.z]
            msg.pose.pose.orientation.w =  attitude_.q[0];
            msg.pose.pose.orientation.x =  attitude_.q[1];
            msg.pose.pose.orientation.y =  attitude_.q[2];
            msg.pose.pose.orientation.z =  attitude_.q[3];
        } else {
            // Keep NED as-is
            msg.pose.pose.position.x = local_pos_.x;
            msg.pose.pose.position.y = local_pos_.y;
            msg.pose.pose.position.z = local_pos_.z;

            msg.twist.twist.linear.x = local_pos_.vx;
            msg.twist.twist.linear.y = local_pos_.vy;
            msg.twist.twist.linear.z = local_pos_.vz;

            msg.pose.pose.orientation.w = attitude_.q[0];
            msg.pose.pose.orientation.x = attitude_.q[1];
            msg.pose.pose.orientation.y = attitude_.q[2];
            msg.pose.pose.orientation.z = attitude_.q[3];
        }

        pub_odom_->publish(msg);
    }

    // Parameters
    bool use_enu_{true};

    // State storage
    std::mutex mtx_;
    px4_msgs::msg::VehicleLocalPosition local_pos_;
    px4_msgs::msg::VehicleAttitude      attitude_;
    bool pos_received_{false};
    bool att_received_{false};

    // ROS interfaces
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr      sub_attitude_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                pub_odom_;
    rclcpp::TimerBase::SharedPtr                                         timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}