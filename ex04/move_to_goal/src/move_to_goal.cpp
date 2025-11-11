#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

class MoveToGoal : public rclcpp::Node {
public:
    MoveToGoal(double goal_x, double goal_y, double goal_theta) : Node("move_to_goal") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MoveToGoal::pose_callback, this, std::placeholders::_1));

        goal_x_ = goal_x;
        goal_y_ = goal_y;
        goal_theta_ = goal_theta;

        k_linear_ = 1.5;
        k_angular_ = 6.0;
        distance_tolerance_ = 0.1;
        angle_tolerance_ = 0.01;

        pose_received_ = false;

        timer_ = this->create_wall_timer(100ms, std::bind(&MoveToGoal::control_loop, this));
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    void control_loop() {
        if (!pose_received_) {
            return;
        }

        double dx = goal_x_ - current_pose_.x;
        double dy = goal_y_ - current_pose_.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        double target_angle = std::atan2(dy, dx);
        double angle_error = target_angle - current_pose_.theta;
        double final_angle_error = goal_theta_ - current_pose_.theta;

        auto msg = geometry_msgs::msg::Twist();

        if (distance > distance_tolerance_) {
            if (std::abs(angle_error) > angle_tolerance_) {
                msg.angular.z = k_angular_ * angle_error;
            } else {
                msg.linear.x = k_linear_ * distance;
                msg.angular.z = 0.0;
            }
        } else {
            if (std::abs(final_angle_error) > angle_tolerance_) {
                msg.angular.z = k_angular_ * final_angle_error;
            } else {
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                timer_->cancel();
            }
        }

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose current_pose_;
    bool pose_received_;

    double goal_x_, goal_y_, goal_theta_;
    double k_linear_, k_angular_;
    double distance_tolerance_, angle_tolerance_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: move_to_goal x y theta");
        return 1;
    }

    double goal_x = std::stod(argv[1]);
    double goal_y = std::stod(argv[2]);
    double goal_theta = std::stod(argv[3]);

    auto node = std::make_shared<MoveToGoal>(goal_x, goal_y, goal_theta);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}