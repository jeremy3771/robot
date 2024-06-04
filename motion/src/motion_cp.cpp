#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


#define PI 3.141592653589793

using namespace std::chrono_literals;
using std::placeholders::_1;

class GoToPose: public rclcpp::Node {
public:
  GoToPose(): Node("go_to_pose") {
    declare_parameter("kr", 0.6);
    declare_parameter("ka", 0.8);
    declare_parameter("kb", 0.2);
    declare_parameter("dmax", 0.45);
    declare_parameter("W1", 1);
    declare_parameter("W2", 2);
    get_parameter("kr", kr_);
    get_parameter("ka", ka_);
    get_parameter("kb", kb_);
    get_parameter("dmax", dmax_);
    get_parameter("W1", W1_);
    get_parameter("W2", W2_);

    RCLCPP_INFO(get_logger(), "W1 = %d, W2 = %d", W1_, W2_);

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&GoToPose::timer_cb, this));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&GoToPose::odom_cb, this, _1));
    goal_pose_[0] = 3.0;
    goal_pose_[1] = 3.0;
    goal_pose_[2] = 0;
    goal_pose_[3] = 0;
    goal_pose_[4] = 0;
    goal_pose_[5] = PI/2;
  }
private:
  void timer_cb() {
    RCLCPP_INFO(get_logger()
        , "x = %lf, y = %lf, yaw = %lf pi\n"
        , cur_pose_[0], cur_pose_[1], cur_pose_[5]/PI);
    double dx = goal_pose_[0] - cur_pose_[0];
    double dy = goal_pose_[1] - cur_pose_[1];
    double dist = sqrt(dx*dx + dy*dy);
    double theta = atan(dy/dx);
    if(dx < 0) theta += PI;
    if(theta > PI) theta -= 2*PI;
    if(theta < -PI) theta += 2*PI;

    double alpha = theta - cur_pose_[5];
    double beta = goal_pose_[5] - theta;
    if(alpha > PI) alpha -= 2*PI;
    if(alpha < -PI) alpha += 2*PI;
    if(beta > PI) beta -= 2*PI;
    if(beta < -PI) beta += 2*PI;
    RCLCPP_INFO(get_logger()
        , "dist = %lf, theta = %lf, alpha = %lf, beta = %lf pi\n"
        , dist, theta/PI, alpha/PI, beta/PI);

    double vel = kr_*dist;
    double omega = ka_*alpha - kb_*beta;
    double omax = vel*tan(dmax_)/0.3;
    if(omega > omax) omega = omax;
    if(omega < -omax) omega = -omax;

    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = vel;
    cmd.angular.z = omega;
    pub_->publish(cmd);
  }
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    cur_pose_[0] = msg->pose.pose.position.x;
    cur_pose_[1] = msg->pose.pose.position.y;
    cur_pose_[2] = msg->pose.pose.position.z;

    tf2::Quaternion tf2_quat;
    tf2::fromMsg(msg->pose.pose.orientation, tf2_quat);
    tf2::Matrix3x3 m(tf2_quat);
    m.getRPY(cur_pose_[3], cur_pose_[4], cur_pose_[5]);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double cur_pose_[6];
  double goal_pose_[6];
  int W1_, W2_;
  double kr_, ka_, kb_, dmax_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPose>());
    rclcpp::shutdown();
    return 0;
}