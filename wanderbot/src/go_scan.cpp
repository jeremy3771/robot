#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class GoScan : public rclcpp::Node {
public:
    GoScan() : Node("go_scan") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&GoScan::timer_cb, this));
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 1, std::bind(&GoScan::scan_cb, this, _1));
    }

private:
    void timer_cb() {
        auto cmd = geometry_msgs::msg::Twist();
        if(range_ahead_ > 0 && range_ahead_ < 0.8) {
            cmd.linear.x = 0;
            cmd.angular.z = 0.2;
        }
        else if(range_ahead_ < 0 && range_ahead_ > -0.8) {
            cmd.linear.x = 0;
            cmd.angular.z = -0.2;
        }
        else {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0;
        }
        pub_->publish(cmd);
    }
    void scan_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
        range_ahead_ = msg->ranges[0];
        for(int i = 0; i < 16; i++) {
            if (range_ahead_ > msg->ranges[i])
                range_ahead_ = msg->ranges[i];
        }
        for(int i = 345; i < 360; i++) {
            if (range_ahead_ > msg->ranges[i])
                range_ahead_ = (msg->ranges[i] * -1);
        }
        printf("range ahead: %f\n", range_ahead_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    float range_ahead_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoScan>());
    rclcpp::shutdown();
    return 0;
}