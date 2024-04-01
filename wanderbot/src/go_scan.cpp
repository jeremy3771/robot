#include <chrono>
#include <thread>
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
            "scan", rclcpp::QoS(1).best_effort(), std::bind(&GoScan::scan_cb, this, _1));
    }

private:
    void timer_cb() {
        auto cmd = geometry_msgs::msg::Twist();
        if(range_ahead_ > 0 && range_ahead_ < 0.5 && (minAng < 20 || minAng >= 220) && avoid == 0 && isTurn == 0) {
            cmd.linear.x = 0;
            cmd.angular.z = -0.4;
            isTurn = 1;
            avoid = 1;
            turn_time = 3.9;
        }
        else if (isTurn == 1) {
            if(d1 - d2 > 0.02) {
                cmd.linear.x = 0;
                cmd.angular.z = -0.2;
            }
            else if(d1 - d2 < -0.02) {
                cmd.linear.x = 0;
                cmd.angular.z = 0.2;
            }
            else {
	        cmd.linear.x = 0;
                cmd.angular.z = 0;
                isTurn = 0;
	    }
	    turn_time = 0.01;
        }
        else if(avoid == 1 && d1 > 1) {
            isTurn = 1;
            turn_time = 3.9;
            avoid = 0;
            
            cmd.linear.x = 0;
            cmd.angular.z = 0.4;
            
        }
        else {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0;
        }
        
        pub_->publish(cmd);
        if (isTurn == 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(turn_time * 1000)));
            if (avoid == 0)
                isTurn = 0;
        }
    }
    void scan_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
        range_ahead_ = msg->ranges[0];
        for(int i = 1; i < 20; i++) {
            if (range_ahead_ > msg->ranges[i]) {
                range_ahead_ = msg->ranges[i];
		minAng = i;
	    }
        }

	if(range_ahead_ > 0.5) {
            for(int i = 220; i < 240; i++) {
                if (range_ahead_ > msg->ranges[i]) {
                    range_ahead_ = msg->ranges[i];
		    minAng = i;
		}
            }
	}
	
	if(avoid == 1) {
	    d1 = msg->ranges[73];
	    d2 = msg->ranges[40];
	}

        printf("range ahead: %f\n", range_ahead_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    float range_ahead_;
    int minAng = 0;
    int isTurn = 0;
    float turn_time = 0;
    int avoid = 0;
    int d1, d2;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoScan>());
    rclcpp::shutdown();
    return 0;
}
