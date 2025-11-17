#include "rclcpp/rclcpp.hpp"

class My_env_moveit_control : public rclcpp::Node {
public:
    My_env_moveit_control() : Node("my_env_moveit_control") {
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->timer_callback(); });
    }

private:
    void timer_callback() {
        RCLCPP_INFO(get_logger(), "Hello from my_env_moveit_control!");
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<My_env_moveit_control>());
    rclcpp::shutdown();
    return 0;
}
