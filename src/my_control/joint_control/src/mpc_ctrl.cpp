#include "rclcpp/rclcpp.hpp"

class Mpc_ctrl : public rclcpp::Node {
public:
    Mpc_ctrl() : Node("mpc_ctrl") {
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->timer_callback(); });
    }

private:
    void timer_callback() {
        RCLCPP_INFO(get_logger(), "Hello from mpc_ctrl!");
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mpc_ctrl>());
    rclcpp::shutdown();
    return 0;
}
