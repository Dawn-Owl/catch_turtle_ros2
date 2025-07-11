#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "my_robot_interfaces/srv/catch_turtle.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class turtle_control: public rclcpp::Node
{
public:
    turtle_control():Node("turtle_controller")
    {
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&turtle_control::callback_turtle_control, this,_1)
        );
        alive_turtle_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&turtle_control::callback_alive_turtles, this,_1)
        );
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 10
        );
        timer_ = this->create_wall_timer(100ms,std::bind(&turtle_control::state_checker,this));
        catch_turtle_client_ = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        
        turtle_move_state = true;
    }

private:
    void state_checker()
    {
        /*if(pose_received_ == true && target_received_ == true)
        {
            turtle_move_state = false;
            call_cmd_vel();
        }*/
         if(pose_received_ && target_received_)
        {
            call_cmd_vel();  // 계속 호출!
        }
    }

    void call_cmd_vel()
    {
        
        float dx = current_target_.x - current_pose_.x;
        float dy = current_target_.y - current_pose_.y;
        float distance = std::sqrt(dx * dx + dy * dy);
        float angle_to_target = std::atan2(dy, dx);
        float angle_diff = angle_to_target - current_pose_.theta;

        // [-pi, pi] 범위로 보정
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        auto cmd_vel = geometry_msgs::msg::Twist();

        // ✅ 핵심 수정: 항상 직진 + 회전 함께
        cmd_vel.linear.x = 2.0 * distance;
        cmd_vel.angular.z = 6.0 * angle_diff;

        // 📤 publish
        cmd_vel_publisher_->publish(cmd_vel);

        // ✅ 목표 도달 판단
        if (distance < 0.5) {
            // 도달하면 멈춤
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel);

            pose_received_ = false;
            target_received_ = false;
            turtle_move_state = false;
            RCLCPP_INFO(this->get_logger(), "Reached target: %s", current_target_.name.c_str());

            auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
            request->name = current_target_.name;

            catch_turtle_client_->async_send_request(
                request,
                std::bind(&turtle_control::callback_catch_turtle, this, _1)
            );
            
        }
    }

    void callback_turtle_control(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_.x = msg->x;
        current_pose_.y = msg->y;
        current_pose_.theta = msg->theta;

        pose_received_ = true;
        
    }
    void callback_alive_turtles(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
       if(turtle_move_state == true)
       {
            if (!msg->turtles.empty())
            {
                current_target_ = msg->turtles.at(0);
                target_received_ = true;
            }
       }
    }

    void callback_catch_turtle(rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Catch turtle result: %s", response->success ? "true" : "false");

         turtle_move_state = true;
    }
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtle_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    turtlesim::msg::Pose current_pose_;
    bool pose_received_ = false;
    my_robot_interfaces::msg::Turtle current_target_;
    bool target_received_ = false;
    bool turtle_move_state = false;

    rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<turtle_control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}