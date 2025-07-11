#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <random>

#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"

#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include <algorithm> 
#include "turtlesim/srv/kill.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class turtle_spawn: public rclcpp::Node
{
public:
    turtle_spawn(): Node("turtle_spawner")
    {
        this->declare_parameter("turtle_name_prefix","My");
        this->declare_parameter("spawn_frequency", 2.0);
        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();
        spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();
        
        spawn_client_= this->create_client<turtlesim::srv::Spawn>("spawn");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / spawn_frequency_)),
            std::bind(&turtle_spawn::call_spawn, this)
            );
        //timer_ = this->create_wall_timer(2s, std::bind(&turtle_spawn::call_spawn, this));
        
        turtle_alive_publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10
        );
        turtle_timer_ = this->create_wall_timer(2s, std::bind(&turtle_spawn::callback_turtle_alive,this));
        
        turtle_catch_server_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "catch_turtle", std::bind(&turtle_spawn::callback_turtle_catch, this , _1)
        );
        turtle_kill_ = this->create_client<turtlesim::srv::Kill>("kill");
    }

private:
    void call_spawn()
    {
        while(!spawn_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for Server!!");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = getRandomDouble(0.0,11.0);
        request->y = getRandomDouble(0.0,11.0);
        request->theta = 0.2;

        // 이름 지정
        std::stringstream ss;
        ss << turtle_name_prefix_ << "_" << alive_turtles_.size();
        request->name = ss.str();

        last_spawn_x_ = request->x;
        last_spawn_y_ = request->y;
        
        spawn_client_->async_send_request(request, 
            std::bind(&turtle_spawn::callback_spawn,this,_1));
    }
    void callback_spawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();
        
        auto turtle_state = my_robot_interfaces::msg::Turtle();
        turtle_state.x = last_spawn_x_;
        turtle_state.y = last_spawn_y_;
        turtle_state.name = response->name;

        alive_turtles_.push_back(turtle_state);
        
        RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", response->name.c_str());
    }

    float getRandomDouble(float min, float max)
    {
        static std::random_device rd;   // 하드웨어 시드
        static std::mt19937 gen(rd());  // Mersenne Twister 엔진
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }

    void callback_turtle_alive()
    {
        auto msg = my_robot_interfaces::msg::TurtleArray();
        msg.turtles = alive_turtles_;
        turtle_alive_publisher_->publish(msg);
    }


    void callback_turtle_catch(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request) 
    {
        auto msg = std::string();
        msg = request->name;
        turtle_kill(msg);
        
    }
    void turtle_kill(const std::string& name)
    {
        while(!turtle_kill_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for Turtke Kill Server!!");
        }
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        
        //turtle_kill_->async_send_request(request, 
        //    std::bind(&turtle_spawn::callback_turtle_kill, this, _1 , name));
        turtle_kill_->async_send_request(
            request,
            [this, name](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
            {
                this->callback_turtle_kill(future, name);
            }
        );

    }
    void turtle_kill_update(const std::string& name)
    {
        auto it = std::remove_if(alive_turtles_.begin(), alive_turtles_.end(),
            [&name](const my_robot_interfaces::msg::Turtle& t){
                return t.name == name;
            });

        if (it != alive_turtles_.end())
        {
            alive_turtles_.erase(it, alive_turtles_.end());
            RCLCPP_INFO(this->get_logger(), "Removed turtle: %s", name.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Turtle not found: %s", name.c_str());
        }
    }
    void callback_turtle_kill(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future, std::string name)
    {
        auto response = future.get();
        turtle_kill_update(name);
        RCLCPP_INFO(this->get_logger(), "%s: is ERASED!!!", name.c_str());
    }
    
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr turtle_alive_publisher_;
    rclcpp::TimerBase::SharedPtr turtle_timer_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;
    float last_spawn_x_;
    float last_spawn_y_;

    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr turtle_catch_server_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr turtle_kill_;

    std::string turtle_name_prefix_;
    double spawn_frequency_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<turtle_spawn>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}