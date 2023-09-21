#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include "turtlesim/msg/pose.hpp"

class RobotNode : public rclcpp::Node {
public:
    RobotNode() : Node("robot"){
        this->declare_parameter("closed_turtle", true);
        this->declare_parameter("linear_speed", 2.0);
        this->declare_parameter("angular_speed", 15.0);
        this->declare_parameter("catch_distance", 0.5);
        
        closed_turtle = this->get_parameter("closed_turtle").as_bool();
        linear_speed = this->get_parameter("linear_speed").as_double();
        angular_speed = this->get_parameter("linear_speed").as_double();
        cath_distance = this->get_parameter("catch_distance").as_double();

        turtles_subscriber = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10,
            std::bind(&RobotNode::turtles_to_catch, this, std::placeholders::_1));
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&RobotNode::pose_callback, this, std::placeholders::_1));
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(
            100),
            std::bind(&RobotNode::main_loop, this));
        set_current_turtle_timer_ = this->create_wall_timer(std::chrono::microseconds(
            500), 
            std::bind(&RobotNode::set_current_turtle, this));
        
        RCLCPP_INFO(this->get_logger(), "Robot has been started.");
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose){
        current_pose_ = *pose.get(); // pose is a pointer.
    } 

    void main_loop(){
        auto msg = geometry_msgs::msg::Twist();
        if(current_turtle_.name == ""){ return;} // after caught, vel = 0.
        
        //float distance = calculate_distance_from_curr_pose(current_turtle_); no because we need d_x, d_y to atan2
        float d_x = current_turtle_.x - current_pose_.x;
        float d_y = current_turtle_.y - current_pose_.y;
        float d_distance = std::sqrt(d_x * d_x + d_y * d_y);
        if(d_distance > cath_distance){
            msg.linear.x = linear_speed * d_distance;
            float basic_angle = std::atan2(d_y, d_x);
            float d_angle = basic_angle - current_pose_.theta;
            if(d_angle < -M_PI){
                d_angle += 2 * M_PI;
            } else if(d_angle > M_PI){
                d_angle -= 2 * M_PI;
            }
            msg.angular.z = angular_speed * d_angle;

        } else { // we caught the turtle
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            caught_turtles_threads_.push_back(std::make_shared<std::thread>(
                std::bind(&RobotNode::call_catch_turtle_service, this, current_turtle_.name)));
            current_turtle_.name = "";
        }
        vel_publisher_->publish(msg); // send move.
    }

    void turtles_to_catch(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg){
        // odczytywanie tablicy, wybieranie tego co ma najmniejsza odleglosc,
        // jesli nie najblizszy, to bierzemy ostatniego, czyli na pierszej pozycji.
        turtles_ = msg->turtles;
        set_current_turtle();
    }

    void set_current_turtle(){ // set turtle with min distance
        if(!turtles_.empty()){
            current_turtle_ = turtles_.at(0);
            if(closed_turtle){
                float min_distance = calculate_distance_from_curr_pose(current_turtle_);
                for(int i = 1; i < (int)turtles_.size(); i++){
                    float distance = calculate_distance_from_curr_pose(turtles_.at(i));
                    if( distance < min_distance){
                        current_turtle_ = turtles_.at(i);
                        min_distance = distance;
                    }
                }
            } // if no closed_turtle, then we will catch turtles at(0). logic.
        }
    }

    void call_catch_turtle_service(std::string name){
        auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        while(!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "W8 for spawner killer server...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle_Request>();
        request->name = name;

        auto future = client->async_send_request(request);

        try{
            auto response = future.get();
            if(response->success){
                RCLCPP_INFO(this->get_logger(), "You killed %s", name.c_str());
            }
        } catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Spawner killing Error");
        }
    }

    float calculate_distance_from_curr_pose(my_robot_interfaces::msg::Turtle turtle){
        float d_x = turtle.x - current_pose_.x;
        float d_y = turtle.y - current_pose_.y;
        return std::sqrt(d_x * d_x + d_y * d_y);
    }

private:
    bool closed_turtle;
    float linear_speed, angular_speed, cath_distance;
    std::vector<std::shared_ptr<std::thread>> caught_turtles_threads_;
    std::vector<my_robot_interfaces::msg::Turtle> turtles_;
    my_robot_interfaces::msg::Turtle current_turtle_;
    turtlesim::msg::Pose current_pose_;

    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr turtles_subscriber;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr vel_timer_;
    rclcpp::TimerBase::SharedPtr set_current_turtle_timer_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
