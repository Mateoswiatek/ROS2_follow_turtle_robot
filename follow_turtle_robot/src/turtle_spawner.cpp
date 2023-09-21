#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

// poruszanie się, ale to już na oddzielnych wątkach chyba.
// i w tedy jeśli dobije do ściany, odwraca się, i znowu idzie do przodu. podczas pawnowania, cyklicznie co jakiś czas byśmy musieli wysyłać randomowe liczby na każdy z dostępnych /turtlex/cmd_vel

class TurtleSpawnerNode: public rclcpp::Node {
public:
    TurtleSpawnerNode() : Node("turtle_spawner"){
        this->declare_parameter("max_turtles", 10);
        this->declare_parameter("spawn_freq", 1.0);

        max_turtles = this->get_parameter("max_turtles").as_int();

        spawner_timer_ = this->create_wall_timer(std::chrono::milliseconds(
            (int)(1000.0 / this->get_parameter("spawn_freq").as_double())), 
            std::bind(&TurtleSpawnerNode::spawn_turtleCallBack, this));
        alive_turtle_publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10);

        catch_turtle_server_ =this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "catch_turtle", std::bind(&TurtleSpawnerNode::catch_turtle_callback, this,
            std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Spawner has been started.");
    }

    void spawn_turtleCallBack(){
        spawner_threads_.push_back(std::make_shared<std::thread>(std::bind(
            &TurtleSpawnerNode::spawn_turtle_thread, this)));
    }

    void spawn_turtle_thread(){
        if(turtles_.size() < max_turtles){
            auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
            while(!client->wait_for_service(std::chrono::seconds(5))){
                RCLCPP_WARN(this->get_logger(), "W8 for board, write 'ros2 run turtlesim turtlesim_node'");
            }
            std::srand(std::time(0));
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            float x = static_cast<float>(rand()) / RAND_MAX * 11.0f;
            float y = static_cast<float>(rand()) / RAND_MAX * 11.0f;
            float thetha = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
            
            request->x = x;
            request->y = y;
            request->theta = thetha;
            
            auto future = client->async_send_request(request);
            try{
                auto response = future.get();
                auto new_turtle = my_robot_interfaces::msg::Turtle();
                new_turtle.name = response->name;
                new_turtle.x = x;
                new_turtle.y = y;
                new_turtle.theta = thetha;

                turtles_.push_back(new_turtle);
                //publish_alive_turtles();
                RCLCPP_INFO(this->get_logger(), "Spawn Success!.");
            } catch(const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Spawn ERROR.");
            }
        } else { 
            RCLCPP_WARN(this->get_logger(), "there is a maximum number of turtles.");
        }
        publish_alive_turtles();
    }

    void publish_alive_turtles(){ // when spawn or kill.
        auto msg = my_robot_interfaces::msg::TurtleArray();
        msg.turtles = turtles_;
        alive_turtle_publisher_->publish(msg);
    }

    void catch_turtle_callback(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
    const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response){
        kill_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(
            &TurtleSpawnerNode::kill_turtle_thread, this, request->name)));
        response->success = true;
    }

    void kill_turtle_thread(std::string name){
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while(!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "W8 for kill server...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;

        auto future = client->async_send_request(request);
        
        try{
            future.get();
            for(int i = 0; i < turtles_.size(); i++){
                if(turtles_.at(i).name == name){
                    turtles_.erase(turtles_.begin() + i);
                    publish_alive_turtles();
                    break;
                }
            }

        } catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Service Kill Error, Turtle: %s", name.c_str());
        }
    }

private:
    int max_turtles;
    std::vector<my_robot_interfaces::msg::Turtle> turtles_;
    std::vector<std::shared_ptr<std::thread>> spawner_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> move_threads_;
    rclcpp::TimerBase::SharedPtr spawner_timer_;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtle_publisher_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_server_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
