#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
using namespace std::chrono_literals;

class GroundTruthFilterNode : public rclcpp::Node{
    public:
        GroundTruthFilterNode(): Node("ground_truth_filter_node"){
            subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/world/world_model/pose/info", //Tópico de ign que contiene la pose con respecto al marco global de todos los links del modelo
                10,
                std::bind(&GroundTruthFilterNode::sub_callback, this, std::placeholders::_1)
            );

            publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/r1/ground_truth",10);
        }
    
    private:
        void sub_callback(const tf2_msgs::msg::TFMessage & msg) const{
            for (auto transform : msg.transforms){
                if(transform.child_frame_id == "r1"){
                    auto pose = geometry_msgs::msg::PoseStamped();
                    pose.header.stamp = transform.header.stamp;
                    pose.header.frame_id = transform.header.frame_id;
                
                    //Posición
                    pose.pose.position.x = transform.transform.translation.x;
                    pose.pose.position.y = transform.transform.translation.y;
                    pose.pose.position.z = transform.transform.translation.z;
                    
                    //Orientación
                    pose.pose.orientation.x = transform.transform.rotation.x;
                    pose.pose.orientation.y = transform.transform.rotation.y;
                    pose.pose.orientation.z = transform.transform.rotation.z;
                    pose.pose.orientation.w = transform.transform.rotation.w;
                    publisher->publish(pose);
                }
            }
        }

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription;
};

int main(int argc, char*argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GroundTruthFilterNode>());
    rclcpp::shutdown();
    return 0;
}