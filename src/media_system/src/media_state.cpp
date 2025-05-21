#include "media_system/media_state.hpp"


namespace media_system{
    
    MediaState::MediaState(const rclcpp::NodeOptions &options,
         const std::string &state_name, 
         const std::string &state_msg)
         : Node(state_name, options){
            
            this->pub_ = 
                this -> create_publisher<SpeakerMsg>(
                    this->topic_name,
                    rclcpp::QoS(rclcpp::KeepLast(10)));

            RCLCPP_INFO(this->get_logger(), "Starting Node: '%s'", state_msg.c_str());
         
            SpeakerMsg msg;
            msg.speaker = std::move(state_msg);
            this->pub_->publish(msg);
        
        }
}