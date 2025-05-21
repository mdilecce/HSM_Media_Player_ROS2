#include "media_system/media_listener.hpp"

#include <iostream>
#include <memory>


namespace media_system{
    MediaListener::MediaListener(const rclcpp::NodeOptions & options)
    : Node("media_listener", options){
        auto callback =
            [this](SpeakerMsg::ConstSharedPtr speaker_msg) -> void{
                std::string msg = speaker_msg->speaker;
                RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.c_str());
                std::flush(std::cout);
            };  
        this-> sub_ =  this->create_subscription<SpeakerMsg>(this->topic_name, 10, callback);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(media_system::MediaListener)