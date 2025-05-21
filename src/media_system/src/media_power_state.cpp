#include "media_system/media_power_state.hpp"
#include "media_system/media_player_state.hpp"

namespace media_system{
    
    PowerOn::PowerOn(const rclcpp::NodeOptions &options, 
        const std::string &state_name, 
        const std::string &state_msg)
        : MediaState(options,state_name,state_msg){}

    std::shared_ptr<MediaState> PowerOn::handle_event(Events event){
        if (event == Events::POWER_TOGGLE)
            return std::make_shared<PowerOff>();
        else
            return nullptr;
    }

    PowerOff::PowerOff(const rclcpp::NodeOptions &options, 
        const std::string &state_name, 
        const std::string &state_msg)
        : MediaState(options, state_name, state_msg) {
        // Constructor implementation (if needed)
    }
     
    std::shared_ptr<MediaState> PowerOff::handle_event(Events event){
        if (event == Events::POWER_TOGGLE)
            return std::make_shared<Stop>();
        else
            return nullptr;
    }        
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(media_system::PowerOn)
RCLCPP_COMPONENTS_REGISTER_NODE(media_system::PowerOff)

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(media_system::PowerOn, media_system::MediaState)
PLUGINLIB_EXPORT_CLASS(media_system::PowerOff, media_system::MediaState)