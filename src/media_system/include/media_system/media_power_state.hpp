/**
 * @file media_power_state.hpp
 * @brief Defines the PowerOn and PowerOff states for the HSM Media Player.
 * @details This file contains the implementation of the PowerOn and PowerOff states, which manage power transitions in the media player.
 * @author Michele Di Lecce
 * @date 2025-05
 * @email michele.dilecce@hotmail.com
 */

#ifndef MEDIA_SYSTEM_MEDIA_POWER_STATE_HPP
#define MEDIA_SYSTEM_MEDIA_POWER_STATE_HPP

#include "media_system/media_state.hpp"

namespace media_system{
    
    /**
     * @class PowerOn
     * @brief Represents the powered-on state of the media player.
     */
    class PowerOn : public MediaState{
        public:       
            /**
             * @brief Constructor for PowerOn.
             * @param options Node options for ROS 2.
             * @param state_name Name of the state.
             * @param state_msg Message associated with the state.
             */
            PowerOn(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), 
                const std::string &state_name = "Power_on", 
                const std::string &state_msg = "Power On");

            /**
             * @brief Handles events in the PowerOn state.
             * @param event The event to handle.
             * @return A shared pointer to the next MediaState.
             */
            std::shared_ptr<MediaState> handle_event(Events);
    };

    /**
     * @class PowerOff
     * @brief Represents the powered-off state of the media player.
     */
    class PowerOff : public MediaState{
        public:       
            /**
             * @brief Constructor for PowerOff.
             * @param options Node options for ROS 2.
             * @param state_name Name of the state.
             * @param state_msg Message associated with the state.
             */
            PowerOff(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), 
                const std::string &state_name = "Power_off", 
                const std::string &state_msg = "Power Off");

            /**
             * @brief Handles events in the PowerOff state.
             * @param event The event to handle.
             * @return A shared pointer to the next MediaState.
             */
            std::shared_ptr<MediaState> handle_event(Events);
    };
}

#endif