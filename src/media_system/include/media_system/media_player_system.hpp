/**
 * @file media_player_system.hpp
 * @brief Defines the MediaPlayerSystem class for the HSM Media Player.
 * @details This file contains the implementation of the MediaPlayerSystem class, which manages the state transitions and actions of the media player.
 * @program HSM Media Player
 * @date 2025-05
 * @authored_by Michele Di Lecce
 * @contact michele.dilecce@hotmail.com
 */

#ifndef MEDIA_SYSTEM_MEDIA_PLAYER_SYSTEM_HPP
#define MEDIA_SYSTEM_MEDIA_PLAYER_SYSTEM_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "media_system/media_state.hpp"
#include "media_system/media_events.hpp"
#include "media_system/action/media.hpp"

namespace media_system{

    /**
     * @class MediaPlayerSystem
     * @brief Manages state transitions and actions for the media player.
     */
    class MediaPlayerSystem final : public rclcpp::Node{
        
        public:
            /**
             * @brief Constructor for MediaPlayerSystem.
             * @param options Node options for ROS 2.
             */
            explicit MediaPlayerSystem (const rclcpp::NodeOptions &);

        private:
            using MediaAction = media_system::action::Media;
            using MediaGoalHandle = rclcpp_action::ServerGoalHandle<MediaAction>;
            
            rclcpp_action::Server<MediaAction>::SharedPtr action_server_;
            std::shared_ptr<MediaState> state_node_;

            /**
             * @brief Executes a goal received by the action server.
             * @param goal_handle The goal handle.
             */
            void execute(const std::shared_ptr<MediaGoalHandle> goal_handle);
    };

}

#endif