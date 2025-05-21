/**
 * @file media_state.hpp
 * @brief Base class for all media states in the HSM Media Player.
 * @details This file defines the MediaState class, which serves as the base class for all states in the media player.
 * @author Michele Di Lecce
 * @date 2025-05
 * @email michele.dilecce@hotmail.com
 */

#ifndef MEDIA_SYSTEM_MEDIA_STATE_HPP
#define MEDIA_SYSTEM_MEDIA_STATE_HPP

#pragma once

#include <string>
#include <memory> // Include for shared_ptr

#include "rclcpp/rclcpp.hpp"
#include "media_system/media_events.hpp"
#include "media_system/msg/speaker.hpp"

namespace media_system{

    /**
     * @class MediaState
     * @brief Abstract base class for media states.
     */
    class MediaState : public rclcpp::Node, public std::enable_shared_from_this<MediaState> {
    public:
        /**
         * @brief Constructor for MediaState.
         * @param options Node options for ROS 2.
         * @param state_name Name of the state.
         * @param state_msg Message associated with the state.
         */
        MediaState(const rclcpp::NodeOptions &options, 
            const std::string &state_name = "Base State", 
            const std::string &state_msg = "State Base Class");

        /**
         * @brief Pure virtual method to handle events.
         * @param event The event to handle.
         * @return A shared pointer to the next MediaState.
         */
        virtual std::shared_ptr<MediaState> handle_event(Events) = 0;

    protected:
        using SpeakerMsg = msg::Speaker;

        const std::string topic_name = "speaker";

        rclcpp::Publisher<SpeakerMsg>::SharedPtr pub_;
    };
}

#endif //MEDIA_SYSTEM_STATE_HPP