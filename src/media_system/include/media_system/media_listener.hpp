/**
 * @file media_listener.hpp
 * @brief Defines the MediaListener class for the HSM Media Player.
 * @details This file contains the implementation of the MediaListener class, which subscribes to speaker messages and logs them.
 * @program HSM Media Player
 * @date 2025-05
 * @authored_by Michele Di Lecce
 * @contact michele.dilecce@hotmail.com
 */

#ifndef MEDIA_SYSTEM_MEDIA_LISTENER_HPP
#define MEDIA_SYSTEM_MEDIA_LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "media_system/msg/speaker.hpp"

namespace media_system{

    /**
     * @class MediaListener
     * @brief Subscribes to speaker messages and logs them.
     */
    class MediaListener : public rclcpp::Node{
        public:
            /**
             * @brief Constructor for MediaListener.
             * @param options Node options for ROS 2.
             */
            explicit MediaListener(const rclcpp::NodeOptions & options);

        private:
            using SpeakerMsg = msg::Speaker;
            const std::string topic_name = "speaker";
            rclcpp::Subscription<SpeakerMsg>::SharedPtr sub_;
    };
}

#endif