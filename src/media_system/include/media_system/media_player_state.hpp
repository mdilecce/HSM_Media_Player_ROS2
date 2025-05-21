/**
 * @file media_player_state.hpp
 * @brief Defines the Play, Pause, and Stop states for the HSM Media Player.
 * @details This file contains the implementation of the Play, Pause, and Stop states, which manage playback transitions in the media player.
 * @author Michele Di Lecce
 * @date 2025-05
 * @email michele.dilecce@hotmail.com
 */

#ifndef MEDIA_SYSTEM_MEDIA_PLAYER_STATE_HPP
#define MEDIA_SYSTEM_MEDIA_PLAYER_STATE_HPP

#include <thread>
#include "media_system/media_power_state.hpp"

namespace media_system{

    /**
     * @class Play
     * @brief Represents the playing state of the media player.
     */
    class Play : public PowerOn{
        public:       
            /**
             * @brief Constructor for Play.
             * @param options Node options for ROS 2.
             * @param state_name Name of the state.
             * @param state_msg Message associated with the state.
             */
            Play(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), 
                const std::string &state_name = "Play", 
                const std::string &state_msg = "Music Playing");

            /**
             * @brief Destructor for Play.
             */
            ~Play();

            /**
             * @brief Handles events in the Play state.
             * @param event The event to handle.
             * @return A shared pointer to the next MediaState.
             */
            std::shared_ptr<MediaState> handle_event(Events);

        private:
            SpeakerMsg  speaker_msg_;
            rclcpp::TimerBase::SharedPtr timer_;
            std::thread spin_thread_;
    };
    
    /**
     * @class Pause
     * @brief Represents the paused state of the media player.
     */
    class Pause : public PowerOn{
        public:       
            /**
             * @brief Constructor for Pause.
             * @param options Node options for ROS 2.
             * @param state_name Name of the state.
             * @param state_msg Message associated with the state.
             */
            Pause(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), 
                const std::string &state_name = "Media_Player", 
                const std::string &state_msg = "Music Paused");

            /**
             * @brief Handles events in the Pause state.
             * @param event The event to handle.
             * @return A shared pointer to the next MediaState.
             */
            std::shared_ptr<MediaState> handle_event(Events);
    };

    /**
     * @class Stop
     * @brief Represents the stopped state of the media player.
     */
    class Stop : public PowerOn{
        public:       
            /**
             * @brief Constructor for Stop.
             * @param options Node options for ROS 2.
             * @param state_name Name of the state.
             * @param state_msg Message associated with the state.
             */
            Stop(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), 
                const std::string &state_name = "Media_Player", 
                const std::string &state_msg = "Music Stopped");

            /**
             * @brief Handles events in the Stop state.
             * @param event The event to handle.
             * @return A shared pointer to the next MediaState.
             */
            std::shared_ptr<MediaState> handle_event(Events);
    };
}

#endif