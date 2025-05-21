/**
 * @file media_control.hpp
 * @brief Defines the MediaControl and KeyboardReader classes for the HSM Media Player.
 * @details This file contains the implementation of the MediaControl class, which handles user input and sends actions to the media player, and the KeyboardReader class for reading keyboard input.
 * @program HSM Media Player
 * @date 2025-05
 * @authored_by Michele Di Lecce
 * @contact michele.dilecce@hotmail.com
 */

#ifndef MEDIA_SYSTEM_MEDIA_CONTROL_HPP
#define MEDIA_SYSTEM_MEDIA_CONTROL_HPP

#pragma once 

#include <termios.h>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "media_system/action/media.hpp"
#include "media_system/media_events.hpp"

namespace media_system{

    /**
     * @class KeyboardReader
     * @brief Handles raw keyboard input for the media player.
     */
    class KeyboardReader final{
        private:
            struct termios cooked_;
        public:
            /**
             * @brief Constructor for KeyboardReader.
             * @throws std::runtime_error if the console mode cannot be set.
             */
            KeyboardReader();

            /**
             * @brief Reads a single character from the keyboard.
             * @return The character read.
             * @throws std::runtime_error if reading fails.
             */
            char readOne();

            /**
             * @brief Destructor for KeyboardReader.
             */
            ~KeyboardReader();
    };

    /**
     * @class MediaControl
     * @brief Handles user input and sends actions to the media player.
     */
    class MediaControl final : public rclcpp::Node{

        public:
            /**
             * @brief Constructor for MediaControl.
             */
            explicit MediaControl();

            /**
             * @brief Destructor for MediaControl.
             */
            ~MediaControl();

            /**
             * @brief Main loop for handling keyboard input.
             * @return Exit code.
             */
            int keyLoop();

        private:
            using MediaAction = action::Media;

            std::map<char,Events> keyMap = {
                {POWER_KEY,Events::POWER_TOGGLE},
                {PLAY_BUTTON,Events::PLAY_PAUSE},
                {STOP_BUTTON,Events::STOP}
            };
            
            // Parameters Keybinding        
            std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

            // Action Client
            rclcpp_action::Client<MediaAction>::SharedPtr client_ptr_;
            rclcpp_action::ClientGoalHandle<MediaAction>::SharedPtr goal_handle_;

            // Message Console
            std::string console_msg_;
            std::string console_commands_;

            // Keyboard read
            std::shared_ptr<KeyboardReader> keyboard_ptr_;

            /**
             * @brief Sends a goal to the media player.
             * @param event The event to send.
             */
            void send_goal(Events);

            /**
             * @brief Handles parameter changes.
             * @param p The parameter that changed.
             */
            void parameter_change(const rclcpp::Parameter & p);
    };

    /**
     * @brief Global flag for running the media control loop.
     */
    bool running = true;

}

#endif
