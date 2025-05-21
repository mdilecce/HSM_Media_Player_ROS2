/**
 * @file media_events.hpp
 * @brief Defines events and key mappings for the HSM Media Player.
 * @details This file contains the enumeration of events and their corresponding key mappings for user input in the media player.
 * @program HSM Media Player
 * @date 2025-05
 * @authored_by Michele Di Lecce
 * @contact michele.dilecce@hotmail.com
 */

#ifndef MEDIA_SYSTEM_MEDIA_EVENTS_HPP
#define MEDIA_SYSTEM_MEDIA_EVENTS_HPP

#pragma once

#define POWER_KEY 's'
#define PLAY_BUTTON 'd'
#define STOP_BUTTON 'f'

#include <map>
#include <string>

namespace media_system{

    /**
     * @enum Events
     * @brief Enumeration of media player events.
     */
    enum class Events {
        POWER_TOGGLE=0, /**< Power toggle event. */
        PLAY_PAUSE=1,   /**< Play or pause event. */
        STOP=2          /**< Stop event. */
    };

    /**
     * @brief Maps events to their string representations.
     */
    static std::map<Events,std::string> eventStringMap = {
        {Events::POWER_TOGGLE,"POWER_TOGGLE"},
        {Events::PLAY_PAUSE,"PLAY_PAUSE"},
        {Events::STOP,"STOP"}
    };

}

#endif