#include "media_system/media_player_system.hpp"
#include "media_system/media_power_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <chrono>

namespace media_system{

    using namespace std::chrono_literals;

    MediaPlayerSystem::MediaPlayerSystem(
        const rclcpp::NodeOptions &options)
        : Node("media_player_system", options){

        // Initialize the state node
        this->state_node_ = std::make_shared<PowerOff>();

        auto handle_goal = [this](
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MediaAction::Goal> goal){
            Events event = static_cast<Events>(goal->action_type);
            RCLCPP_INFO(this->get_logger(), 
                "Received Command %s", eventStringMap[event].c_str());
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          };
      
          auto handle_cancel = [this](
            const std::shared_ptr<MediaGoalHandle> goal_handle){
            RCLCPP_INFO(this->get_logger(), 
                "Received request to cancel Command");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
          };
      
          auto handle_accepted = [this](
            const std::shared_ptr<MediaGoalHandle> goal_handle){
                auto execute_in_thread = 
                [this, goal_handle](){
                    return this->execute(goal_handle);
                };
            std::thread{execute_in_thread}.detach();
          };

        RCLCPP_INFO(this->get_logger(), "MediaPlayerSystem action server started.");
   
        this->action_server_ = rclcpp_action::create_server<MediaAction>(
            this,
            "media_action",
            handle_goal,
            handle_cancel,
            handle_accepted);   
    }

    void MediaPlayerSystem::execute(
        const std::shared_ptr<MediaGoalHandle> goal_handle){

        RCLCPP_INFO(this->get_logger(), "Executing goal...");

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MediaAction::Result>();

        // Retrieve the event from the goal
        Events event = static_cast<Events>(goal->action_type);

        // Transition to the next state
        auto newNode = this->state_node_->handle_event(event);
            
        if(newNode != nullptr)
            this->state_node_ = std::move(newNode);

        // Simulate some processing time
        std::this_thread::sleep_for(1s);

        // Mark the goal as succeeded
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(media_system::MediaPlayerSystem)