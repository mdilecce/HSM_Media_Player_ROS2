#include <algorithm>
#include <random>
#include <chrono>
#include <thread>

#include "media_system/media_player_state.hpp"

namespace media_system{

    using namespace std::chrono_literals;

    Play::Play(const rclcpp::NodeOptions &options, 
        const std::string &state_name, 
        const std::string &state_msg)
        : PowerOn(options, state_name, state_msg) {

            this->speaker_msg_.speaker = "_|         ";

            auto shift_msg = [this]() -> void {                
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> dist(0, 1); 
                bool shift_right = dist(gen);
                if (shift_right) {
                    // Circular shift right
                    char last_char = this->speaker_msg_.speaker.back();
                    this->speaker_msg_.speaker.pop_back();
                    this->speaker_msg_.speaker.insert(this->speaker_msg_.speaker.begin(), last_char);
                } else {
                    // Circular shift left
                    char first_char = this->speaker_msg_.speaker.front();
                    this->speaker_msg_.speaker.erase(this->speaker_msg_.speaker.begin());
                    this->speaker_msg_.speaker.push_back(first_char);
                }
                this->pub_->publish(this->speaker_msg_);
            };

            // Start the timer to periodically publish the shift_msg
            this->timer_ = this->create_wall_timer(1s, shift_msg);

            // Start spinning the node in a thread
            this->spin_thread_ = std::thread([this]() {
                rclcpp::spin(this->get_node_base_interface());
            });
        }

    Play::~Play(){
        this->timer_.reset();
        if (this->spin_thread_.joinable()) {
            this->spin_thread_.join();
        }
    }
    
    std::shared_ptr<MediaState> Play::handle_event(Events event){
        if (event == Events::PLAY_PAUSE)
            return std::make_shared<Pause>();
        else if (event == Events::STOP)
            return std::make_shared<Stop>();
        else
            return this->PowerOn::handle_event(event);   
    }
    

    Pause::Pause(const rclcpp::NodeOptions &options, 
        const std::string &state_name, 
        const std::string &state_msg)
        : PowerOn(options, state_name,state_msg){}
    
    std::shared_ptr<MediaState> Pause::handle_event(Events event){
        if (event == Events::PLAY_PAUSE)
            return std::make_shared<Play>();
        else if (event == Events::STOP)
            return std::make_shared<Stop>();
        else
            return this->PowerOn::handle_event(event);
    }

    Stop::Stop(const rclcpp::NodeOptions &options, 
        const std::string &state_name, 
        const std::string &state_msg)
        : PowerOn(options, state_name,state_msg){}

    std::shared_ptr<MediaState> Stop::handle_event(Events event){
        if (event == Events::PLAY_PAUSE)
            return std::make_shared<Play>();
        else
            return this->PowerOn::handle_event(event);
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(media_system::Play)
RCLCPP_COMPONENTS_REGISTER_NODE(media_system::Pause)
RCLCPP_COMPONENTS_REGISTER_NODE(media_system::Stop)

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(media_system::Play, media_system::MediaState)
PLUGINLIB_EXPORT_CLASS(media_system::Pause, media_system::MediaState)
PLUGINLIB_EXPORT_CLASS(media_system::Stop, media_system::MediaState)