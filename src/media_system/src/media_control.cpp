#include <functional>
#include <memory>


#include <stdexcept>
#include <thread>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

#include "media_system/media_control.hpp"

namespace media_system{

    KeyboardReader::KeyboardReader(){
        // get the console in raw mode
        if (tcgetattr(0, &cooked_) < 0) 
            throw std::runtime_error("Failed to get old console mode");
        
        struct termios raw;
        memcpy(&raw, &cooked_, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        raw.c_cc[VTIME] = 1;
        raw.c_cc[VMIN] = 0;

        if (tcsetattr(0, TCSANOW, &raw) < 0) 
            throw std::runtime_error("Failed to set new console mode");
    }

    char KeyboardReader::readOne()
    {
        char c = 0;

        int rc = read(0, &c, 1);

        if (rc < 0) 
            throw std::runtime_error("read failed");

        return c;
    }

    KeyboardReader::~KeyboardReader(){
        tcsetattr(0, TCSANOW, &cooked_);
    }

    MediaControl::MediaControl() : Node("media_control"){   

        //Parameters
        for (const auto& k : this->keyMap) 
            this->declare_parameter(eventStringMap[k.second],k.first);

        //Monitor Parameter Change 
        this->param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto cb = [this](const rclcpp::Parameter & p) {
            this->parameter_change(p);
        };
        this->cb_handle_ = 
            this->param_subscriber_->add_parameter_callback("change_key_map", cb);
    

        //Client definition
        this -> client_ptr_ = rclcpp_action::create_client<MediaAction>(this,"media_action");

        this-> console_msg_= "Media Actions";
        console_commands_ = "";
        for (const auto& k : this->keyMap) 
            console_commands_   +=" <"+eventStringMap[k.second]+": "+k.first+">";

        this->keyboard_ptr_ = std::make_shared<KeyboardReader>();

        std::cout << "\033[?1049h";
        this->keyLoop();
    }

    MediaControl::~MediaControl(){
        // Switch back to the normal screen
        std::cout << "\033[?1049l";

        this->keyboard_ptr_.reset();
    }

    void MediaControl::parameter_change(const rclcpp::Parameter & p){
        
        //Find Event
        auto it1 =  eventStringMap.begin();
        while (it1->second != p.get_name())
            it1++;
        auto event = it1->first;

        //Find Key Previous
        auto it2 = this->keyMap.begin();
        while(it2->second != event) 
            it2++;

        try{    
            //Replace Key
            this->keyMap[p.as_string()[0]] = event;
            this->keyMap.erase(it2); 
            
            this->set_parameter(rclcpp::Parameter(p.get_name(),p.as_string()[0]));
            RCLCPP_INFO(this->get_logger(), "Change KeyMap for \"%s\" to %c",
            p.get_name().c_str(), p.as_string()[0]);
        }
        catch(const std::out_of_range& e){
            //Revert Mapping
            this->set_parameter(rclcpp::Parameter(p.get_name(),it2->first));
            RCLCPP_INFO(this->get_logger(), "Revert KeyMap for \"%s\" to %c",
            p.get_name().c_str(), it2->first);
        } 

        console_commands_ = "";
        for (const auto& k : this->keyMap) 
            console_commands_   +=" <"+eventStringMap[k.second]+": "+k.first+">";

    }

    int MediaControl::keyLoop(){

        while(running){

            // Clear the alternate screen
            std::cout << "\033[2J";

            // Retrieve terminal dimensions
            winsize size;
            ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);

            // Find Center Console
            int mid_row = size.ws_row / 2;
            int mid_col = size.ws_col / 2;

            // Print Info Message
            std::cout << "\033[" << mid_row - 1 << ";" << 
                mid_col - this->console_msg_.size() / 2 << 
                "H" << this->console_msg_;

            // Print Commands Options
            std::cout << "\033[" << mid_row + 1 << ";" << 
                mid_col - this->console_commands_.size() / 2 << 
                "H" << this->console_commands_;

            // Move Cursor to the End
            std::cout << "\033[" << mid_row + 3 << ";" << mid_col - 5 << "H";

            // Flush the output
            std::cout.flush();

            char input;
            //Wait for command
            try{
                input = this->keyboard_ptr_->readOne(); 
            } catch (const std::runtime_error &) {
                perror("read():");
                return -1;
            }

            try{
                Events event = this->keyMap.at(input);  
                // Move Cursor to the End
                std::cout << "\033[" << size.ws_row - 2 << ";1H "<< 
                    eventStringMap[event];    
                this->send_goal(event);
                sleep(2);
            } catch (const std::out_of_range& e) {       
            }
        }
        return 0;

    }

    void MediaControl::send_goal(Events event){
        //action msg
        using MediaAction = action::Media;

        auto goal = MediaAction::Goal();
        goal.action_type = static_cast<uint8_t>(event);
        auto send_goal_options = rclcpp_action::Client<MediaAction>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<MediaAction>::SharedPtr goal_handle){
            RCLCPP_INFO(this->get_logger(), "Action Requested received");
            this->goal_handle_ = goal_handle;
        };

        send_goal_options.result_callback = 
            [this](const rclcpp_action::Client<MediaAction>::WrappedResult & result){
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Action Done");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Action was aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Action was canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }
            };
        this->client_ptr_->async_send_goal(goal, send_goal_options);
    }

    void quit(int sig){
        (void)sig;
        running = false;
      }
      
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, media_system::quit);
  signal(SIGKILL, media_system::quit);

  std::shared_ptr<media_system::MediaControl> mcontrol = 
    std::make_shared<media_system::MediaControl>();

  int rc = mcontrol->keyLoop();

  mcontrol.reset();

  rclcpp::shutdown();

  return rc;
}
