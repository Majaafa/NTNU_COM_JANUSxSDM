#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <stdio.h>
#include <functional>
#include <memory>

#include "../../../../lib/janusxsdm/janusxsdm.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std;

string JANUSPATH = "../lib/janus-c-3.0.5/bin/";
string SDMPATH = "../lib/sdmsh/";

int JANUS_RX_PORT = 9977;
int JANUS_TX_PORT = 9962;

string received_data;

janusxsdm::connection modem("192.168.0.189",JANUSPATH,SDMPATH, JANUS_RX_PORT,JANUS_TX_PORT);


class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("masterTopic", 10);
      MinimalPublisher::timer_callback();
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = received_data;
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int node_run(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin_some(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

int janus_rx(int timeOut_interval){                          //Reception from JANUS and SDMSH
    std::cout << "Listening for signal.. 1 " << "\n";
    std::chrono::duration<double> t;
    std::cout << "Listening for signal.. 2 " << "\n";
    t = std::chrono::duration<double> {timeOut_interval};       //Setting the timeout interval to seconds in variable t
    std::cout << "Listening for signal.. 3 " << "\n";    
    string response;
    std::cout << "Listening for signal.. 4 " << "\n";     
    modem.listen(response, t);
    std::cout << "Listening for signal.. 5 " << "\n"; 
    received_data = response;

    // t = std::chrono::duration<double> {1};
    // modem.listen(response, t); 

    std::cout << "Listening for signal.. 6 " << "\n"; 
    std::cout << received_data << "\n"; 
    std::cout << "Ros2 etter " << "\n";

    return 0;
}

int main(){
    modem.sdmconfDialogue();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging

    while(true){
        std::this_thread::sleep_for(1000ms);
        janus_rx(12);
    }
    return 0;
}
