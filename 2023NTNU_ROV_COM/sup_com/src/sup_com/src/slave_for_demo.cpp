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

using std::placeholders::_1;
using namespace std;

string JANUSPATH = "../lib/janus-c-3.0.5/bin/";
string SDMPATH = "../lib/sdmsh/";

int JANUS_RX_PORT = 9920;
int JANUS_TX_PORT = 9914;

string received_data;
string prev_data = "";

janusxsdm::connection modem("192.168.0.199",JANUSPATH,SDMPATH, JANUS_RX_PORT,JANUS_TX_PORT);

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "slaveTopic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      received_data = msg->data.c_str();
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int sub_run(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

int ros(){
    sub_run(0,0);
    return 0;
}

int janus_tx(string data){
    cout << "Sending: " << data << endl;
    modem.sendSimple(data);
    return 0;
}

int init(){
    modem.sdmconfDialogue();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging
    modem.setPreamble();
    std::this_thread::sleep_for(500ms); 
    while(true){
        std::this_thread::sleep_for(1000ms);
        if(received_data != prev_data){
            janus_tx(received_data);
            prev_data = received_data;
        }
    }
}

int main(){
    thread sub(ros);
    thread processing(init);

    sub.join();
    processing.join();

    return 0;
}
