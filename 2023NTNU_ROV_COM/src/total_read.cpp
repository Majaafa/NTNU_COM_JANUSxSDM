#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <cstdlib>

using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

std::string output_log;     //current package reading
std::string output_log_prev = "";  //previous package
std::string packet_line;
bool r_flag = false;  //read-flag, in effect when package-data has been identified by ""

using namespace std::chrono_literals;

std::string DATAFILE = "../lib/janus-c-3.0.5/data/decodedJanus.txt";

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    MinimalPublisher::timer_callback();
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = packet_line;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int node_run(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}

void packet_extraction(){       //extracting the packet cargo from the header
std::cout << "Extracting the packet" << "\n";
    packet_line = "";       //line containing package data
    for (std::size_t i = 0, length = output_log.length(); i != length; ++i){       //checking all symbols in the received package
        std::string tmp_string(1, output_log[i]);       //buffer for conversion of current symbol from char to string
        if(output_log[i] == '"' && r_flag == false){
            r_flag = true;
        }
        else if(output_log[i] == '"' && r_flag == true){
            r_flag = false;
        }
        if(r_flag == true){
            if(output_log[i] != '"'){
                packet_line.append(tmp_string);     //appending the symbol to the package data line if r_flag is active
            }
        }
        tmp_string = "";
    }
    r_flag = false;
    node_run(0, 0);
}

void read_output(){         //reading the log file upon change
std::cout << "Reading file" << "\n";
    ifstream indata; // indata is like cin
    std::string output = ""; // variable for input value
    output_log = "";
    indata.open(DATAFILE); // opens the file
    if(!indata) { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }
    indata >> output;
    while ( !indata.eof() ) { // keep reading until end-of-file
        indata >> output; // sets EOF flag if no value found
        output_log.append(output + " ");        //appending each word to the output log, and adding a space following
    }
    std::cout << "File read" << "\n";
    indata.close();
    //std::ofstream file("log_output.txt");       //wipes the log file clean after reading, ready for new data
}

void wait(){
  std::cout << "Waiting..." << "\n";
	std::this_thread::sleep_for(std::chrono::milliseconds(15000));  //wait interval
}

void output_comparison(){       //comparing current output to previous, and extracting the package data if new data has arrived
std::cout << "Comparing" << "\n";
    if(output_log != output_log_prev){
      std::cout << "Not the same" << "\n";
        output_log_prev = output_log;
        packet_extraction();
    }
}

int main()
{
  std::cout << "Program commencing" << "\n";
  while(true){
      read_output();
      output_comparison();
      wait();
  }
  return 0;
}
