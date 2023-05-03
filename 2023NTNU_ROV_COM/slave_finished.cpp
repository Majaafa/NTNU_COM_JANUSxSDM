#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <stdio.h>
#include <thread>
#include <functional>
#include <memory>

// #include "rclcpp/rclcpp.hpp"                 //For ROS 2 implementation
// #include "std_msgs/msg/string.hpp"
#include "lib/janusxsdm/janusxsdm.cpp"

using namespace std;
using std::placeholders::_1;

string fname = "src/cpp_pubsub/src/info_files/node_info.csv";      //path for node info file, remember to change these depending on ROS 2 implementation or not
string JANUSPATH = "src/cpp_pubsub/src/lib/janus-c-3.0.5/bin/";    //path to janus-folder
string SDMPATH = "src/cpp_pubsub/src/lib/sdmsh/";                  //path to sdm-folder

bool master;
bool check_done = false;
bool config = false;

int time_slot_duration;
int node_id;
int testct = 1;

string master_mac;
string node_mac;
string node_alias;
string min_f;
string max_f;
string protocols;

string data1;
string data2;
string data3;
string data4;

janusxsdm::connection modem("192.168.0.199", JANUSPATH, SDMPATH, 9926, 9916); //Constructing a connection object;

string janus_rx(int timeOut_interval){
    string response;
    //std::this_thread::sleep_for(1000ms);                  //For debugging purposes
    std::chrono::duration<double> t;
    t = std::chrono::duration<double> {timeOut_interval};
    if(modem.listen(response, t)){
        cout << "Cargo: " << response << endl;
    }
    return response;
}

int janus_tx(string command){
    //std::this_thread::sleep_for(1000ms);                  //For debugging purposes
    modem.sendSimple(command);
    return 1;
}

void edit_node_info(string mac){                //Mainly for adding a master upon reception of master ready
    string line;
    ifstream fin;
    string addLine = mac + ",master";           //Masters alias is always master
    
    fin.open(fname);
    ofstream temp;
    temp.open("src/cpp_pubsub/src/info_files/temp.csv");
    temp << addLine << endl;
    while (getline(fin, line)) {
        if(line != ""){
            temp << line << endl;
        }
    }

    temp.close();
    fin.close();
    // required conversion for remove and rename functions
    const char * p = "src/cpp_pubsub/src/info_files/node_info.csv";
    std::remove(p);
    rename("src/cpp_pubsub/src/info_files/temp.csv", p);
}

void response_check(string response){                   //Checking all received responses
    if(response != ""){
        size_t spos = response.find("-");
        size_t epos;
        string command = response.substr(spos+1, 2);
        spos = 10;          //standard for all commands
        epos = response.find(";");                  
        string rec_mac = response.substr(spos, epos-spos);          //Extracting the received MAC to speed up check process
        if(command == "MD" && rec_mac == node_mac){                 //Master checking the nodes information
            spos = epos+1;
            epos = response.find(">>");
            string data_str = response.substr(spos, epos-spos);
            string tmp;
            stringstream ss(data_str);
            vector<string> words;
            while(getline(ss,tmp,'|')){
                words.push_back(tmp);
            }
            
            if(rec_mac == node_mac && node_alias == words[1] && min_f == words[2] && max_f == words[3]&&protocols==words[4]){       //If information matches
                string send_command = "//SUP-NC//OK;>>";        
                janus_tx(send_command);
            }
            else {                                                                                                                  //If information doesn't match
                string send_command = "//SUP-NC//"+node_mac+";"+node_alias+"|" +min_f+"|"+max_f+"|"+protocols+">>";
                janus_tx(send_command);
            }
            check_done = true;                                              //Node has been checked, proceed in the initialization
        }
        else if(command == "MR"){                                              //Master ready signal, node only answers if no master is previously registered
            std::cout << "Master is ready" << "\n";
            if(!master){
                string send_command = "//SUP-NR//"+node_mac+";"+">>";       //Registration request
                janus_tx(send_command);
                edit_node_info(rec_mac);
            }
        }
        else if(command == "MC" && rec_mac == node_mac){                    //Master sends config parameters
            spos = epos+1;
            epos = response.find(">>");
            string data_str = response.substr(spos, epos-spos);
            string tmp;
            stringstream ss(data_str);
            vector<string> words;
            while(getline(ss,tmp,'|')){
                words.push_back(tmp);
            }
            time_slot_duration = stoi(words[0]);
            node_id = stoi(words[1]);
            config = true;
        }
        else if(command == "MT" && rec_mac == node_mac){                                                //Network transmission
            string send_command = "//SUP-NT//"+node_mac+";"+data1+"|"+data2+"|"+data3+"|"+data4+">>";   //Data processed in the subscriber node from ROS 2
            janus_tx(send_command);
        }
        else if(command == "MA" && rec_mac == node_mac){                    //Master acknowledges registration request
            spos=epos+1;
            epos=response.find(">>");
            cout << "Request acknowledged" << endl;
            string send_command = "//SUP-NI//"+node_mac+";"+node_alias+"|" +min_f+"|"+max_f+"|"+protocols+">>";     //Node sends its information
            check_done =true;                                                                                       //Node has been checked
            janus_tx(send_command);
        }
        else if(command == "TS"){                                       //Future implementation of tasks???
        }
        else{
            cout << "Unknown command or wrong MAC-address!"<<endl;      //If data corrupted and MAC/command is not valid
        }
    }
    else{
        cout << "No response was given" << endl;                        //No command received
    }
}

void read_node_info(){                  //Read the saved info from node_info.csv
    vector<string> row;
	string line, word;
	fstream file (fname, ios::in);
    cout << "Reading node info.." << "\n";
	if(file.is_open()){
		while(getline(file, line)){
			stringstream str(line);
            row.push_back(line);
        }
        cout << "Node info retrieved" << "\n";
	}
	else{
		cout<<"Could not open the file\n";
    }
    for(int i = 0;i<row.size();i++){                //Check for master
        vector<string> words;
        string str = row[i];
        if(str != ""){
            string tmp;
            stringstream ss(str);
            while(getline(ss,tmp,',')){
                words.push_back(tmp);
                if(tmp == "master" && i==0){
                    master_mac = words[0];
                    master = true;
                }
            }
            if(i==1){
                node_mac = words[0];
                node_alias = words[1];
                min_f = words[2];
                max_f = words[3];
                protocols = words[4];
            }
        }
        else if(str=="" && i==0){                     //If no master was found in the records
            cout << "No master detected" << endl;
            master = false;
        }  
    }
}

void network_transmission(){
    cout << "Transmission commencing" << endl;
    while(true){                                            //Easy logic for the node to listen for sign to transmit from the master
        response_check(janus_rx(time_slot_duration));
    }
}

void pre_transmission(){
    read_node_info();                       
    while(!master){                         //If no master is detected, wait for master ready signal
        response_check(janus_rx(7));
        read_node_info();
    }
    while(!check_done){                     //If check has not been done, wait for it..
        response_check(janus_rx(7));
        std::this_thread::sleep_for(1000ms);
    }
    while(!config){                                 //Wait for configuration parameters from master
        cout << "waitin for config" << endl;    
        response_check(janus_rx(time_slot_duration));
    }
    if(master && check_done && config){                         //When all steps are completed, transmission can begin
        std::cout << "Starting transmission with time slot duration: " << time_slot_duration << "s and ID " << node_id << "\n";
        network_transmission();
    }
}

int slave_run(){                         
    cout << "Node starting.." << endl;
    pre_transmission();
    return 1;
}

int main(){                                 //Standard modem configurations
    modem.sdmconf();
    std::this_thread::sleep_for(1000ms);        //Mainly for debugging
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);         //^^
    slave_run();
    return 1;
}

// class MinimalSubscriber : public rclcpp::Node                    //This code is related to the ROS 2 implementation into the slave, and uses threading to run
// {                                                                //both the subscriber and the slave logic, the name of the main-function must be changed to
// public:                                                          //protocol(int i) to work properly with this implementation.
//   MinimalSubscriber()
//   : Node("minimal_subscriber")
//   {
//     subscription_ = this->create_subscription<std_msgs::msg::String>(
//       "slaveTopic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
//   }

// private:
//   void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//   {
//     //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//     string received = msg->data.c_str();                                    //format = num|num|num|num
    
//     string tmp;
//     stringstream ss(received);
//     vector<string> words;
//     while(getline(ss,tmp,'|')){
//         words.push_back(tmp);
//     }
//     data1 = words[0];
//     data2 = words[1];
//     data3 = words[2],
//     data4 = words[3];
//     std::cout << "changing data" << std::endl;
//   }
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };

// int ros(int i)
// {
//   std::cout << "Subscriber starting\n";
//   rclcpp::init(0, 0);
//   rclcpp::spin(std::make_shared<MinimalSubscriber>());
//   rclcpp::shutdown();
  
//   return 0;
// }

// int main(){
//     thread subscriber(ros,0);
//     thread slave(protocol,0);

//     subscriber.join();
//     slave.join();
//     return 0;
// }
