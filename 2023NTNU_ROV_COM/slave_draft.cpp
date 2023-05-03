#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <stdio.h>

#include "lib/janusxsdm/janusxsdm.cpp"

using namespace std;
using namespace std::chrono_literals;

string fname = "info_files/node_info.csv";

bool master;

int time_slot_duration;
int node_id;

string master_mac;
string node_mac;
string node_alias;
string min_f;
string max_f;
string protocols;

//Globals
std::string JANUSPATH = "lib/janus-c-3.0.5/bin/";
std::string SDMPATH = "lib/sdmsh/";

int JANUS_RX_PORT = 9988;
int JANUS_TX_PORT = 9977;

janusxsdm::connection con("192.168.0.189",JANUSPATH,SDMPATH,JANUS_RX_PORT,JANUS_TX_PORT);

string janus_rx(int timeOut_interval){
    string response;
    con.listen(response, 15s);
    return response;
}

int janus_tx(string command){
    con.sendSimple(command);
    return 1;
}

void edit_node_info(string mac){
    string line;
    ifstream fin;
    string addLine = mac + ",master";
    
    fin.open(fname);
    // contents of path must be copied to a temp file then
    // renamed back to the path file
    ofstream temp;
    temp.open("temp.csv");
    temp << addLine << endl;
    while (getline(fin, line)) {
        if(line != ""){
            temp << line << endl;
        }
    }

    temp.close();
    fin.close();
    // required conversion for remove and rename functions
    const char * p = "node_info.csv";
    std::remove(p);
    rename("temp.csv", p);
}

void response_check(string response){
    size_t spos = response.find("-");
    size_t epos;
    string command = response.substr(spos+1, 2);
    spos = 10;          //standard for all commands
    epos = response.find(";");
    string rec_mac = response.substr(spos, epos-spos);
    if(command == "MD"){
        spos = epos+1;
        epos = response.find(">>");
        string data_str = response.substr(spos, epos-spos);
        string tmp;
        stringstream ss(data_str);
        vector<string> words;
        while(getline(ss,tmp,'|')){
            words.push_back(tmp);
        }
        if(rec_mac == node_mac && node_alias == words[1] && min_f == words[2] && max_f == words[3]&&protocols==words[4]){
            string send_command = "//SUP-NC//OK>>";;
            janus_tx(send_command);
        }
        else {
            string send_command = "//SUP-NC//"+node_mac+";"+node_alias+"|" +min_f+"|"+max_f+"|"+protocols+">>";
            janus_tx(send_command);
        }
    }
    else if(command == "MR"){
        if(!master){
            string send_command = "//SUP-NC//"+node_mac+";"+node_alias+"|" +min_f+"|"+max_f+"|"+protocols+">>";
            janus_tx(send_command);
            edit_node_info(rec_mac);
        }
    }
    else if(command == "MC" && rec_mac == node_mac){
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
    }
    else if(command == "MT" && rec_mac == node_mac){
        string send_command = "her er det data";
        janus_tx(send_command);
    }
    else if(command == "RA" && rec_mac == node_mac){
        spos=epos+1;
        epos=response.find(">>");
        string master_mac = response.substr(spos,epos-spos);
        edit_node_info(master_mac);
        cout << master_mac << endl;
        string send_command = "//SUP-NC//"+node_mac+";"+node_alias+"|" +min_f+"|"+max_f+"|"+protocols+">>";
        janus_tx(send_command);
    }
    else if(command == "TS"){

    }
    else{
        cout << "Unknown command or wrong MAC-address!"<<endl;
    }
}

void read_node_info(){
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
    for(int i = 0;i<row.size();i++){
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
        else if(str=="" && i==0){
            cout << "No master detected" << endl;
            master = false;
        }  
    }
}

void network_transmission(){
    while(true){
        string response = janus_rx(5);
        response_check(response);
    }
}

void pre_transmission(){
    read_node_info();
    if(!master){
        response_check(janus_rx(5));
        read_node_info();
    }
    else{
        network_transmission();
    }
}

int main(){
    cout << "Node starting.." << endl;
    pre_transmission();
    return 1;
}