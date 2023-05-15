// Simple code demonstrating how to recieve data using janusxsdm

#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<string.h>
#include<chrono>
#include<unistd.h>
#include<signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <stdio.h>
#include <functional>
#include <memory>

#include "lib/janusxsdm/Evo_janusXsdm.cpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;
float STREAMFS = 250000.0;

std::string JANUSPATH = "lib/janus-c-3.0.5/bin/";
std::string SDMPATH = "lib/sdmsh/";
string IP ="192.168.0.198";

int tall = 1000000;
int JANUS_RX_PORT = 9920;
int JANUS_TX_PORT = 9914;

int main()
{ 
    Evo_janusXsdm::connection modem(IP,JANUSPATH,SDMPATH, JANUS_RX_PORT,JANUS_TX_PORT,STREAMFS); //Constructing a connection object;
    modem.sdmConfigAir();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging
    while(true){

        tall= tall+10;                                  // "+ x", sike med +x pr sending      
        string message =  to_string(tall);
        //string message = "";
        std::string comment;
        getline(cin,comment);
        modem.startTX(message);  
        std::this_thread::sleep_for(6000ms);            // intervall mellom sendiger 
        }
    }


