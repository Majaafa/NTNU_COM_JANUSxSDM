/*
This example shows how the "Evo_janusXsdm" library can be used. 
The code below uses the RX family to reecieve the JANUS packets. 
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <functional>
#include <memory>
#include <chrono>
#include <poll.h>
#include <array>

#include "../lib/Evo_janusXsdm/Evo_janusXsdm.cpp"

//Constructor parameters for Evo_janusXsdm.h
std::string JANUSPATH = "../lib/janus-c-3.0.5/bin/";
std::string SDMPATH = "../lib/sdmsh/";
std::string IP = "192.168.0.189";
int JANUS_RX_PORT = 9955;
int JANUS_TX_PORT = 9955;
float STREAMFS = 250000.0;

//Global variables
std::string response;
int timeout = 500;    //NB should be small in this example, but should not be zero!
int mode = 0;
std::string myString;


int main()
{ 
    //Constructing a connection object
    Evo_janusXsdm::connection modem(IP, JANUSPATH, SDMPATH, JANUS_RX_PORT, JANUS_TX_PORT, STREAMFS); 
    
    //Configures modem and sets preamble
    modem.sdmConfigAir();
    std::this_thread::sleep_for(500ms);        //TODO:Test if sleep is needed
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);        //TODO:Test if sleep is needed

    //runs the JANUS and SDMSH commands for listening
    int fd_listen = modem.startRX();
    std::this_thread::sleep_for(500ms); 

    //listens for three packets
    while(true){
        std::string respons;
        std::array<std::string,4> responsFromFrame = modem.listenRX(fd_listen, respons, timeout);
        if(responsFromFrame[0] != "NaN"){
            std::cout << "\n\nMessage: " << responsFromFrame[0] <<" \n" << "CRC (8 bits): " <<responsFromFrame[1]
            <<" \n" "Cargo size: " <<responsFromFrame[2] <<" \n" "Reservation Time: " <<responsFromFrame[3] 
            <<"\n"<< std::endl;
            
        }
        if(respons == "5"){
            mode = 1;
        }
        else if (respons=="10"){
            mode = 2;
        }
        
        
        // if mode is 1, this is in this example when cargo is "5", the modem will be set in sending mode, 
        // and it will transmitt the message written in the terminal
        if(mode == 1){
            //close pipe and stop listening processes
            modem.closePipeRX(fd_listen);
            modem.stopRX();

            //start transmitting
            std::cout <<"Write a message: " <<std::endl;
            std::getline(std::cin,myString);
            modem.startTX(myString); 
            //start processes for receiving
            std::this_thread::sleep_for(500ms);    //500ms break between sending and listening
            fd_listen = modem.startRX();
            std::this_thread::sleep_for(500ms); 
        }
        // if mode is 2, this is in this example when cargo is "10", the processes and terminal will be stopped
        else if(mode == 2){
            modem.closePipeRX(fd_listen);
            modem.stopRX();
            break;
        }
    }

}