/*
This example shows how the "Evo_janusXsdm" library can be used. 
The code below uses the listenOnceRXsimple funtion to reecieve the JANUS packets. 
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
std::string IP = "192.168.0.199";
int JANUS_RX_PORT = 9955;
int JANUS_TX_PORT = 9955;
float STREAMFS = 250000.0;

//Global variables
std::string responsOnce;
int timeout = 60000;

int main()
{ 
    //Constructing a connection object
    Evo_janusXsdm::connection modem(IP, JANUSPATH, SDMPATH, JANUS_RX_PORT, JANUS_TX_PORT, STREAMFS); 

    //Configures modem and sets preamble
    modem.sdmConfigAir();
    std::this_thread::sleep_for(500ms);         //TODO:Test if sleep is needed
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);         //TODO:Test if sleep is needed
    /*
    std::ofstream MyFile5 ("rx_cargo.txt", std::ios::app);
    MyFile5 <<"--New-Test--"<<"\n";               
    MyFile5.close(); 
    */

    while(true)
    {
        std::array<std::string,4> responsFromFrame = modem.listenOnceRXsimple(responsOnce,timeout);
        std::cout << "\n\nMessage: " << responsFromFrame[0] <<" \n" << "CRC (8 bits): " <<responsFromFrame[1]
        <<" \n" "Cargo size: " <<responsFromFrame[2] <<" \n" "Reservation Time: " <<responsFromFrame[3] 
        <<"\n"<< std::endl;
        /*
        std::ofstream MyFile5 ("rx_cargo.txt", std::ios::app);
        MyFile5 <<responsOnce <<"\n";               
        MyFile5.close();  
        */
        //std::this_thread::sleep_for(2000ms);       //used for debugging
    }
}