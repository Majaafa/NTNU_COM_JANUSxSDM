/*
This example shows how the "Evo_janusXsdm" library can be used. 
The code below uses the startTX function to transmit the JANUS packets. 
*/

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <vector>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <stdio.h>
#include <functional>
#include <memory>

#include "../lib/Evo_janusXsdm/Evo_janusXsdm.cpp"

//Constructor parameters for Evo_janusXsdm.h
std::string JANUSPATH = "../lib/janus-c-3.0.5/bin/";
std::string SDMPATH = "../lib/sdmsh/";
std::string IP ="192.168.0.198";
int JANUS_RX_PORT = 9920;
int JANUS_TX_PORT = 9914;
float STREAMFS = 250000.0;

//Global variables
std::string myString = "1";
std::string comment;
int tall = 0;

int main()
{ 
    //Constructing a connection object
    Evo_janusXsdm::connection modem(IP,JANUSPATH,SDMPATH, JANUS_RX_PORT,JANUS_TX_PORT,STREAMFS); 

    //Configures modem and sets preamble
    modem.sdmConfigAir();
    std::this_thread::sleep_for(500ms);        //TODO:Test if sleep is needed
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);        //TODO:Test if sleep is needed

    while(true)
    {                                 
        //std::cout <<"Press enter to send message" <<std::endl;
        //std::getline(std::cin,comment);
        modem.startTX(std::to_string(tall));  
        tall += 1;
        std::this_thread::sleep_for(6000ms);    //interval between sending
    }
}