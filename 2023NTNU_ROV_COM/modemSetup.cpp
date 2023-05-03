// Simple code demonstrating how to set source level and upload preamble

#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<string.h>
#include<chrono>
#include<unistd.h>
#include<signal.h>

#include "lib/janusxsdm/janusxsdm.cpp"

//Globals
std::string JANUSPATH = "lib/janus-c-3.0.5/bin/";
std::string SDMPATH = "lib/sdmsh/";

int JANUS_RX_PORT = 9988;
int JANUS_TX_PORT = 9977;

using namespace std::chrono_literals;

int main()
{
    
    std::cout << "Setting source level to low and configuring preamble.." << std::endl;
    janusxsdm::connection modem("192.168.0.199", JANUSPATH, SDMPATH, 9007, 9045); //Constructing a connection object;
    modem.sdmconfDialogue();
    /*
    modem.sdmconf();
    std::this_thread::sleep_for(1000ms);
    modem.setPreamble();
    std::cout << "Setup done" << std::endl;
    */

}