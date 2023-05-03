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
#include <chrono>
#include <ctime>
#include <cstdio>
#include <stdio.h>
#include <functional>
#include <memory>

#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "lib/janusxsdm/janusxsdm.cpp"

//Global
std::string JANUSPATH = "lib/janus-c-3.0.5/bin/";
std::string SDMPATH = "lib/sdmsh/";

int JANUS_RX_PORT = 9960;
int JANUS_TX_PORT = 9961;

using namespace std::chrono_literals;
using namespace std;

int main()
{ 
    //std::cout << "Tester\n";
    janusxsdm::connection modem("192.168.0.189", JANUSPATH, SDMPATH, 9938, 9955); //Constructing a connection object;
    std::string returnMessage;
    modem.sdmconfDialogue();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging

    while(true){
       //Mainly for debugging
//----------------------------------------
        //std::cout <<  "---- 1\n"; 
        std::chrono::duration<double> t;
        //std::cout <<  "---- 2\n"; 
        t = std::chrono::duration<double> {60s};       //Setting the timeout interval to seconds in variable t
        //std::cout <<  "---- 3\n";
        std::string response;
        //std::cout <<  "---- 4\n";
        modem.listen(response, t);
        cout << "UTE! \n"; 
        //std::cout <<  "---- 5\n";
        if(response ==""){
                cout << "Empty message!" << endl;
        }
        else{
                cout << "\n Response: "<< response << endl; 
        }
        
        
        //std::this_thread::sleep_for(500ms);      
        //std::cout <<  "---- 6\n";
        //std::this_thread::sleep_for(10000ms);        //Mainly for debugging
//----------------------------------------

        }
    }