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

std::string JANUSPATH = "lib/janus-c-3.0.5/bin/";
std::string SDMPATH = "lib/sdmsh/";
string IP ="192.168.0.199";
int tall = 1;
int JANUS_RX_PORT = 9920;
int JANUS_TX_PORT = 9914;




int main()
{ 
    Evo_janusXsdm::connection modem(IP,JANUSPATH,SDMPATH, JANUS_RX_PORT,JANUS_TX_PORT); //Constructing a connection object;
    modem.sdmConfigAir();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging
    modem.setPreamble();
    std::this_thread::sleep_for(500ms);        //Mainly for debugging


//--------------- Tid ----------------------   

    auto start = std::chrono::system_clock::now();
    // Some computation here
    auto end = std::chrono::system_clock::now();
 
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  
    // std::ctime(&end_time)                    // legge in Dato med tid 
    // elapsed_seconds.count() << "s"           //Arduino 

//--------------write to file-----------------  
    //cout << "Comment: ...?" <<endl;
    string comment; 


    std::ofstream MyFilew2 ("TX_logg_testdag.txt", std::ios::app);
    MyFilew2 << "-----------------------------------------------" << "\n"
                << "Start New Sending:  " << "\n\n"
                << "Kl.                 " << std::ctime(&end_time) 
                << "Modem.PI.           " << IP << "\n"
                << "--stream-fs         " << "250000" << "\n"                                   //Må skiftes 
                << "TCP port            " << to_string(JANUS_TX_PORT) << "\n"
                << "Comment:            " << to_string(JANUS_TX_PORT)<< "\n\n";                  
    MyFilew2.close(); 

        //--------------write to TOREADFILE-----------------  
        std::ofstream MyFilew333ss33 ("TX_logg_NOREAD.txt", std::ios::app);
        MyFilew333ss33 << "START!!!";
        MyFilew333ss33.close(); 
        //-------------------------------------------

//----------------------------------------   
    
    while(true){

        tall= tall+10;                                  // "+ x", sike med +x pr sending      
        string message =  "1234";
        

        //--------------write to file-----------------  
        std::ofstream MyFilew3 ("TX_logg_testdag.txt", std::ios::app);
        MyFilew3 << "Message:          " <<message<< "\n";
        MyFilew3.close(); 
        //-------------------------------------------

        //--------------write to TOREADFILE-----------------  
        std::ofstream MyFilew3n2 ("TX_logg_NOREAD.txt", std::ios::app);
        MyFilew3n2 << ";" << message << ";" <<std::ctime(&end_time);
        MyFilew3n2.close(); 
        //-------------------------------------------

        auto start = std::chrono::system_clock::now();
        modem.startTX(message);  
        auto end = std::chrono::system_clock::now();   
        std::chrono::duration<double> elapsed_seconds = end-start;     

        //--------------write to file-----------------  
        std::ofstream MyFilew4 ("TX_logg_testdag.txt", std::ios::app);
        MyFilew4        << "Duration:         " <<elapsed_seconds.count()<< " sec" <<"\n\n";
        MyFilew4.close(); 
        //-------------------------------------------

        //--------------write to TOREADFILE-----------------  
        std::ofstream MyFilewn4 ("TX_logg_NOREAD.txt", std::ios::app);
        MyFilewn4 << ";" <<std::ctime(&end_time)
                 << ";" <<elapsed_seconds.count()<<"<;>\n";
        MyFilewn4.close(); 
        //-------------------------------------------

        std::this_thread::sleep_for(9000ms);            // intervall mellom sendiger 
        
        }
    }


