// Simple code demonstrating how to recieve data using janusxsdm

#include<stdlib.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<signal.h>
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<sstream>
#include<ctime>
#include<cstdio>
#include<functional>
#include<memory>
#include<chrono>
#include<iostream>
#include<fstream>
#include<poll.h>
#include<ctime>
#include<array>


#include "lib/janusxsdm/Evo_janusXsdm.cpp"

//Global
using namespace std;

string JANUSPATH = "lib/janus-c-3.0.5/bin/";
string SDMPATH = "lib/sdmsh/";

int JANUS_RX_PORT = 9938;
int JANUS_TX_PORT = 9955;
int n = 0;
int forventer = 0;
int loss = 0;
int corrupt = 0;
string IP = "192.168.0.189";


string returnMessage;
string response;
string janus_frame;
char janus_char[1024] ={};
string prevMessage = "maja";

int main()
{ 
    //std::cout << "Tester\n";
    Evo_janusXsdm::connection modem(IP, JANUSPATH, SDMPATH, JANUS_RX_PORT, JANUS_TX_PORT); //Constructing a connection object;
    std::string responsOnce;
    chrono::duration<double> t;
    t = chrono::duration<double> {60s};       //Setting the timeout interval to seconds in variable t

    auto start = chrono::system_clock::now();
    auto end = chrono::system_clock::now();

    chrono::duration<double> elapsed_seconds = end-start;
    time_t end_time = chrono::system_clock::to_time_t(end);

    //cout << "Write a comment to your test: "<< endl;
    //string comment;
   // getline(cin,comment);
   

    modem.dummyFlushJanusTX();



    //while ???wilhelm?
/*
    int fd_listen = modem.startRX();
    this_thread::sleep_for(500ms); 

    std::ofstream MyFile121 ("myRX_test.txt", std::ios::app);
    MyFile121 << "\n" << "----New test ----" << ctime(&end_time)<< "\n"
              << "Modem IP: "<< IP <<"\n"
              << "Port: " << to_string(JANUS_RX_PORT) <<"\n";
              //<< "Config: " << config << "\n"m
              //<< "Comment: " << comment << "\n"; 
    MyFile121.close(); 

    while(true){
        std::string respons;
        std::array<string,6> noe = modem.listenRX(fd_listen, respons);
        std::cout << "\n\nMessage: " << noe[0] <<" \n" << "CRC (8 bits): " <<noe[1]<<" \n" "Cargo size: " <<noe[2] <<" \n" "Reservation Time: " <<noe[3] <<"\n"<< std::endl;
        //modem.closePipeRX(fd_listen);
    }
    
    close(fd_listen);
*/


    
}