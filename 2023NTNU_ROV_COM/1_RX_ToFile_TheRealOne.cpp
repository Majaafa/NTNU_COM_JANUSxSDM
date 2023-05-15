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

int JANUS_RX_PORT = 9955;
int JANUS_TX_PORT = 9955;
string IP = "192.168.0.198";
float STREAMFS = 250000.0;

int n = 0;
int forventer = 0;
int loss = 0;
int corrupt = 0;

string returnMessage;
string response;
string janus_frame;
char janus_char[1024] ={};
string prevMessage = "maja";

int main()
{ 
    //std::cout << "Tester\n";
    Evo_janusXsdm::connection modem(IP, JANUSPATH, SDMPATH, JANUS_RX_PORT, JANUS_TX_PORT, STREAMFS); //Constructing a connection object;
    std::string responsOnce;
    chrono::duration<double> t;
    t = chrono::duration<double> {60s};       //Setting the timeout interval to seconds in variable t

    auto start = chrono::system_clock::now();
    auto end = chrono::system_clock::now();

    chrono::duration<double> elapsed_seconds = end-start;
    time_t end_time = chrono::system_clock::to_time_t(end);

    modem.sdmConfigAir();
    this_thread::sleep_for(500ms);        //Mainly for debugging
    modem.setPreamble();
    this_thread::sleep_for(500ms);        //Mainly for debugging

    int fd_ldummyisten = modem.SdmshTORawFile();

    while(true){

        std::string respons;
        std::string comment;

        std::cout << "-- 1 --" << std::endl;
        std::getline(cin,comment);
        int fd_listen1 = modem.JanusFromRawFile();
        modem.listenRX(fd_listen1, respons);
        close(fd_listen1);

        std::cout << "-- 2 --" << std::endl;
        std::getline(cin,comment);
        int fd_listen2 = modem.JanusFromRawFile();
        modem.listenRX(fd_listen2, respons);
        close(fd_listen2);

        std::cout << "-- 3 --" << std::endl;
        std::getline(cin,comment);
        int fd_listen3 = modem.JanusFromRawFile();
        modem.listenRX(fd_listen3, respons);
        close(fd_listen3);
        
    } 
}