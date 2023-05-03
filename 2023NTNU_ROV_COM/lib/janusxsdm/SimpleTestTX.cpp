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
    janusxsdm::connection modem("192.168.0.189", JANUSPATH, SDMPATH, 9938, 9955); //Constructing a connection object;

    
    while(true){

        }
    }