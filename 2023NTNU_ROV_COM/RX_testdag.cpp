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

    chrono::duration<double> t;
    t = chrono::duration<double> {60s};       //Setting the timeout interval to seconds in variable t

    auto start = chrono::system_clock::now();
    auto end = chrono::system_clock::now();

    chrono::duration<double> elapsed_seconds = end-start;
    time_t end_time = chrono::system_clock::to_time_t(end);

    cout << "Write a comment to your test: "<< endl;
    string comment;
    getline(cin,comment);
    
    string config = modem.sdmConfig();
    this_thread::sleep_for(500ms);        //Mainly for debugging
    modem.setPreamble();
    this_thread::sleep_for(500ms);        //Mainly for debugging

    //while

    int fd_listen = modem.startRx();
    this_thread::sleep_for(500ms); 

    std::ofstream MyFile121 ("myRX_test.txt", std::ios::app);
    MyFile121 << "\n" << "----New test ----" << ctime(&end_time)<< "\n"
              << "Modem IP: "<< IP <<"\n"
              << "Port: " << to_string(JANUS_RX_PORT) <<"\n"
              << "Config: " << config << "\n"
              << "Comment: " << comment << "\n"; 
    MyFile121.close(); 

    while(true){
        //cout << "wiii jeg starter while!"<<endl;
        struct pollfd pfd; 
        //int timeout = 50000;         // 5 sek 
        int timeout = 120000;
        pfd.fd = fd_listen;
        pfd.events = POLLIN;
        int gatekeeper = poll(&pfd, 1, timeout);

        if (gatekeeper == -1) 
        {
            perror("poll");
        }
        else if (gatekeeper== 0) 
        {
            printf("timeout, going to 5.3\n");
            break;
            
        } 
        else 
        {
            int rdstate = read(fd_listen, &janus_char, sizeof(janus_char));
            switch (rdstate){
                case -1:                    
                    if(errno == EAGAIN)                             //Read(): Error handler, if empty 
                    {
                        //?
                        break;
                    }
                    else                                            //Read(): Error handler, if error 
                    {
                        printf("\n Read from pipe failed:  \n");
                        perror("read");
                        exit(4);
                        //??
                    }
                default:                                            //Read(): OK
                    janus_frame += janus_char;                         //Appending "ch1" to "janusframe1"
                  /*
                    // WM: dette er for å debuge koden, vil printe tid. 
                    std::ofstream MyFile11("newERR.txt", std::ios::app);
                    MyFile11 << janus_char;               
                    MyFile11.close(); 
                   */ 
                    break;
            }

            /* Look For A Massage In String: "janusframe1"*/ 
    
            if(janus_frame.find("Packet         : Cargo (ASCII)                                :") != std::string::npos)
            {
                std::string mess = modem.findInJanus_frame("message",janus_frame);
                cout <<"My message: "<< mess <<endl;
                this_thread::sleep_for(200ms);  
                /*
                //cout << "\nMassage found!" <<endl;

                string idStr = "Packet         :   Payload                                    : ";
                string RT = "Packet         :     Reservation Time (7 bits)                : ";
                string CRC = "Packet         :   CRC (8 bits)                               : ";
                string cargo_s = "Packet         :   Cargo Size                                 : ";
                string endStr = "\n";
                size_t spos, epos, pos; 

                size_t posRT, posCRC, posCargo; 
                size_t sposRT, sposCRC, sposCargo; 
                size_t eposRT, eposCRC, eposCargo; 

                pos = janus_frame.find(idStr);
                posRT = janus_frame.find(RT);
                posCRC = janus_frame.find(CRC);
                posCargo = janus_frame.find(cargo_s);

                if(pos != std::string::npos)                        // npos error handeling for .find(), if the sring is not found.                     
                {
                    
                    spos = pos + idStr.length();
                    //cout << "Cargo found in " << spos << endl;
                    epos = janus_frame.find(endStr, spos);
                    string message = janus_frame.substr(spos, epos-spos);

                    sposCRC = posCRC + CRC.length();
                    eposCRC = janus_frame.find(endStr, sposCRC);
                    string CRC_new = janus_frame.substr(sposCRC, eposCRC-sposCRC);

                    sposCargo= posCargo + cargo_s.length();
                    eposCargo = janus_frame.find(endStr, sposCargo);
                    string cargo_s_new = janus_frame.substr(sposCargo, eposCargo-sposCargo);

                    sposRT = posRT + RT.length();
                    eposRT = janus_frame.find(endStr, sposRT);
                    string RT_new = janus_frame.substr(sposRT, eposRT-sposRT);
                    
                    if(message != prevMessage){*/
                       
                       /*if(typeid(message) == typeid(to_string(forventer))){
                            if(forventer == 0 || forventer > stoi(message)){
                                forventer = stoi(message); 
                            }
                            if(message != to_string(forventer)){
                                loss += stoi(message)-forventer;
                                forventer = stoi(message)+1;

                            }
                        forventer = stoi(message)+1;
                       }
                     
                        else{
                            ++corrupt;
                        }
                    
                       *//*
                    
                     
                        ++n;
                        
                        prevMessage = message;
                        cout << "Cargo is: " << message << endl;


                        // WM: dette er for å debuge koden, vil printe tid. 
                        std::ofstream MyFile ("newSVAR.txt", std::ios::app);
                        MyFile << "\n" << "Rx: svar: " << message << "\n";               
                        MyFile.close(); 
                        this_thread::sleep_for(200ms); 


                        std::ofstream MyFileRX ("myRX_test.txt", std::ios::app);
                        MyFileRX << "\n" <<"\n"
                                << "Time : " << ctime(&end_time)
                                << "Messagenr.: " << n <<"\n"
                                << "Recieved: " << message <<"\n"
                                << "CRC (8 bits): " << CRC_new <<"\n"
                                << "Recervation time: " << RT_new <<"\n"
                                << "Cargo size: "<< cargo_s_new<<"\n\n";

                               
                        MyFileRX.close(); 




                    }
                    
                }*/
                if(mess != "nei")                        // npos error handeling for .find(), if the sring is not found.                     
                {
                janus_frame = "";
                janus_char[1024] ={};
                }
            }
 
        }
    }
    
    close(fd_listen);

        /*
        cout << "UTE! "<< endl; 
        if(response ==""){
                cout << "Empty message!" << endl;
        }
        else{
                cout << "\n Response: "<< response << endl; 
        }
        */
    
}