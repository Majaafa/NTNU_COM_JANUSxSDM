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

#include<fstream>
#include<stdlib.h>
#include<string.h>
#include<pthread.h>
#include<array>
#include<chrono>
#include<thread>
#include<mutex>
#include<string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iterator>
#include <iostream>         /* Input/output in C++.         */
#include <cstring>          /* String manipulation in C++.  */
#include <stdio.h>          /* Basic I/O routines.          */
#include <sys/types.h>      /* Define pid_t, etc.           */
#include <unistd.h>         /* Define fork(), etc.          */
#include <sys/wait.h>       /* Define wait(), etc.          */
#include <signal.h>         /* Define signal(), etc.        */
#include <ctime>            // for feilsøking, tid vi printer til fil 
#include <poll.h>
#include "janusxsdm.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h> 


//Global
std::string JPATH = "../janus-c-3.0.5/bin/";
std::string SPATH = "../sdmsh/";
std::string mIP = "192.168.0.189";
std::string RX_PORT = "9977";
std::string messagee = ""; 

using namespace std::chrono_literals;
using namespace std;

int main()
{ 
    /*Sette opp modem*/
    std::string sdmcommandConfig = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
    FILE* terminalConfig = popen(sdmcommandConfig.c_str(), "r");
    pclose(terminalConfig);
    std::cout << "Modem with IP: " << mIP << " configured." << std::endl;

    /*Sette opp Preamble*/
    std::string sdmcommandPreamble = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;ref preamble.raw')";
    FILE* terminalPreamble = popen(sdmcommandPreamble.c_str(), "r");
    pclose(terminalPreamble);
    std::cout << "Preamble set for modem with IP: " << mIP << std::endl;
        
    /* Creating pipe Err for ./janus */
    int fdERR[2];                                      //pipe(): In The Future
    if(pipe(fdERR) == -1)                              //pipe(): Error handler 
    {
        perror("pipe");
        exit(EXIT_FAILURE);
    }
    if (fcntl(fdERR[0], F_SETFL, O_NONBLOCK) < 0)      //pipe(): Making non-blocking to prevent code lock
    {
        exit(2);
    }

    while(true) //--------------------->  
    {


        /* Creating a fork "jns_child_pid" */
        pid_t jns_child_pid = fork();               //fork(): Creating "Rx Janus Child"
        if(jns_child_pid == -1)                     //fork(): Error handler 
        {
            perror("fork");                         // Print error to "stderr"
            std::cerr << "Error 5" << std::endl;
            exit(EXIT_FAILURE);                     // EXIT_FAILURE = konstant 
        }
        else if (jns_child_pid == 0) // Gen: X-J                                                  
        {
            /*Setup for error outputstream to pipe*/
            close(fdERR[0]);                                                               //pipe(): close read
            while((dup2(fdERR[1], STDERR_FILENO) == -1) && (errno == EINTR)) {}       //pipe(): Redirecting stderr to pipe (erroroutput now goes to pipe)                 
            close(fdERR[1]);                                                          //pipe(): close write
        
            /*Make string for ./janus TCP setup*/
            char arg1[] = "sh";                                                      
            char arg2[] = "-c";
            std::string jcmd = "(cd " + JPATH + " && ./janus-rx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args listen:127.0.0.1:" + RX_PORT +" --stream-fs 96000 --verbose 1 )";
            char* jns_arg[] = {arg1, arg2, (char*)jcmd.c_str(), NULL}; 

            /*Create new prosess image*/
            execvp(jns_arg[0], jns_arg);                                            //im_1  : Replace process image, (Running a new prosess), "fork-exec"
            std::cerr << "Error 4" << std::endl;
            perror("ERROR -> jns_child_pid -> execvp");                             //im_1  : Print error to "stderr" 
            exit(EXIT_FAILURE);
        }
        else { 

            /*Let janus TCP time to setup*/
            std::this_thread::sleep_for(500ms); 

            /* Creating a fork "sdm_child_pid" */
            pid_t sdm_child_pid = fork();           //fork(): Creating "Rx Janus Child"
            if(sdm_child_pid == -1)                 //fork(): Error handler  
            {
                perror("fork");                     // Print error to "stderr"
                std::cerr << "Error 3" << std::endl;
                exit(EXIT_FAILURE);                 // EXIT_FAILURE = konstant
            }
            else if (sdm_child_pid == 0) // Gen: X-X-X-S
            {   
                /*Make string for ./janus TCP connect*/
                char arg1[] = "sh";                                         
                char arg2[] = "-c";                             
                std::string scmd = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'rx 0 tcp:connect:127.0.0.1:" + RX_PORT + "' 2>&1 >> rxlog.txt)";
                char* sdm_arg[] = {arg1, arg2, (char*)scmd.c_str(), NULL};

                /*Create new prosess image*/
                execvp(sdm_arg[0], sdm_arg);                                //im_1  : Replace process image, (Running a new prosess), "fork-exec"
                perror("ERROR ");                 //im_1  : Print error to "stderr"  (OPS! Denne feilmeldinger vil vi ikkje få!!)
                std::cerr << "Error 2" << std::endl;
                exit(0);
            }
            else
            {
                char chERR[1024] = {0};
                std::chrono::duration<double> timeout = 20s; 
                std::string janusframeERR; 
                std::chrono::steady_clock::time_point start, now;   //Time: For exiting after timeout
                start = std::chrono::steady_clock::now(); 

                while(true) //--------------------->  
                {
                    /*read from pipe for janus error stream*/
                    int rdstat1 = read(fdERR[0], &chERR, sizeof(char)); //Read(): From pipe(), write to "ch1", one character at a time.  
                    switch (rdstat1)
                    {
                    case -1:                    
                        if(errno == EAGAIN)                             //Read(): Error handler, if empty 
                        {
                            break;
                        }
                        else                                            //Read(): Error handler, if error 
                        {
                            std::cerr << "Error 1" << std::endl;   
                        }
                    default:                                            //Read(): OK
                        janusframeERR += chERR;                         //Appending "ch1" to "janusframe1"

                        std::ofstream MyFile11 ("ERR.txt", std::ios::app);
                        MyFile11 << chERR ;               
                        MyFile11.close(); 
                        break;
                    }

                    /* Look For A Massage In String: "janusframeERR"*/ 
                    if(janusframeERR.find("Packet         : Cargo (ASCII)                                :") != std::string::npos)
                    {
                        std::cout << "\nMassage found!\n";
                        break; //If massage found break while(). 
                    }

                    /* Timeout Reached: */
                    now = std::chrono::steady_clock::now();         //Time: Read time now
                    if(std::chrono::duration<double>(now - start).count() >= timeout.count())
                    {
                        std::cout << "\nlistener: Timeout reached, terminating!\n";
                        break;                                   //Exit "SendSimple"
                    }
                }//---------------------> 

                /* Processing message: sorting out the message.*/
                std::string idStr = "Packet         :   Payload                                    : ";
                std::string endStr = "\n";
                size_t spos, epos, pos;         
                pos = janusframeERR.find(idStr);

                if(pos != std::string::npos)                        // npos error handeling for .find(), if the sring is not found.                     
                {
                    spos = pos + idStr.length();
                    std::cout << "Cargo found in " << spos << std::endl;
                    epos = janusframeERR.find(endStr, spos);
                    messagee = janusframeERR.substr(spos, epos-spos);
                    std::cout << "Cargo is: " << messagee << std::endl;
        
                    std::ofstream MyFile121 ("Message.txt", std::ios::app);
                    MyFile121 << "\n" << messagee << "\n";               
                    MyFile121.close(); 
                    
                }
                else
                {
                    std::cerr << "Error cargo not found." << std::endl;   
                }
            
            } // Etter SDMSH 
        } // Etter janus
        std::cout << "Starter på nytt fra ./janus" << std::endl;
    }//--------------------->                             
} // main

    


    