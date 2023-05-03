// Library that simplifies interfacing with janus and sdm for subsea acoustic communication
// This library requires that sdmsh and janus is installed

// This library is not an ideal solution
// An ideal version was not chosen due to time-constraints, lack of experience and lack of documentation for both janus.h and sdm.h
// People seeking to implement an efficent version of JANUS with sdm should look into implementing the libraries directly

#define LIBNAME "janusxsdm"

#include<stdlib.h>
#include<string.h>
#include<pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "janusxsdm.h"
#include <poll.h>
#include <signal.h>         //Define signal(), etc.        
#include <sys/wait.h>       //Define wait(), etc.          
#include <fcntl.h>          // library for fcntl function
#include <unistd.h>         // Define fork(), etc.          
#include <sys/types.h>      //Define pid_t, etc.           
#include <stdio.h>          // Basic I/O routines.          

#include<fstream>
#include<array>
#include<chrono>
#include<thread>
#include<mutex>
#include<string>
#include <iterator>
#include <iostream>         //Input/output in C++.         
#include <cstring>          //String manipulation in C++.       
#include <ctime>            // for feilsøking, tid vi printer til fil 
#include <cstdint>

#define SRV_PORT 9910 

using namespace std::chrono_literals;


namespace janusxsdm
{
    //std::string janus::mIP, janus::JPATH, janus::SPATH;
    //int janus::RX_PORT, janus::TX_PORT;
    uint8_t connection::SDM_FRAMEHEADER[] = {0x80, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x00, 0x00};

    connection::connection(std::string modemIP, std::string JANUSPATH, std::string SDMPATH, int rxPort, int txPort)
    {
        mIP = modemIP;
        JPATH = JANUSPATH;
        SPATH = SDMPATH;
        RX_PORT = rxPort;
        TX_PORT = txPort;
    }
    int connection::sdmconf()
    {
        //Could also add setting modem in "PHY" state here (nc $IP PORT +++ATP)
        std::string sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
        FILE* terminal = popen(sdmcommand.c_str(), "r");
        pclose(terminal);
        std::cout << "Modem with IP: " << mIP << " configured." << std::endl;
        return 1;
    }
    std::string connection::sdmconfDialogue()
    {
        //Could also add setting modem in "PHY" state here (nc $IP PORT +++ATP)
        std::string input, sdmcommand, sfty = "n";
        std::cout << "Choose source level:\n 3 - -20dB (min)\n 2 - -12dB\n 1 - -6dB \n 0 - 0dB (max)\n Warning chosing a source level higher than -20dB is not recomended when testing in air as it might destroy the modems.\n";
        std::cin >> input;

        //std::cout << "Your answer was: " << input;
        int num = stoi(input);
        
        switch (num)
        {
        case 0:
            std::cout << "Warning! This setting is unsuitable for testing in air, are you sure? [y/n]\n";
            std::cin >> sfty;
            if(sfty == "y" || sfty == "yes"){
                sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 0 0')";
            }
            else{
                sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
            }
            break;

        case 1:
            std::cout << "Warning! This setting is unsuitable for testing in air, are you sure? [y/n]\n";
            std::cin >> sfty;
            if(sfty == "y" || sfty == "yes"){
                sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 1 0')";
            }
            else{
                sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
            }
            break;
        
        case 2:
            std::cout << "Warning! This setting is unsuitable for testing in air, are you sure? [y/n]\n";
            std::cin >> sfty;
            if(sfty == "y" || sfty == "yes"){
                sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 2 0')";
            }
            else{
                sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
            }
            break;
        
        case 3:
            sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
            break;
        
        default:
            std::cout << "Invalid configuration setting default value";
            sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
            break;
        }
        std::cout << sdmcommand << std::endl;
        FILE* terminal = popen(sdmcommand.c_str(), "r");
        
        //Wm lagt inn for test 
        //pid_t pid = fileno(terminal);
        //std::cout << "pid for setup" << pid << std::endl;
        //std::this_thread::sleep_for(5000ms);
        //kill(pid,SIGTERM);

        pclose(terminal);
        std::cout << "Modem with IP: " << mIP << " configured." << std::endl;
        //string config = sdmcommand.substr(sdmcommand.find(";"),sdmcommand.find(')') );
        std::string config = "30 0 "+ std::to_string(num) + " 0";

        return config;
    }
    int connection::setPreamble()
    {
        std::string sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;ref preamble.raw')";
        FILE* terminal = popen(sdmcommand.c_str(), "r");
        pclose(terminal);
        std::cout << "Preamble set for modem with IP: " << mIP << std::endl;
        return 1;
    }


void connection::sendSimple(std::string message)
    {
        std::cout << "Encoding and transmitting the message: " << message << " with janus..." << std::endl;

        /*Rekne ut samples*/
        int samples;

         /*setup for janus pakker*/
        std::string command = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver wav --stream-driver-args JanusRekneSampCou.wav --stream-fs 250000 --stream-format S16 --verbose 1 --packet-cargo \""+message+"\" )";
        FILE* terminal = popen(command.c_str(), "r");//open terminal
        pclose(terminal);

        std::cout << "Wait for 2 sec" << std::endl;
        std::this_thread::sleep_for(2000ms); // intervall mellom sendiger 

        /*open .wav file*/
        std::ifstream file (JPATH +"JanusRekneSampCou.wav");

        /*Skip bits if needed? (Skipper disse pga det er ein del av wav filen og blir ikkje den del av det vi sender?)*/
        file.seekg(44); 

        /*read number of samples for 16 bit per samples*/
        std::int16_t sample;
        std::size_t count = 0; 
        while (file.read(reinterpret_cast<char*>(&sample), sizeof(sample)))
        {
            ++count;
        }

        std::cout << "Samples: " << count << " with janus..." << std::endl;

        float cargo_samples = static_cast<float>(count) - 145310;//should add so it seands empty cargo (145310)
        float time_to_send_cargo = cargo_samples/250000.0; //need some fixing (streamdrime -fs)
        
        //--------------write to file-----------------  
        std::ofstream MyFilew323 ("TX_logg_testdag.txt", std::ios::app);
        MyFilew323 << "Samples:          " <<count<< " CArgo time: "<< std::to_string(time_to_send_cargo)<<"\n";
        MyFilew323.close(); 
        //-------------------------------------------
        //count = count*10; 
        //std::cout << "Samples:   " << count << " with janus..." << std::endl;

        /*make pipe*/
        int filedes[2];
        if(pipe(filedes) == -1) //Creating a pipe
        {
            perror("pipe");
            exit(1);
        }

        /*make fork for  */
        pid_t sdm_pid = fork(); //Creating a subprocess
        if(sdm_pid == -1) //Error handler
        {
            perror("fork");
            exit(1);
        }
//---------------------------------------------------------------------------------------------------------
//S_child:    Gen: X-S                                                                                                                   
//--------------------------------------------------------------------------------------------------------- 
        else if(sdm_pid == 0) //sdmsh child proc
        {
            while((dup2(filedes[1], STDOUT_FILENO) == -1) && (errno == EINTR)) {
            }
            close(filedes[1]);
            close(filedes[0]);

            char arg1[] = "sh";
            char arg2[] = "-c";
            std::string scmd = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;tx " + std::to_string(count) + " tcp:listen:127.0.0.1:" + std::to_string(TX_PORT) + "' )";
            char* sdm_arg[] = {arg1, arg2, (char*)scmd.c_str(), NULL};
            //FILE* sterm = popen(scmd.c_str(), "r");
            //pclose(sterm);
            execvp(sdm_arg[0], sdm_arg);
            perror("execvp");

            _exit(1);

        } 
//---------------------------------------------------------------------------------------------------------
//TX_Parent:       Gen: X                                                                                                                  
//--------------------------------------------------------------------------------------------------------- 
        else //Parent proc
        {
            std::this_thread::sleep_for(500ms); // WM: Delay needed for sdmsh to establish tcp listener

            /*make frok for janus*/
            pid_t jns_pid = fork();
            if(jns_pid == -1) //Error handler
            {
                perror("fork");
                exit(1);
            }
//---------------------------------------------------------------------------------------------------------
//j_child:   Gen: X-j                                                                                                                 
//---------------------------------------------------------------------------------------------------------
            else if(jns_pid == 0) //janus child proc
            {
                //--setup for janus pakker--
                char arg1[] = "sh";
                char arg2[] = "-c";
                std::string jcmd = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args connect:127.0.0.1:"+std::to_string(TX_PORT)+" --stream-fs 250000 --stream-format S16 --packet-reserv-time "+std::to_string(time_to_send_cargo)+" --verbose 9 --packet-cargo \""+message+"\" )";
                //std::string jcmd = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args connect:127.0.0.1:"+std::to_string(TX_PORT)+" --stream-fs 250000  --verbose 9 --packet-cargo \""+message+"\" )";
                //std::string jcmd = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver wav --stream-driver-args ../data/JanusRekneSampCou.wav --stream-fs 96000 --stream-format S16 --verbose 1 --packet-cargo \""+message+"\" )";
                
                char* jns_arg[] = {arg1, arg2, (char*)jcmd.c_str(), NULL};
                execvp(jns_arg[0], jns_arg);
                perror("execvp");
                _exit(1);
            }
//---------------------------------------------------------------------------------------------------------
//Parent:   Gen: X                                                                                                                
//---------------------------------------------------------------------------------------------------------
            else //Parent proc
            {   
                close(filedes[1]);          // WM flytta close fram 
                char buffer[1024] = {0};    // WM tøme bøffer, mulig vi må gjøre dette lenge før?
                int nbytes; 
                struct pollfd pfd; 
                int timeout = 5000;         // 5 sek 

                while(true)//------------------------------------->
                {

                    /*Vente til janus er ferdig*/
                    std::cout << waitpid(jns_pid, 0, WNOHANG);
                    if(waitpid(jns_pid, 0, WNOHANG) == jns_pid)
                    {
                        std::cout << "Janus done, quitting\n";

                        wait(0);
                        wait(0);

                        break;
                    }

                    /*lese fra pipe, vist pipe tom -> vent */
                    pfd.fd = filedes[0];
                    pfd.events = POLLIN;
                    int gatekeeper = poll(&pfd, 1, timeout);
                    if (gatekeeper == -1) 
                    {
                        perror("poll");
                    }
                    else if (gatekeeper== 0) 
                    {
                        printf("timeout, going to 5.3\n");
                    } 
                    else 
                    {
                        ssize_t count = read(filedes[0], buffer, sizeof(buffer));
                        printf("Fikk lest fra pipe()en\n");
                
                        if(count == -1)
                        {
                            if(errno == EINTR)
                            {
                                continue;
                            }
                            else
                            {
                                perror("read");
                                exit(1);
                            }
                        }
                        else if(count == 0)
                        {
                            std::cout << "EOF reached\n";
                            wait(0);
                            wait(0);
                            break;
                        }
                        else
                        {

                            // WM: dette er for å debuge koden, vil printe tid. 
                            std::ofstream MyFile333 ("TX_Terminal.txt", std::ios::app);
                            MyFile333 << "\n" << buffer << "\n";               
                            MyFile333.close(); 

                            std::string dstr = buffer;

                            int j = dstr.find("TX_STOP after");
                            std::cout << "SDMSH(" << std::to_string(j) << "): " << dstr << std::endl;
                            if(j != std::string::npos)
                            {
                                std::cout << "exiting\n";
                                close(filedes[0]);
                                wait(0);
                                wait(0);
                                break;

                            }
                        }
                    }
                }//------------------------------------->              
            }
        }
    }

//---------------------------------------------------------------------------------------------------------
//TX_Parent Start:      Gen: X
//---------------------------------------------------------------------------------------------------------
   /* void connection::sendSimple(std::string message)
    {
        std::cout << "Encoding and transmitting the message: " << message << " with janus..." << std::endl;

        ///Rekne ut samples
        int samples;
        if(message == "") //NB! Samples are hardcoded for sampling rate of 96kHz
        {
            samples = 55800;
        }
        else
        {
            samples = (message.length() * 4800 + 60600); 
        }
        int filedes[2];
        if(pipe(filedes) == -1) //Creating a pipe
        {
            perror("pipe");
            exit(1);
        }

        //make fork for  
        pid_t sdm_pid = fork(); //Creating a subprocess
        if(sdm_pid == -1) //Error handler
        {
            perror("fork");
            exit(1);
        }
//---------------------------------------------------------------------------------------------------------
//S_child:    Gen: X-S                                                                                                                   
//--------------------------------------------------------------------------------------------------------- 
        else if(sdm_pid == 0) //sdmsh child proc
        {
            while((dup2(filedes[1], STDOUT_FILENO) == -1) && (errno == EINTR)) {
            }
            close(filedes[1]);
            close(filedes[0]);

            char arg1[] = "sh";
            char arg2[] = "-c";
            std::string scmd = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'tx " + std::to_string(samples) + " tcp:listen:127.0.0.1:" + std::to_string(TX_PORT) + "' )";
            char* sdm_arg[] = {arg1, arg2, (char*)scmd.c_str(), NULL};
            //FILE* sterm = popen(scmd.c_str(), "r");
            //pclose(sterm);
            execvp(sdm_arg[0], sdm_arg);
            perror("execvp");

            _exit(1);

        } 
//---------------------------------------------------------------------------------------------------------
//TX_Parent:       Gen: X                                                                                                                  
//--------------------------------------------------------------------------------------------------------- 
        else //Parent proc
        {
            std::this_thread::sleep_for(500ms); // WM: Delay needed for sdmsh to establish tcp listener

            //make frok for janus
            pid_t jns_pid = fork();
            if(jns_pid == -1) //Error handler
            {
                perror("fork");
                exit(1);
            }
//---------------------------------------------------------------------------------------------------------
//j_child:   Gen: X-j                                                                                                                 
//---------------------------------------------------------------------------------------------------------
            else if(jns_pid == 0) //janus child proc
            {
                //setup for janus pakker
                char arg1[] = "sh";
                char arg2[] = "-c";
                //std::string jcmd = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args connect:127.0.0.1:"+std::to_string(TX_PORT)+" --stream-fs 250000 --stream-format S16 --verbose 9 --packet-cargo \""+message+"\" )";
                //std::string jcmd = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args connect:127.0.0.1:"+std::to_string(TX_PORT)+" --stream-fs 250000  --verbose 9 --packet-cargo \""+message+"\" )";
                std::string jcmd = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver wav --stream-driver-args ../data/JanusRekneSampCou.wav --stream-fs 96000 --stream-format S16 --verbose 1 --packet-cargo \""+message+"\" )";
                
                char* jns_arg[] = {arg1, arg2, (char*)jcmd.c_str(), NULL};
                execvp(jns_arg[0], jns_arg);
                perror("execvp");
                _exit(1);
            }
//---------------------------------------------------------------------------------------------------------
//Parent:   Gen: X                                                                                                                
//---------------------------------------------------------------------------------------------------------
            else //Parent proc
            {   
                close(filedes[1]);          // WM flytta close fram 
                char buffer[1024] = {0};    // WM tøme bøffer, mulig vi må gjøre dette lenge før?
                int nbytes; 
                struct pollfd pfd; 
                int timeout = 5000;         // 5 sek 

                while(true)//------------------------------------->
                {

                    //Vente til janus er ferdig
                    std::cout << waitpid(jns_pid, 0, WNOHANG);
                    if(waitpid(jns_pid, 0, WNOHANG) == jns_pid)
                    {
                        std::cout << "Janus done, quitting\n";

                        wait(0);
                        wait(0);

                        break;
                    }

                    //lese fra pipe, vist pipe tom -> vent 
                    pfd.fd = filedes[0];
                    pfd.events = POLLIN;
                    int gatekeeper = poll(&pfd, 1, timeout);
                    if (gatekeeper == -1) 
                    {
                        perror("poll");
                    }
                    else if (gatekeeper== 0) 
                    {
                        printf("timeout, going to 5.3\n");
                    } 
                    else 
                    {
                        ssize_t count = read(filedes[0], buffer, sizeof(buffer));
                        printf("Fikk lest fra pipe()en\n");



                
                        if(count == -1)
                        {
                            if(errno == EINTR)
                            {
                                continue;
                            }
                            else
                            {
                                perror("read");
                                exit(1);
                            }
                        }
                        else if(count == 0)
                        {
                            std::cout << "EOF reached\n";
                            wait(0);
                            wait(0);
                            break;
                        }
                        else
                        {

                            // WM: dette er for å debuge koden, vil printe tid. 
                            std::ofstream MyFile333 ("Buffer.txt", std::ios::app);
                            MyFile333 << "\n" << buffer << "\n";               
                            MyFile333.close(); 







                            std::string dstr = buffer;


                            // WM: dette er for å debuge koden, vil printe tid. 
                            std::ofstream MyFile3323 ("string.txt", std::ios::app);
                            MyFile3323 << "\n" << dstr << "\n";               
                            MyFile3323.close(); 


                            int j = dstr.find("TX_STOP after");
                            std::cout << "SDMSH(" << std::to_string(j) << "): " << dstr << std::endl;
                            if(j != std::string::npos)
                            {
                                std::cout << "exiting\n";
                                close(filedes[0]);
                                wait(0);
                                wait(0);
                                break;

                            }
                        }
                    }
                }//------------------------------------->              
            }
        }
    }
        */
    int connection::decode(int16_t buf[], std::string &message)
    {
        std::cerr << LIBNAME << ": Decode does nothing as it is unfinished" << std::endl;
        /*
        std::cout << "Decoding janus message" << std::endl;
        std::string command = "(cd " + JANUSPATH + " && ./janus-rx --config-file ../etc/" + JANUS_RX_CONFIG + " --verbose 1 --stream-driver-args listen:127.0.0.1:" + std::to_string(JANUS_RX_PORT) + " 2>&1)";
        std::FILE *JanusRX = popen(command.c_str(), "r");
        if(!JanusRX)
        {
            std::cout << "Could not open ./janus-rx" << std::endl;
            return 0;
        }
        int sock = 0, valread;
        struct sockaddr_in serv_addr;
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            std::cout << "Socket creation error" << std::endl;
            return 0;
        }
        
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(JANUS_RX_PORT);

        if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
        {
            std::cout << "Invalid address/ Address not supported" << std::endl;
            return 0;
        }

        if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
        {
            std::cout << "Connection failed" << std::endl;
            return 0;
        }
        send(sock, buf, sizeof(buf), 0);

        std::array<char, 128> rcvbuf;

        while(fgets(rcvbuf.data(), 128, JanusRX) != NULL)
        {
            std::cout << "Reading..." << std::endl;
            message += rcvbuf.data();
        }
        auto returnCode = pclose(JanusRX);
        std::cout << "The janus message was:" << std::endl << message << std::endl;
        return 1;
        */
        return 1;
    }


/* Comment:
.....
*/
    int connection::listenSimple(std::string &message)
    {


    }


int connection::beginListen(){

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
    
    pid_t dummy_pid = fork();                 //fork(): 
    if(dummy_pid == -1){                       //fork(): Error handler 
        perror("fork");            
        exit(EXIT_FAILURE);                         // EXIT_FAILURE = konstant 
    }

    else if (dummy_pid == 0) // Gen: X-X                                                  
    {
        /* Creating a fork "Coordinator_child" */
        pid_t janus_pid= fork();                 //fork(): Creating "
        if(janus_pid == -1)                       //fork(): Error handler 
        {
            perror("fork");
            exit(EXIT_FAILURE);                         // EXIT_FAILURE = konstant 
        }
        else if(janus_pid == 0){ // gen x-x-j
            close(fdERR[0]);                                                          //pipe(): close read         
            while((dup2(fdERR[1], STDERR_FILENO) == -1) && (errno == EINTR)) {}       //pipe(): Redirecting stderr to pipe (erroroutput now goes to pipe)                 
            close(fdERR[1]);                                                          //pipe(): close write
            
            /*Make string for ./janus TCP setup*/
            char arg1[] = "sh";                                                      
            char arg2[] = "-c";
            std::string jcmd = "(cd " + JPATH + " && ./janus-rx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args listen:127.0.0.1:" + std::to_string(RX_PORT)+" --stream-fs 250000 --verbose 1)";
            // std::string jcmd = "(cd " + JPATH + " && ./janus-rx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver raw --stream-driver-args janusmessageV2.raw --stream-fs 250000 --stream-format S16 --verbose 1 )";
            //std::string jcmd = "(cd " + JPATH + " && ./janus-rx  --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver raw --stream-driver-args ../data/janusMessage.raw --stream-fs 250000 --stream-format S16 --verbose 1 2>&1)";
            char* jns_arg[] = {arg1, arg2, (char*)jcmd.c_str(), NULL}; 

            /*Create new prosess image*/
            execvp(jns_arg[0], jns_arg);                                            //im_1  : Replace process image, (Running a new prosess), "fork-exec"
                                                                                    //im_2  : Will be a new prosess image, own memory (kinda like fork), code under will not run!!. 
                                                                                    //im_1  : If an error, no new image will be created, code under will run. 
            perror("ERROR -> jns_child_pid -> execvp");                             //im_1  : Print error to "stderr" 
            exit(EXIT_FAILURE);
            
        }
        else{ //gen x-x
            
            pid_t sdmsh_pid= fork();                 //fork(): Creating "
            if(sdmsh_pid == -1) {                      //fork(): Error handler {
                perror("fork");
                exit(EXIT_FAILURE);                         // EXIT_FAILURE = konstant 
            }
            else if (sdmsh_pid == 0){// gen x-x-s

                close(fdERR[0]);                                                          //pipe(): close read         
                close(fdERR[1]);                                                          //pipe(): close write
                
                
                std::this_thread::sleep_for(500ms);                         //Delay: for ./janus to establish tcp listener 

                /*Make string for ./janus TCP connect*/
                char arg1[] = "sh";                                         
                char arg2[] = "-c";                             
                std::string scmd = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;rx 0 tcp:connect:127.0.0.1:" + std::to_string(RX_PORT) + "' 2>&1 >> rxlog.txt)";
                char* sdm_arg[] = {arg1, arg2, (char*)scmd.c_str(), NULL};

                /*Create new prosess image*/
                execvp(sdm_arg[0], sdm_arg);                                //im_1  : Replace process image, (Running a new prosess), "fork-exec"
                                                                            //im_2  : Will be a new prosess image, own memory (kinda like fork), code under will not run!!. 
                                                                            //im_1  : If an error, no new image will be created, code under will run. 
                perror("ERROR -> sdm_child_pid -> execvp");                 //im_1  : Print error to "stderr"  (OPS! Denne feilmeldinger vil vi ikkje få!!)
                exit(EXIT_FAILURE);

            }
            else{ // gen x-x
                exit(0);
            }
        }  
    }

    else{//gen x


        wait(0);
        printf("Inside parent");
        close(fdERR[1]);
        return fdERR[0];
   
    }
}

/* Comment:
This code was developed by BSc students and has the potential to be used by future students. Therefore,
it is well-commented (might be to much, but trust us, it's worth it!). We recommend reading BSc 
theses for additional context and insights.

Recommended material: multiprocessing, inter-process communication (IPC), the Sleeping Barber Problem,
the Producer-Consumer Problem, fork(): "parent(), child, process ID, exit(), and wait()", pipe(), 
exec(), "Difference between fork() and exec()". 

This code is build for establish a TCP connection to an Evologic modem. It uses the "shmsh" library to connect
to the modem and the "Janus" library to decode the data. 

- Added "fdOUT" pipe to view standard output from Rx_Janus_Child process. Can be commented out.
- Rebuild alot of the code
- Implemented "Double Fork" technique for creating daemon processes. Recommended to look up this technique.
- Added comments throughout code for clarity.
*/

//---------------------------------------------------------------------------------------------------------
//Rx_Parent Start:      Gen: X
//---------------------------------------------------------------------------------------------------------
    int connection::listen(std::string &message, std::chrono::duration<double> timeout)
    {
        /* Creating pipe 1*/
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

        /* Creating pipe 2*/
        int fdOUT[2];                                      //pipe(): In The Future
        if(pipe(fdOUT) == -1)                              //pipe(): Error handler 
        {
            perror("pipe");
            exit(EXIT_FAILURE);
        }
        if (fcntl(fdOUT[0], F_SETFL, O_NONBLOCK) < 0)      //pipe(): Making non-blocking to prevent code lock
        {
            exit(2);
        }

        /* Creating pipe 3*/
        int fdERRsdmsh[2];                                      //pipe(): In The Future
        if(pipe(fdERRsdmsh) == -1)                              //pipe(): Error handler 
        {
            perror("pipe");
            exit(EXIT_FAILURE);
        }
        if (fcntl(fdERRsdmsh[0], F_SETFL, O_NONBLOCK) < 0)      //pipe(): Making non-blocking to prevent code lock
        {
            exit(2);
        }
 
        /* Creating a fork "dummy_child_pid" */
        pid_t dummy_child_pid = fork();                 //fork(): Creating "Rx Janus Child"
        if(dummy_child_pid == -1)                       //fork(): Error handler 
        {
            perror("fork");
            exit(EXIT_FAILURE);                         // EXIT_FAILURE = konstant 
        }
//---------------------------------------------------------------------------------------------------------
//Dummy_child:          Gen: X-X                    "Double fork" to avoid zombie process aka "Born to die"                                                                                                    
//---------------------------------------------------------------------------------------------------------  
        else if (dummy_child_pid == 0) // Gen: X-X                                                  
        {
            /* Creating a fork "Coordinator_child" */
            pid_t Coordinator_child = fork();                 //fork(): Creating "Rx Janus Child"
            if(Coordinator_child == -1)                       //fork(): Error handler 
            {
                perror("fork");
                exit(EXIT_FAILURE);                         // EXIT_FAILURE = konstant 
            }
//---------------------------------------------------------------------------------------------------------
//Coordinator_child:          Gen: X-X-X                                                                                                                        
//---------------------------------------------------------------------------------------------------------        
            else if(Coordinator_child == 0) // Gen: X-X-X
            {

                //*******          Ny_kode_før_fork()_Janus_og_Sdmsh



                //*******

                /* Creating a fork "jns_child_pid" */
                pid_t jns_child_pid = fork();               //fork(): Creating "Rx Janus Child"
                if(jns_child_pid == -1)                     //fork(): Error handler 
                {
                    perror("fork");                         // Print error to "stderr"
                    exit(EXIT_FAILURE);                     // EXIT_FAILURE = konstant 
                }
//---------------------------------------------------------------------------------------------------------
//Rx_Janus_Child:       Gen: X-X-X-j
//---------------------------------------------------------------------------------------------------------
                else if (jns_child_pid == 0) // Gen: X-X-X-J                                                  
                {
                    close(fdERR[0]);                                                          //pipe(): close read         
                    close(fdOUT[0]);                                                          //pipe(): close read
                    while((dup2(fdERR[1], STDERR_FILENO) == -1) && (errno == EINTR)) {}       //pipe(): Redirecting stderr to pipe (erroroutput now goes to pipe)                 
                    while((dup2(fdOUT[1], STDOUT_FILENO ) == -1) && (errno == EINTR)) {}      //pipe(): Redirecting stdout to pipe (standaroutput now goes to pipe)
                    close(fdERR[1]);                                                          //pipe(): close write
                    close(fdOUT[1]);                                                          //pipe(): close write       

                    /*Make string for ./janus TCP setup*/
                    char arg1[] = "sh";                                                      
                    char arg2[] = "-c";
                    std::string jcmd = "(cd " + JPATH + " && ./janus-rx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args listen:127.0.0.1:" + std::to_string(RX_PORT)+" --stream-fs 250000 --verbose 1)";
                    // std::string jcmd = "(cd " + JPATH + " && ./janus-rx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver raw --stream-driver-args janusmessageV2.raw --stream-fs 250000 --stream-format S16 --verbose 1 )";
                    //std::string jcmd = "(cd " + JPATH + " && ./janus-rx  --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver raw --stream-driver-args ../data/janusMessage.raw --stream-fs 250000 --stream-format S16 --verbose 1 2>&1)";
                    char* jns_arg[] = {arg1, arg2, (char*)jcmd.c_str(), NULL}; 

                    /*Create new prosess image*/
                    execvp(jns_arg[0], jns_arg);                                            //im_1  : Replace process image, (Running a new prosess), "fork-exec"
                                                                                            //im_2  : Will be a new prosess image, own memory (kinda like fork), code under will not run!!. 
                                                                                            //im_1  : If an error, no new image will be created, code under will run. 
                    perror("ERROR -> jns_child_pid -> execvp");                             //im_1  : Print error to "stderr" 
                    exit(EXIT_FAILURE);
                }
//---------------------------------------------------------------------------------------------------------
//Coordinator_child:    Gen: X-X-X                                                                                                                    
//---------------------------------------------------------------------------------------------------------   
                else // Gen: X-X-X
                {  
                    close(fdERR[0]);                                                          //pipe(): close read         
                    close(fdOUT[0]); 
                    close(fdERR[1]);                                                          //pipe(): close write
                    close(fdOUT[1]);                                                          //pipe(): close write       
    
                    /* Creating a fork "sdm_child_pid" */
                    pid_t sdm_child_pid = fork();           //fork(): Creating "Rx Janus Child"
                    if(sdm_child_pid == -1)                 //fork(): Error handler  
                    {
                        perror("fork");                     // Print error to "stderr"
                        exit(EXIT_FAILURE);                 // EXIT_FAILURE = konstant
                    }
//---------------------------------------------------------------------------------------------------------
//Rx_Sdmsh_Child:       Gen: X-X-X-S
//---------------------------------------------------------------------------------------------------------
                    else if (sdm_child_pid == 0) // Gen: X-X-X-S
                    {   

                        close(fdERRsdmsh[0]);                                                          //pipe(): close read         
                        //while((dup2(fdERRsdmsh[1], STDERR_FILENO) == -1) && (errno == EINTR)) {}       //pipe(): Redirecting stderr to pipe (erroroutput now goes to pipe)                 
                        close(fdERRsdmsh[1]);                                                          //pipe(): close write    

                        std::this_thread::sleep_for(500ms);                         //Delay: for ./janus to establish tcp listener 

                        /*Make string for ./janus TCP connect*/
                        char arg1[] = "sh";                                         
                        char arg2[] = "-c";                             
                        std::string scmd = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;rx 0 tcp:connect:127.0.0.1:" + std::to_string(RX_PORT) + "' 2>&1 >> rxlog.txt)";
                        char* sdm_arg[] = {arg1, arg2, (char*)scmd.c_str(), NULL};

                        /*Create new prosess image*/
                        execvp(sdm_arg[0], sdm_arg);                                //im_1  : Replace process image, (Running a new prosess), "fork-exec"
                                                                                    //im_2  : Will be a new prosess image, own memory (kinda like fork), code under will not run!!. 
                                                                                    //im_1  : If an error, no new image will be created, code under will run. 
                        perror("ERROR -> sdm_child_pid -> execvp");                 //im_1  : Print error to "stderr"  (OPS! Denne feilmeldinger vil vi ikkje få!!)
                        exit(0);
                    }
//---------------------------------------------------------------------------------------------------------
//Dummy_child:          Gen: X-X-X                                                                                                                    
//---------------------------------------------------------------------------------------------------------        
                    else // Gen: X-X-X          
                    {

                        //*******          Ny_kode_etter_fork()_Janus_og_Sdmsh
 

                        //*******
                    }
                    // Gen: X-X-X     

                    //*******          Ny_kode_etter_fork()_Janus_og_Sdmsh
                    //sleep(3);
                    wait(0);
                    wait(0);
                    //pls close tcp;
                    close(0);
                    //kill(sdm_child_pid, SIGTERM);
                    //kill(jns_child_pid, SIGTERM);
                    exit(0);

                    //*******     // kan prøve etterpå "SIGINT" samme som control-C  , "SIGTERM "
                }
//---------------------------------------------------------------------------------------------------------
//Dummy_child_1:          Gen: X-X                                                                                                                    
//---------------------------------------------------------------------------------------------------------   

            } else 
            /* Dummy_child end his life */
            exit(0);
        }
//---------------------------------------------------------------------------------------------------------
//Rx_Parent:            Gen: X
//---------------------------------------------------------------------------------------------------------
        else  // Gen: X
        {
            /* Final step: "Double Fork"-technique, (terminat child, grandchild become orphan)*/
            wait(0);                                            // Parent wait for Dummy_child to die. 

            std::cout << "\nProject Orphan's done! Free from responsibilities! YEAH!! \n"; 
            /* Parant is now free, he does not have any children  */
            /* Orphan prosess gets new parent, "Init", with pid=1 */
            /* "Init" designed to terminat orphan in a good way   */

            std::this_thread::sleep_for(500ms);    
            
            char chERR[1024] = {0};                                           //Buffer for storing from pipe() ""
            char chOUT[1024] = {0};                                           //Buffer for storing from pipe() "fdOUT"
            char chERRsdsmh[1024] = {0};                                           //Buffer for storing from pipe() ""

            std::string janusframeERR;                            //Will build janus massage in this string, from strr
            std::string janusframeOUT;                            //Will build janus massage in this string, from strr
            std::string janusframeSdsmh;                            //Will build janus massage in this string, from strr

            std::chrono::steady_clock::time_point start, now;   //Time: For exiting after timeout
            start = std::chrono::steady_clock::now();           //Time: Start

            std::string pid_str; 

            while(true) //--------------------->  
            {
                /* 1. Read from pipe "ERR" */ 
                //count = read(filedes[0], buffer, sizeof(buffer));
                
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
                        printf("\n Read from pipe failed:  \n");
                        perror("read");
                        exit(4);
                    }
                default:                                            //Read(): OK
                    janusframeERR += chERR;                         //Appending "ch1" to "janusframe1"

                    // WM: dette er for å debuge koden, vil printe tid. 
                    std::ofstream MyFile11 ("ERR.txt", std::ios::app);
                    MyFile11 << chERR ;               
                    MyFile11.close(); 
                    break;
                }

               /* 2. Read from pipe "OUT" */ 
                int rdstat2 = read(fdOUT[0], &chOUT, sizeof(char)); //Read(): From pipe(), write to "ch1", one character at a time.  
                switch (rdstat2)
                {
                case -1:                    
                    if(errno == EAGAIN)                         //Read(): Error handler, if empty 
                    {
                        break;
                    }
                    else                                        //Read(): Error handler, if error 
                    {
                        printf("\n Read from pipe failed:  \n");
                        perror("read");
                        exit(4);
                    }
                default:                                        //Read(): OK
                    janusframeOUT += chOUT;                     //Appending "ch1" to "janusframe1"

                    // WM: dette er for å debuge koden, vil printe tid. 
                    std::ofstream MyFile121 ("JanusOUT.txt", std::ios::app);
                    MyFile121 << "\n" << janusframeOUT << "\n";               
                    MyFile121.close(); 

                    break;
                }

               /* 3. Read from pipe "ERR" to sdmsh*/ 
                int rdstat3 = read(fdERRsdmsh[0], &chERRsdsmh, sizeof(char)); //Read(): From pipe(), write to "ch1", one character at a time.  
                switch (rdstat3)
                {
                case -1:                    
                    if(errno == EAGAIN)                         //Read(): Error handler, if empty 
                    {
                        break;
                    }
                    else                                        //Read(): Error handler, if error 
                    {
                        printf("\n Read from pipe failed:  \n");
                        perror("read");
                        exit(4);
                    }
                default:                                        //Read(): OK
                    janusframeSdsmh += chERRsdsmh;                     //Appending "ch1" to "janusframe1"

                    // WM: dette er for å debuge koden, vil printe tid. 
                    std::ofstream MyFile333 ("OUTsdmsh.txt", std::ios::app);
                    MyFile333 << "\n" << janusframeSdsmh << "\n";               
                    MyFile333.close(); 

                    break;
                }

                /* get pid of ./janus*/
                std::string jframe = janusframeERR; 
                // Find the position of the colon separator
                size_t colon_pos = jframe.find(":PIDKLARJANUS");
                if (colon_pos == std::string::npos) {
                    //printf("\n lol dont work yo \n"); 
                }
                else 
                {
                    // Extract the number substring before the colon separator
                    pid_str = jframe.substr(0, colon_pos);

                    // Convert the number substring to a pid_t value
                    
                    //kill(pid_CCJ, SIGINT);
                    //kill(pid_CCJ, SIGQUIT);
                    //kill(pid_CCJ, SIGHUP);

                    //printf("\n lol pid janus string is here:  %d \n", pid_str );                     
                } 


                /* Look For A Massage In String: "janusframe1"*/ 
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

                    pid_t pid_CCJ1 = std::atoi(pid_str.c_str());
                    printf("\n killing janus PID:%d:\n", pid_CCJ1 );

                    close(fdERR[0]);                              //pipe(): Close read
                    close(fdOUT[0]);  /*                            //pipe(): Close read 
                    kill(pid_CCJ1, SIGINT);
                    kill(pid_CCJ1, SIGQUIT);
                    kill(pid_CCJ1, SIGHUP);
                    kill(pid_CCJ1, SIGTERM);*/
/*
                    //----------------------------------------------------------------------
                    std::string sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;ref preamble.raw')";
                    FILE* terminal = popen(sdmcommand.c_str(), "r");
                    pclose(terminal);
                    std::cout << "1 Preamble set for modem with IP: " << mIP << std::endl;

                    std::this_thread::sleep_for(1000ms); 

                    std::string sdmcommand2 = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;ref preamble.raw')";
                    FILE* terminal2 = popen(sdmcommand.c_str(), "r");
                    pclose(terminal2);
                    std::cout << "2 Preamble set for modem with IP: " << mIP << std::endl;
                    //----------------------------------------------------------------------
*/
                    std::ofstream MyFilem ("SVAR.txt", std::ios::app);
                    MyFilem << "RX: exit pga timeout"<< "\n";                    
                    MyFilem.close(); 
                    //return 1;

                    return 0;                                   //Exit "SendSimple"
                }
            }//---------------------> 

            close(fdERR[0]);                              //pipe(): Close read
            close(fdOUT[0]);                              //pipe(): Close read 

            pid_t pid_CCJ2 = std::atoi(pid_str.c_str());
            printf("\n killing janus PID:%d:\n", pid_CCJ2 );
/*
            kill(pid_CCJ2, SIGINT);
            kill(pid_CCJ2, SIGQUIT);
            kill(pid_CCJ2, SIGHUP);
            kill(pid_CCJ2, SIGTERM);*/

/*----------------------------------------------------------------------
            std::string sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;ref preamble.raw')";
            FILE* terminal = popen(sdmcommand.c_str(), "r");
            pclose(terminal);
            std::cout << "1 Preamble set for modem with IP: " << mIP << std::endl;

            std::this_thread::sleep_for(1000ms); 

            std::string sdmcommand2 = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;ref preamble.raw')";
            FILE* terminal2 = popen(sdmcommand2.c_str(), "r");
            pclose(terminal2);
            std::cout << "2 Preamble set for modem with IP: " << mIP << std::endl;
*/

            std::ofstream MyFilem ("SVAR.txt", std::ios::app);
            MyFilem << "RX: exit pga message found" << "\n";                    
            MyFilem.close(); 

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
                message = janusframeERR.substr(spos, epos-spos);
                std::cout << "Cargo is: " << message << std::endl;


                // WM: dette er for å debuge koden, vil printe tid. 
                std::ofstream MyFile121 ("SVAR.txt", std::ios::app);
                MyFile121 << "\n" << "Rx: svar: " << message << "\n";               
                MyFile121.close(); 
                return 1;
            }
            else
            {
                std::cerr << "Error cargo not found, this should not happen..." << std::endl;
                exit(EXIT_FAILURE);
                return 1;
            }
            return 1;
        }
        return 1;
    }
//---------------------------------------------------------------------------------------------------------

    int connection::printheader()
    {
        for(int i = 0; i <= 7; i++)
        {
            std::cout << "HEEEEEEEEEEEEEEEEEEEEElp: " <<std::hex << unsigned(SDM_FRAMEHEADER[i]);
            std::cout << " ";
        }
        std::cout << std::endl;
        return 1;
    }
    int connection::sdmStop()
    {
        return 0;
    }

    /* Comment:
    .....
    */

    void connection::findNumberOfSamples(std::string message)
    {
        //-----------------------------for debug---------------------------------------
        /*Rekne ut samples*/
        /*
        int samplesOld;
        if(message == "") //NB! Samples are hardcoded for sampling rate of 96kHz
        {
            samplesOld = 55800;
        }
        else
        {
            samplesOld = (message.length() * 4800 + 60600); 
        }
        */
        //-----------------------------------------------------------------------------

        /*setup for janus pakker*/
        std::string command = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver wav --stream-driver-args JanusRekneSampCou.wav --stream-fs 250000 --stream-format S16 --verbose 1 --packet-cargo \""+message+"\" )";
        FILE* terminal = popen(command.c_str(), "r");
        pclose(terminal);

        std::cout << "Okay so I just made the .wav file but I need some time to meditate" << std::endl;
        std::this_thread::sleep_for(1000ms); // intervall mellom sendiger 
        std::cout << "Belive me meditate some to, its good for you bro" << std::endl;
        std::this_thread::sleep_for(1000ms); // intervall mellom sendiger 
        std::cout << "Nice right, anyhow here are your data" << std::endl;
        std::this_thread::sleep_for(1000ms); // intervall mellom sendiger 

        /*open .wav file*/
        std::ifstream file (JPATH +"JanusRekneSampCou.wav");

        /*Skip bits if needed? (Skipper disse pga det er ein del av wav filen og blir ikkje den del av det vi sender?)*/
        file.seekg(44); 

        /*read number of samples for 16 bit per samples*/
        std::int64_t sample;
        std::size_t count = 0; 
        while (file.read(reinterpret_cast<char*>(&sample), sizeof(sample)))
        {
            ++count;
        }

        /*print number of samples*/
        std::cout << "number of samples are: " << count << std::endl;


        // WM: dette er for å debuge koden, vil printe tid. 
        std::ofstream MyFile121 ("NumberOfSamples.txt", std::ios::app);
        MyFile121 << "\n" << "Message: " << message << "\n"; 
        MyFile121 <<         "N.O.S.:  " << count << "\n";  
        int mysamples = count;
        std::cout <<"HER KOMMER SAMPLES!: " <<mysamples<<std::endl;
        //gMyFile121 <<         "BScOld:  " << samplesOld << "\n";                 
        MyFile121.close(); 

        return ;
    }
}
