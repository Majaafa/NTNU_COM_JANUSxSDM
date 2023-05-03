//*************************************************************************
// This library is for underwater communication using EvoLogics modems.      
// It uses EvoLogics SDM library and janus protocol.
//*************************************************************************
//    *
//         *

//*************************************************************************
// Authors: Wilhelm Merkesvik and Maja Austin Fauske                      *
//*************************************************************************

#define LIBNAME "Evo_janusxsdm"

#include "Evo_janusXsdm.h"

#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>              
#include <sys/wait.h>             
#include <fcntl.h>          
#include <unistd.h>                
#include <sys/types.h>             
#include <stdio.h> 

#include <fstream>
#include <array>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <iterator>
#include <iostream>                
#include <cstring>                
#include <ctime>            
#include <cstdint>


#define SRV_PORT 9910 //?

using namespace std::chrono_literals; //?

namespace Evo_janusXsdm
{
    uint8_t connection::SDM_FRAMEHEADER[] = {0x80, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x00, 0x00}; //?

    connection::connection(std::string modemIP, std::string JANUSPATH, std::string SDMPATH, int rxPort, int txPort, float STREAM_fs)
    {
        mIP = modemIP;
        JPATH = JANUSPATH;
        SPATH = SDMPATH;
        RX_PORT = rxPort;
        TX_PORT = txPort;
        stream_fs = STREAM_fs;
    }

    int 
    connection::sdmConfigAir()
    {
        std::string sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;config 30 0 3 0')";
        FILE* terminal = popen(sdmcommand.c_str(), "r"); //writes to terminal
        pclose(terminal);
        std::cout << "Modem with IP: " << mIP << " configured." << std::endl;
        return 1;
    }

    std::string 
    connection::sdmConfig()
    {
        std::string input, sdmcommand, sfty = "n";
        std::cout << "Choose source level:\n 3 - -20dB (min)\n 2 - -12dB\n 1 - -6dB \n 0 - 0dB (max)\n Warning chosing a source level higher than -20dB is not recomended when testing in air as it might destroy the modems.\n";
        std::cin >> input;

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
        pclose(terminal);

        std::cout << "Modem with IP: " << mIP << " configured." << std::endl;
        std::string config = "30 0 "+ std::to_string(num) + " 0";

        return config;
    }

    int 
    connection::setPreamble()
    {
        std::string sdmcommand = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;ref preamble.raw')";
        FILE* terminal = popen(sdmcommand.c_str(), "r");
        pclose(terminal);
        std::cout << "Preamble set for modem with IP: " << mIP << std::endl;
        return 1;
    }

    float 
    connection::getNumberOfSamples(std::string message)
    { 
    //-- Variabler vi skal hive ut etterpå:                     (TODO: legge disse til i "connection")
        //float stream_fs = 250000.0; 

        int samples;
        //Creat a dummy janus packeage 
        std::string command = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver wav --stream-driver-args JanusRekneSampCou.wav --stream-fs " + std::to_string(stream_fs) +"  --stream-format S16 --verbose 1 --packet-cargo \""+message+"\" )";
        FILE* terminal = popen(command.c_str(), "r");
        pclose(terminal);

        // Delay for creating dummy janus packeage
        std::cout << "Wait for 2 sec" << std::endl;
        std::this_thread::sleep_for(2000ms); // This is not tested, prob 500ms is good

        //Open .wav file
        std::ifstream file (JPATH +"JanusRekneSampCou.wav");

        //Skip bits of header in Wav file 
        file.seekg(44); 

        //Read number of samples for 16 bit per samples
        std::int16_t sample;
        std::size_t count = 0; 
        while (file.read(reinterpret_cast<char*>(&sample), sizeof(sample)))
        {
            ++count;
        }
        return static_cast<float>(count);
    }

    float 
    connection::getPacketReservTime(float samplesCount) 
    {
        float samples_no_cargo = 145310;                            // Hardcode for --stream-fs 250000
        float samples_per_sec = 250000.0;                           // Hardcode for --stream-fs 250000  (could use settings for --stream-fs directly here)
        float samples_tot_pkg = static_cast<float>(samplesCount); 
        float samples_in_cargo = samples_tot_pkg - samples_no_cargo;       
        float time_reserv_cargo = samples_in_cargo/samples_tot_pkg;    
        return time_reserv_cargo; 
    }

    void 
    connection::startTX(std::string message)
    {
        std::cout << "Starting TX for message: " << message << std::endl;

    //-- Variabler vi skal hive ut etterpå:                     (TODO: legge disse til i "connection")
        float stream_fs = 250000.0; 

    //-- Calulate samples in Janus package (used in TCP) --  
        float samples = getNumberOfSamples(message);
    
    //-- Calulate reservation time for cargo in janus pack --       (TODO: Make a function for this!)
        float reserv_time = getPacketReservTime(samples); 

    //-- Creat pipe, used in janus stream -- 
        int filedes[2];
        if(pipe(filedes) == -1) //Creating a pipe
        {
            perror("pipe");
            exit(1);
        }

    //-- Creat fork for running sdmsh process --
        pid_t sdm_pid = fork(); 
        if(sdm_pid == -1)
        {
            perror("fork");
            exit(1);
        }

//---------------------------------------------------------------------------------------------------------
//S_child:    Gen: X-S                                                                                                              
//--------------------------------------------------------------------------------------------------------- 
        else if(sdm_pid == 0) // Gen: x-s
        {
        //-- Connect pipe to Standard Output Stream for sdmsh -- 
            close(filedes[0]);
            while((dup2(filedes[1], STDOUT_FILENO) == -1) && (errno == EINTR)) {}   
            close(filedes[1]);

        //-- Start a Sdmsh process (New process image -> open new sdmsh process) -- 
            char arg1[] = "sh";
            char arg2[] = "-c";
            std::string scmd = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;tx " + std::to_string(samples) + " tcp:listen:127.0.0.1:" + std::to_string(TX_PORT) + "' )";
            char* sdm_arg[] = {arg1, arg2, (char*)scmd.c_str(), NULL};
            execvp(sdm_arg[0], sdm_arg);
            perror("execvp");
            _exit(1);
        } 
//---------------------------------------------------------------------------------------------------------
//TX_Parent:       Gen: X                                                                                                                  
//--------------------------------------------------------------------------------------------------------- 
        else //Gen: X
        {
            std::this_thread::sleep_for(500ms); //Delay needed for sdmsh to establish tcp socket 
            close(filedes[1]);                  

        //-- Creat fork for running janus process -- 
            pid_t jns_pid = fork();
            if(jns_pid == -1) 
            {
                perror("fork");
                exit(1);
            }
//---------------------------------------------------------------------------------------------------------
//j_child:   Gen: X-j                                                                                                                 
//---------------------------------------------------------------------------------------------------------
            else if(jns_pid == 0) //Gen: x-j
            {
                close(filedes[0]); 

            //-- Start Janus process (New process image -> open new sdmsh process) -- 
                char arg1[] = "sh";
                char arg2[] = "-c";
                std::string jcmd = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args connect:127.0.0.1:"+std::to_string(TX_PORT)+" --stream-fs " + std::to_string(stream_fs) +" --stream-format S16 --packet-reserv-time "+std::to_string(reserv_time)+" --verbose 9 --packet-cargo \""+message+"\" )";
                char* jns_arg[] = {arg1, arg2, (char*)jcmd.c_str(), NULL};
                execvp(jns_arg[0], jns_arg);
                perror("execvp");
                _exit(1);
            }
//---------------------------------------------------------------------------------------------------------
//Parent:   Gen: X                                                                                                                
//---------------------------------------------------------------------------------------------------------
            else //Gen: x
            {                          
                char buffer[1024] = {0};
                int nbytes; 
                struct pollfd pfd; 
                int timeout = 5000;     // Time before exiting Poll,     (TODO:  can be sett as a optinal variabel. ) 

                while(true)//--->
                {
                //-- Wait for janus and sdmsh to exit --            (TODO: look at this more, might be one wait to much? or not?)
                    std::cout << waitpid(jns_pid, 0, WNOHANG);
                    if(waitpid(jns_pid, 0, WNOHANG) == jns_pid)
                    {
                        std::cout << "TX done, janus and sdmsh have closed \n";
                        wait(0);
                        wait(0);
                        close(filedes[0]);
                        break;
                    }

                //-- Read from pipe (Wait if pipe is empty, else read form pipe) --             (TODO: Not jet a use for this pipe, can read from error stream or somting in the future)
                    pfd.fd = filedes[0];
                    pfd.events = POLLIN;
                    int gatekeeper = poll(&pfd, 1, timeout);
                    if (gatekeeper == -1) 
                    {
                        perror("poll");
                    }
                    else if (gatekeeper== 0)            //TODO exit when timeout poll
                    {
                        std::cout << "Timeout in TX: \n should not happen. If this message appears, you might have a zombie or two. They will probably die in the next TX run\n"
                                    << "Tips: use 'ps -ef' commands and look for '[sh] <defunct>', these are the zombies \n"  << std::endl;
                        close(filedes[0]);
                        break;                          // WM added after new lib, not tested          
                    } 
                    else 
                    {
                        ssize_t count = read(filedes[0], buffer, sizeof(buffer));                
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
                        else if(count == 0)     // EOF reached 
                        {
                            wait(0);
                            wait(0);
                            close(filedes[0]);
                            break;
                        }
                        else
                        {
                            std::string dstr = buffer;
                            int j = dstr.find("TX_STOP after");
                            std::cout << "SDMSH(" << std::to_string(j) << "): " << dstr << std::endl;
                            if(j != std::string::npos)
                            {
                                std::cout << "exiting\n";
                                wait(0);
                                wait(0);
                                close(filedes[0]);
                                break;

                            }
                        }
                    }
                }//--->              
            }
        }
    }

    int 
    connection::startRX()//beginListen
    {
        
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
                std::string jcmd = "(cd " + JPATH + " && ./janus-rx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args listen:127.0.0.1:" + std::to_string(RX_PORT)+" --stream-fs 250000 --verbose 1 )";
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
            else
            { //gen x-x
                std::this_thread::sleep_for(500ms);                         //Delay: for ./janus to establish tcp listener 

                pid_t sdmsh_pid= fork();                 //fork(): Creating "
                if(sdmsh_pid == -1) 
                {                      //fork(): Error handler {
                    perror("fork");
                    exit(EXIT_FAILURE);                         // EXIT_FAILURE = konstant 
                }
                else if (sdmsh_pid == 0)
                {// gen x-x-s

                    close(fdERR[0]);                                                          //pipe(): close read         
                    close(fdERR[1]);                                                          //pipe(): close write
                    
                    
                    
                    /*Make string for ./janus TCP connect*/
                    char arg1[] = "sh";                                         
                    char arg2[] = "-c";                             
                    std::string scmd = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;rx 0 tcp:connect:127.0.0.1:" + std::to_string(RX_PORT) + "' )";
                    char* sdm_arg[] = {arg1, arg2, (char*)scmd.c_str(), NULL};

                    /*Create new prosess image*/
                    execvp(sdm_arg[0], sdm_arg);                                //im_1  : Replace process image, (Running a new prosess), "fork-exec"
                                                                                //im_2  : Will be a new prosess image, own memory (kinda like fork), code under will not run!!. 
                                                                                //im_1  : If an error, no new image will be created, code under will run. 
                    perror("ERROR -> sdm_child_pid -> execvp");                 //im_1  : Print error to "stderr"  (OPS! Denne feilmeldinger vil vi ikkje få!!)
                    exit(EXIT_FAILURE);

                }
                else
                { // gen x-x
                    exit(0);
                }
            }  
        }

        else
        {//gen x


            wait(0);
            //printf("Inside parent");
            close(fdERR[1]);

            return fdERR[0];
    
        }
        
    }

     std::array<std::string,6> 
     connection::listenOnceTheFoolproofRX(std::string &message)
    {
        
        int fd_pipe = startRX();
        std::array<std::string,6> fromRX = listenRX(fd_pipe,message);
        closePipeRX(fd_pipe);
        stopRX();

        return fromRX;
        
    }

    std::array<std::string,6> 
    connection::listenRX(int infoFromJanus, std::string &message)
    {
        std::string idStr = "Packet         :   Payload                                    : ";
        std::string idRT = "Packet         :     Reservation Time (7 bits)                : ";
        std::string idCRC = "Packet         :   CRC (8 bits)                               : ";
        std::string idCargoSize = "Packet         :   Cargo Size                                 : ";

        struct pollfd pfd; 
        std::string janus_frame;
        char janus_char[1024] = {};
        
        int timeout = 60000;

        std::this_thread::sleep_for(500ms); 
       
        //dummyFlushJanusTX();
          
        
        //

        while(true)
        {    
            //int timeout = 50000;         // 5 sek 
            std::array<std::string,6> myArray; 
            pfd.fd = infoFromJanus;
            pfd.events = POLLIN;
            int gatekeeper = poll(&pfd, 1, timeout);

            if (gatekeeper == -1) 
            {
                perror("poll");
                break;
            }
            else if (gatekeeper== 0) 
            {
                printf("timeout, going to 5.3\n");
                break;
                
            } 
            else 
            {
                int rdstate = read(infoFromJanus, &janus_char, sizeof(janus_char));
                switch (rdstate){
                    case -1:                    
                        if(errno == EAGAIN)                             //Read(): Error handler, if empty 
                        {
                            //?
                            break;
                        }
                        else                                            //Read(): Error handler, if error 
                        {
                            //printf("\n Read from pipe failed:  \n");
                            perror("read something wrong with pipe");
                            exit(4);
                            //break;//?
                        }
                    default:                                            //Read(): OK
                        janus_frame+= janus_char; 
                        std::ofstream MyFile333 ("janus_char.txt", std::ios::app);
                        MyFile333 <<janus_char;               
                        MyFile333.close();                
                                                //Appending "ch1" to "janusframe1"
                        janus_char[1024] ={0};
                        
                    
                        break;
                }
                
                std::ofstream MyFile333 ("TR_TerminalM.txt", std::ios::app);
                MyFile333 << "\n melding: " <<"size januse_char: " <<std::to_string(strlen(janus_char)) <<" size januse_frame: " <<std::to_string(janus_frame.length()) ;               
                MyFile333.close(); 
                

                /* Look For A Massage In String: "janusframe1"*/ 
        
                if(janus_frame.find("Packet         : Cargo (ASCII)                                :") != std::string::npos)
                {
                    //std::cout << "look at me 95"<< std::endl;

                    
                    
                    
                    message = findInJanus_frame(idStr,janus_frame);
                    std::string CRC, CargoSize, RT;

                    CRC = findInJanus_frame(idCRC,janus_frame);
                    CargoSize = findInJanus_frame(idCargoSize,janus_frame);
                    RT = findInJanus_frame(idRT,janus_frame);

                    //std::string CRC = findInJanus_frame("CRC",janus_frame);

                    //std::this_thread::sleep_for(500ms);  
                    //std::cout <<"My message: "<< CRC <<std::endl;
                     // WM: dette er for å debuge koden, vil printe tid. 
                   




                    myArray= {message,CRC,CargoSize,RT,std::to_string(strlen(janus_char)),std::to_string(janus_frame.length())}; 
                    if(message != "nei")                        // npos error handeling for .find(), if the sring is not found.                     
                    {
                    janus_frame = "";
                    //janus_char[1024] ={0};
                    
                    }
                
                
                return myArray;
                }
    
            }

        }
        
        return {"NaN","NaN","NaN","NaN"};
        

    }

    void 
    connection::stopRX()
    {
        std::string command = "(cd " + SPATH + " && ./sdmsh " + mIP + " -e 'stop;rx 0 tcp:connect:127.0.0.1:" + std::to_string(RX_PORT) + "')";
        FILE* terminal = popen(command.c_str(), "r");//open terminal
        pclose(terminal);
        //std::this_thread::sleep_for(500ms);                         //Delay: for ./janus to establish tcp listener 

    }

    void 
    connection::closePipeRX(int &pipe)
    {

        close(pipe);

    }

    std::string 
    connection::findInJanus_frame(std::string idStr,std::string janusframe)
    {

        //--Look For A Massage In String: "janusframe"
        std::string endStr = "\n";
        size_t spos, epos, pos; 

        pos = janusframe.find(idStr);

        if(pos != std::string::npos)                        // npos error handeling for .find(), if the sring is not found.                     
        {
            spos = pos + idStr.length();
            epos = janusframe.find(endStr, spos);
            std::string responseFrame = janusframe.substr(spos, epos-spos);
            
            
            return responseFrame;
        }
        else
        {
            return "Unable to find in frame";

        }
        
        

    }

     //void connection::findNumberOfSamples(std::string message)
    //{
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

        //*setup for janus pakker*/
       /* std::string command = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver wav --stream-driver-args JanusRekneSampCou.wav --stream-fs 250000 --stream-format S16 --verbose 1 --packet-cargo \""+message+"\" )";
        FILE* terminal = popen(command.c_str(), "r");
        pclose(terminal);

        std::cout << "Okay so I just made the .wav file but I need some time to meditate" << std::endl;
        std::this_thread::sleep_for(1000ms); // intervall mellom sendiger 
        std::cout << "Belive me meditate some to, its good for you bro" << std::endl;
        std::this_thread::sleep_for(1000ms); // intervall mellom sendiger 
        std::cout << "Nice right, anyhow here are your data" << std::endl;
        std::this_thread::sleep_for(1000ms); // intervall mellom sendiger 

        //open .wav file
        std::ifstream file (JPATH +"JanusRekneSampCou.wav");

        //Skip bits if needed? (Skipper disse pga det er ein del av wav filen og blir ikkje den del av det vi sender?)
        file.seekg(44); 

        //read number of samples for 16 bit per samples
        std::int64_t sample;
        std::size_t count = 0; 
        while (file.read(reinterpret_cast<char*>(&sample), sizeof(sample)))
        {
            ++count;
        }

        //print number of samples
        std::cout << "number of samples are: " << count << std::endl;


        // WM: dette er for å debuge koden, vil printe tid. 
        std::ofstream MyFile121 ("NumberOfSamples.txt", std::ios::app);
        MyFile121 << "\n" << "Message: " << message << "\n"; 
        MyFile121 <<         "N.O.S.:  " << count << "\n";  
        int mysamples = count;
        std::cout <<"HER KOMMER SAMPLES!: " <<mysamples<<std::endl;
        //gMyFile121 <<         "BScOld:  " << samplesOld << "\n";                 
        MyFile121.close(); 

        
    }*/

    void 
    connection::dummyFlushJanusTX(){

                 /*setup for janus pakker*/
        std::string command = "(cd " + JPATH + " && ./janus-tx --pset-file ../etc/parameter_sets.csv --pset-id 2 --stream-driver tcp --stream-driver-args connect:127.0.0.1:"+std::to_string(RX_PORT)+" --stream-fs 250000 --stream-format S16 --packet-reserv-time "+std::to_string(0.15031051)+" --verbose 9 --packet-cargo \""+"2"+"\" )";
        FILE* terminal = popen(command.c_str(), "r");//open terminal
        pclose(terminal);


    }

    void 
    connection::printNumberOfSamples(std::string message)
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

        
    }


}