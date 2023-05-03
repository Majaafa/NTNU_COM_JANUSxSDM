

#pragma once


namespace Evo_janusXsdm
{
    class connection
    {
    private:
        std::string mIP, JPATH, SPATH;
        int RX_PORT, TX_PORT;
        float stream_fs;
        static uint8_t SDM_FRAMEHEADER[];

        // Will return the number of samples in a janus pkg.
        // Need messages as a string
        // Creats a wav. file and make some calulations, there are limitations to the size of message
        float getNumberOfSamples(std::string message); 

        // Will return the reservation time for a janus pkg
        // Need number of samples from a janus pkg
        // There are one number that is hardcoded for stream-fs 250000, recemeded sulution is to run "printNumberOfSamples" and dont add a cargo (message = ""), this will get samples of an emty cargo. Add this new value to "samples_no_cargo"
        float getPacketReservTime(float samplesCount);

        std::string findInJanus_frame(std::string idStr,std::string janusframe);

    public:
        //Constructor, takes IPv4 of acoustic modem, paths to janus and sdmsh executables and ports to reserve for janus encoding/decoding
        connection(std::string modemIP, std::string JANUSPATH, std::string SDMPATH, int rxPort, int txPort, float stream_fs);

        void setPHY();//TODO

        //2022: Setting correct config on sdm for air test
        int sdmConfigAir();

        //2022: Setting custom config on sdm
        std::string sdmConfig();

        //2022: Sets the correct preamble for janus-data
        int setPreamble();

        // NOT part of RX fam!
        // EZ to use, for dummies, will cleanup its selfe, no inseption,  
        //Need to run a "sdmConfig.." and a "setPreamble" before first time running this commands.
        std::array<std::string,6> listenOnceTheFoolproofRX(std::string &message);

        //TX: sends message
        //Need to set modem in PHY mode 
        //Need to run a "sdmConfig.." and a "setPreamble" before first time running this commands. 
        void startTX(std::string message); //sends message

        //Part of RX fam: -> read dokumentation before use 
        //Begins the sdmsh and janus commands for RX, 
        //Creates a pipe connected to a output stream from janus, read end of pipe is returned (pipe[0])
        //1. Need to set modem in PHY mode 
        //2. Need to run a "sdmConfig.." and a "setPreamble" before first time running this commands.
        int startRX(); //begins the sdmsh and janus commands and creates a pipe and return read side of pipe (pipe[0])

        //Part of RX fam: -> read dokumentation before use 
        //RX fam: will listen to the pipe from startRX and return message if recieved or timeout
        std::array<std::string,6> listenRX(int infoFromJanus,std::string &message); //will listen to the pipe from startRX and return message if recieved or timeout

        //Part of RX fam: -> read dokumentation before use 
        //Will crash the RX processes 
        void stopRX();  

        //Part of RX fam:
        //Will close a pipe created by the "startRx" command. When closing the program or plan to run the "startRx" a second time, "closePipe" need to be run. 
        void closePipeRX(int &pipe); 

        //For finding he number of samples from janus wav, can use this to debug or learn about janus and TCP. For fun. 
        void printNumberOfSamples(std::string message);

        void dummyFlushJanusTX();
        
    };
}