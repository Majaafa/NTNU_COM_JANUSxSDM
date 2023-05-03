// Library that simplifies interfacing with janus and sdm for subsea acoustic communication
// This library requires that sdmsh and janus is installed

// This library is not an ideal solution
// An ideal version was not chosen due to time-constraints, lack of experience and lack of documentation for both janus.h and sdm.h
// People seeking to implement an efficent version of JANUS with sdm should look into implementing the libraries directly


#pragma once


namespace janusxsdm
{
    class connection
    {
    private:
        std::string mIP, JPATH, SPATH;
        int RX_PORT, TX_PORT;
        static uint8_t SDM_FRAMEHEADER[];
    public:
        //Constructor, takes IPv4 of acoustic modem, paths to janus and sdmsh executables and ports to reserve for janus encoding/decoding
        connection(std::string modemIP, std::string JANUSPATH, std::string SDMPATH, int rxPort, int txPort);

        //Setting correct config on sdm
        int sdmconf();

        //Setting custom config on sdm
        std::string sdmconfDialogue();

        //Sets the correct preamble for janus-data
        int setPreamble();

        //Encodes message to janus in a buffer
        void sendSimple(std::string message);

        //Code comented out as it does not work, the idea is that you can call this funtion with a binary buffer and get the message payload
        int decode(int16_t buf[], std::string &message);

        //Simplified sdm listener, will break if communication is incomplete or missing
        int listenSimple(std::string &message);

        int beginListen();

        //Listener that stops blocking after timeout (works 9/10 times, needs fixing, generates zombies i think)
        int listen(std::string &message, std::chrono::duration<double> timeout);

        //Beginning of an attempt to a direct com with the modem
        int printheader();

        //Sends the stop commandt to modem over sdm (not implemented)
        int sdmStop();

        //for finding he number of samples from janus wav
        void findNumberOfSamples(std::string message);
    };
}