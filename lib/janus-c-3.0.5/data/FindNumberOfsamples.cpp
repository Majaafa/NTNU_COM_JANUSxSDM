#include <fstream>
#include <cstdint>
#include <iostream>

int main()
{
    /*open .wav file*/
    std::ifstream file ("JanusRekneSampCou.wav");

    /*Skip bits if needed? (Skipper disse pga det er ein del av wav filen og blir ikkje den del av det vi sender?)*/
    file.seekg(44); 

    /*read number of samples for 16 bit per samples*/
    std::int16_t sample;
    std::size_t count = 0; 
    while (file.read(reinterpret_cast<char*>(&sample), sizeof(sample)))
    {
        ++count;
    }

    /*print number of samples*/
    std::cout << "number of samples are: " << count << std::endl;
}