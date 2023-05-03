#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<string.h>
#include<pthread.h>
using namespace std;

//some paths to commonly used locations
string SCRIPTPATH = "..";

int janus_tx(string data) //function for sending data
{
    string command = "(cd " + SCRIPTPATH + " && bash login.sh \"" + data + "\" 198)";
    system(command.c_str());
    return 1;
}



int main(){
    string input;
    while (true) //simple recurring loop for user input, this input is then passed to janus_tx
    {
        cout << endl << endl << "Enter data or command\nexit - stop transmissionscript\n";
        std::getline(std::cin, input);
        cout << endl << "Your command was: " << input << endl;
        if(input == "exit") //Terminate loop with exit command
        {
            break;
        }
        janus_tx(input);
        cout << "Data sent!";
    }
    
    return 0;
}
