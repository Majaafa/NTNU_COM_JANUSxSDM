#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<string.h>
#include<pthread.h>
using namespace std;

//some paths to commonly used locations
string JANUSPATH = "/home/markerv/Documents/Bproject/janus-c-3.0.5/";
string SCRIPTPATH = "/home/markerv/Ros/NTNU_ROV_COM/lib/tools/";

string myMac = "98:fa:9b:7e:7f:83";


int janus_tx(string data) //function for sending data
{ 
    string payload = myMac + "," + data;
    cout << "Sending: " << payload << endl;
    string command = SCRIPTPATH + "janus-send.sh " + payload;
    system(command.c_str());
    return 1;
}

string janus_rx()
{
    string response;
    string spoofedMac = "98:fa:9b:7e:8a:62";
    cin >> response;
    //cout << endl << "Recieved response: " << response << endl;
    return response;
}



int init(){
    bool haveMaster = false;
    cout << "Asking if master exist" << endl;
    janus_tx("££?MM");

    time_t start;
    time(&start);
    double delay = 10;
    cout << "Waiting " << delay << "s for master to respond" << endl; 
    while(double((time(NULL)-start) <= delay) and !haveMaster)
    {
        string response = janus_rx();
        cout << "Recieved response: " << response << endl;
        if (response == "££!MM")//- Venter på respons for om master eksisterer
        {
            cout << "Found master on address: " << "Not implemented" << endl;
            haveMaster = true;
        }
        
    }
    if(!haveMaster)
    {
        cout << "No master found :(" << endl << "I wil be master!" << endl;
    }
    return 1;
}


int main(){
    init();
    /*
    string input;
    while (true) //simple recurring loop for user input, this input is then passed to janus_tx
    {
        cout << endl << endl << "Enter data or command\nexit - stop transmissionscript\n";
        cin >> input;
        cout << endl << "Your command was: " << input << endl;
        if(input == "exit") //Terminate loop with exit command
        {
            break;
        }
        janus_tx(input);
        cout << "Data sent!";
    }

    */
    
    
    return 0;
}