#include<iostream>
#include<stdlib.h>
#include<string.h>
#include<pthread.h>

int sdmsh(){
    if(system(NULL) != 0){
        system("/home/markerv/Documents/Bproject/scripts/sdmsh_listen.sh");
        return 1;
    }
    else{
        return 0;
    }
}

int janus(){
    if(system(NULL) != 0){
        int variable = system("/home/markerv/Documents/Bproject/scripts/janus_listen.sh");
        std::cout << variable << "\n";
        return 1;
    }
    else return 0;
}

int main(){
    /*
    if(janus() <= 0){
        std::cout << "System not available, janus not ready\n";
        return -1;
    }
    */
    std::cout << system("/home/markerv/Documents/Bproject/scripts/janus_listen.sh") << "\n";
    
    return 0;
}