#include <iostream>
#include <string>
#include "misc.cc"
int main()
{

    struct record_data//struct for share information between threads
    {
        int id; //id of every robot
        double x, y , z;
        double rx,ry,rz;
        int n;  //counter for know how many robots there are
        
    };

    unsigned char mensaje[32];

    record_data data;

    data.id=-20;
    data.x=-20.50; data.y = 10; data.z = 10;
    data.n=30;

    longToBytes(data.id,&mensaje[0]);
    doubleToBytes(data.x,&mensaje[4]);
    int g;
    double d;
    g=bytesToLong(&mensaje[0]);
    d=bytesToDouble(&mensaje[4]);
    std::cout<<d<<std::endl;
    return 0;
}