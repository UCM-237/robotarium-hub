#pragma once
#include <vector>   
#include <pthread.h>
struct record_data//struct for share information between threads
{
    int id; //id of every robot
    double x, y , z;
    double yaw;
};  
struct ArenaLimits
{
    std::vector<double> x;
    std::vector<double> y;
};
const int BUFFER_SIZE =3;

pthread_mutex_t bufferMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t bufferNotEmpty = PTHREAD_COND_INITIALIZER;
class ringBuffer
{
    public:
        ringBuffer();
        ~ringBuffer();
        void push(const record_data& data);
        record_data pop();

    private:
       std::vector<record_data> buffer;
       
       size_t readPos = 0;
       size_t writePos = 0;
       bool full = false;
};