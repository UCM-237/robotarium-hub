#include "ringBuffer.h"

ringBuffer::ringBuffer()
{
    buffer.resize(BUFFER_SIZE);
}

ringBuffer::~ringBuffer()
{
}

void ringBuffer::push(const record_data &data)
{
    pthread_mutex_lock(&bufferMutex);
    this->buffer[this->writePos] = data;
    this->writePos = (this->writePos + 1) % BUFFER_SIZE;
    if (this->writePos == this->readPos) {
        this->full = true;
        //this->readPos = (this->readPos) % BUFFER_SIZE;
    }
    pthread_cond_signal(&bufferNotEmpty); // Señalar que hay datos disponibles
    pthread_mutex_unlock(&bufferMutex);
}

record_data ringBuffer::pop()
{
    pthread_mutex_lock(&bufferMutex);
    while (writePos == readPos) {
        // Esperar hasta que haya datos disponibles en el buffer
        pthread_cond_wait(&bufferNotEmpty, &bufferMutex);
    }
    // if (!full && writePos == readPos) {
    //     // El buffer está vacío
    //     pthread_mutex_unlock(&bufferMutex);
    //     throw std::runtime_error("Buffer is empty.");
    // }
    record_data data = buffer[readPos];
    readPos = (readPos + 1) % BUFFER_SIZE;
    full = false;
    pthread_mutex_unlock(&bufferMutex);
    return data;
}

