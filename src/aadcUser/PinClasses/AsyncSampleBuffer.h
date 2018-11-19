//
// Created by aadc on 27.09.18.
//

#ifndef AADC_USER_ASYNCSAMPLEBUFFER_H
#define AADC_USER_ASYNCSAMPLEBUFFER_H


#include <atomic>
#include <mutex>

template <class Data>
class AsyncSampleBuffer {
private:
    std::atomic<bool> hasData={false};
    std::mutex data_mutex;
    Data data;



public:
    void setData(Data data) {
        std::lock_guard<std::mutex> lock(data_mutex);
        this->data = data;
        hasData = true;
    }


    Data* getDataPtr() {
        return &data;
    }

    void setDataAvailable(bool data) {
        hasData = data;
    }

    bool hasDataAvailable(){
        return hasData;
    }

};


#endif //AADC_USER_ASYNCSAMPLEBUFFER_H
