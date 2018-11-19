//
// Created by aadc on 18.09.18.
//
// Source: https://github.com/mtrebi/thread-pool
#ifndef AADC_USER_SAFEQUEUE_H
#define AADC_USER_SAFEQUEUE_H
#pragma once

#include <mutex>
#include <queue>

// Thread safe implementation of a Queue using a std::queue
template <typename T>
class SafeQueue {
private:
    std::queue<T> m_queue;
    std::mutex m_mutex;
public:
    SafeQueue() = default;

    SafeQueue(SafeQueue& other) {
        //TODO:
    }

    ~SafeQueue() = default;


    bool empty() {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    int size() {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    void enqueue(T& t) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_queue.push(t);
    }

    bool dequeue(T& t) {
        std::unique_lock<std::mutex> lock(m_mutex);

        if (m_queue.empty()) {
            return false;
        }
        t = std::move(m_queue.front());

        m_queue.pop();
        return true;
    }
};
#endif //AADC_USER_SAFEQUEUE_H
