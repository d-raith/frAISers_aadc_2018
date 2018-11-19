//
// Created by aadc on 18.09.18.
//

// Source: https://github.com/mtrebi/thread-pool

#ifndef AADC_USER_THREADPOOL_H
#define AADC_USER_THREADPOOL_H

#pragma once

#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include "SafeQueue.h"

class ThreadPool {
private:
    class ThreadWorker {
    private:
        int m_id;
        ThreadPool * m_pool;
    public:
        ThreadWorker(ThreadPool * pool, const int id)
                :  m_id(id), m_pool(pool) {
        }

        void operator()() {
            std::function<void()> func;
            bool dequeued;
            while (!m_pool->m_shutdown) {
                {
                    std::unique_lock<std::mutex> lock(m_pool->m_conditional_mutex);
                    if (m_pool->m_queue.empty()) {
                        m_pool->m_conditional_lock.wait(lock);
                    }
                    dequeued = m_pool->m_queue.dequeue(func);
                }
                if (dequeued) {
                    func();
                }
            }
        }
    };


    SafeQueue<std::function<void()>> m_queue;
    std::vector<std::thread> m_threads;
    bool m_shutdown;
    std::mutex m_conditional_mutex;
    std::condition_variable m_conditional_lock;
public:
    explicit ThreadPool(const int n_threads)
            : m_threads(std::vector<std::thread>(n_threads)), m_shutdown(false) {
    }

    ThreadPool(const ThreadPool &) = delete;
    ThreadPool(ThreadPool &&) = delete;

    ThreadPool & operator=(const ThreadPool &) = delete;
    ThreadPool & operator=(ThreadPool &&) = delete;

    // Inits thread pool
    void init() {
        for (int i = 0; i < static_cast<int>(m_threads.size()); ++i) {
            m_threads[i] = std::thread(ThreadWorker(this, i));
        }
    }


    bool hasJobs(){
        return getJobCount()>0;
    }

    int getJobCount(){
        return m_queue.size();
    }

    // Waits until threads finish their current task and shutdowns the pool
    void shutdown() {
        m_shutdown = true;
        m_conditional_lock.notify_all();

        for (auto &m_thread : m_threads) {
            if(m_thread.joinable()) {
                m_thread.join();
            }
        }
    }

    // Submit a function to be executed asynchronously by the pool
    template<typename F, typename...Args>
    auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
        // Create a function with bounded parameters ready to execute
        std::function<decltype(f(args...))()> func = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
        // Encapsulate it into a shared ptr in order to be able to copy construct / assign
        auto task_ptr = std::make_shared<std::packaged_task<decltype(f(args...))()>>(func);

        // Wrap packaged task into void function
        std::function<void()> wrapper_func = [task_ptr]() {
            (*task_ptr)();
        };

        // Enqueue generic wrapper function
        m_queue.enqueue(wrapper_func);

        // Wake up one thread if its waiting
        m_conditional_lock.notify_one();

        // Return future from promise
        return task_ptr->get_future();
    }

    template<typename...Args>
    auto submit(std::function<void(Args...)> fn) -> std::future<decltype(fn)()> {
        // Create a function with bounded parameters ready to execute
        // Encapsulate it into a shared ptr in order to be able to copy construct / assign

        std::packaged_task<decltype(fn)()> task(fn);
        auto task_ptr = std::make_shared<std::packaged_task<decltype(fn)()>>(fn);

        // Wrap packaged task into void function
        std::function<void()> wrapper_func = [task_ptr]() {
            (*task_ptr)();
        };

        // Enqueue generic wrapper function
        m_queue.enqueue(wrapper_func);

        // Wake up one thread if its waiting
        m_conditional_lock.notify_one();

        // Return future from promise
        return task_ptr->get_future();
    }


    /*template<typename F, typename...Args>
    auto submit(std::function<void (const bool)> &func) -> std::future<decltype(func)> {
        // Create a function with bounded parameters ready to execute
        // Encapsulate it into a shared ptr in order to be able to copy construct / assign
        auto task_ptr = std::make_shared<std::packaged_task<decltype(func)()>>(func);

        // Wrap packaged task into void function
        std::function<void()> wrapper_func = [task_ptr]() {
            (*task_ptr)();
        };

        // Enqueue generic wrapper function
        m_queue.enqueue(wrapper_func);

        // Wake up one thread if its waiting
        m_conditional_lock.notify_one();

        // Return future from promise
        return task_ptr->get_future();
    }*/
};

#endif //AADC_USER_THREADPOOL_H
