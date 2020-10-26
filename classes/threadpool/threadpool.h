//
// Created by juancarlos on 15/10/20.
//

#ifndef SIMPLE_THREADPOOL
#define SIMPLE_THREADPOOL

#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <vector>

//using namespace std::chrono_literals;

class function_wrapper_base
{
public:
    virtual ~function_wrapper_base(){};
    virtual void operator()() {};
};

template <typename Function, typename... Arguments>
class function_wrapper : public function_wrapper_base
{
public:
    function_wrapper(Function &&fn, std::tuple<Arguments...> args) : f(std::forward<Function>(fn)),
                                                                     args(std::move(args)){};

    void operator()() override
    {
        std::apply(f, std::move(args));
    }

private:
    Function f;
    std::tuple<Arguments...> args;
};

class ThreadPool
{
public:

    ThreadPool(const ThreadPool &tp) = delete;
    ThreadPool(ThreadPool &tp) = delete;
    ThreadPool &operator=(const ThreadPool &tp) = delete;

    ThreadPool(uint32_t num_threads = 0) : done(false)
    {
        uint32_t nt = (num_threads == 0) ? std::thread::hardware_concurrency() : num_threads;
        for (std::size_t i = 0; i < nt; i++)
        {
            threads.emplace_back(std::thread(&ThreadPool::thread_loop, this, i));
        }
    }

    ~ThreadPool()
    {

        std::unique_lock<std::mutex> lock(tp_mutex);
        std::queue<std::unique_ptr<function_wrapper_base>> tmp;
        std::swap(tasks, tmp);
        lock.unlock();

        done = true;
        cv.notify_all();

        for (auto &th : threads)
        {
            if (th.joinable())
                th.join();
        }
    }

    uint32_t remaining_tasks() {
        return tasks.size();
    }

    template <typename Function, typename... Arguments,
            typename = std::enable_if_t<std::is_invocable<Function &&, Arguments &&...>::value /*&& std::is_same<std::result_of_t<Function(Arguments...)>, void>::value*/>>
    void spawn_task(Function &&fn, Arguments &&... args)
    {
        std::unique_lock<std::mutex> task_queue_lock(tp_mutex, std::defer_lock);
        task_queue_lock.lock();
        auto tmp_ptr = std::unique_ptr<function_wrapper_base>(new function_wrapper<Function, Arguments...>(std::forward<Function>(fn), std::forward_as_tuple(std::move(args)...)));
        tasks.emplace(std::move(tmp_ptr));
        task_queue_lock.unlock();
        cv.notify_one();
    }

    template <typename Function, typename... Arguments,
            typename = std::enable_if_t<std::is_invocable<Function &&, Arguments &&...>::value>>
    auto spawn_task_waitable(Function &&fn, Arguments &&... args)
    {
        std::unique_lock<std::mutex> task_queue_lock(tp_mutex, std::defer_lock);

        auto task = std::packaged_task<std::invoke_result_t<Function, Arguments...>()>(
                [fn_ = std::forward<Function>(fn), args_ = std::forward_as_tuple(args...)]() mutable -> auto {
                    return std::apply(std::move(fn_), std::move(args_));
                }
        );

        auto future = task.get_future();

        task_queue_lock.lock();
        auto tmp_ptr = std::unique_ptr<function_wrapper_base>(
                new function_wrapper<std::packaged_task<std::invoke_result_t<Function, Arguments...>()>>(
                        std::move(task), std::tuple<>{}));
        tasks.emplace(std::move(tmp_ptr));
        task_queue_lock.unlock();
        cv.notify_one();

        return future;
    }

private:

    void thread_loop(int i)
    {
        [[maybe_unused]] static thread_local uint32_t thread_index = i;
        std::unique_lock<std::mutex> task_queue_lock(tp_mutex, std::defer_lock);
        while (!done)
        {
            task_queue_lock.lock();

            cv.wait(task_queue_lock,
                    [&]() -> bool { return !tasks.empty() || done; });

            if (done) {
                task_queue_lock.unlock();
                cv.notify_all();
                break;
            }

            std::unique_ptr<function_wrapper_base> t = std::move(tasks.front());
            tasks.pop();
            task_queue_lock.unlock();

            if (t != nullptr)
            {
                (*t)();
            }

        }
        //std::cout << "Terminando thread " << thread_index << std::endl;
    }

    std::vector<std::thread> threads;
    std::queue<std::unique_ptr<function_wrapper_base>> tasks;
    static thread_local uint32_t thread_index;
    std::condition_variable cv;
    std::atomic_bool done = false;
    mutable std::mutex tp_mutex;
};


#endif