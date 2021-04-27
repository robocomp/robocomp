//
// Created by juancarlos on 15/10/20.
// This class is a simple threadpool with a thread-safe interface.
// The interface offers methods to execute either asynchronous tasks from which
// no result is expected using the spawn_task method, or asynchronous tasks from
// which a result is expected, which return a future object (https://en.cppreference.com/w/cpp/thread/future)
// with the method spawn_task_waitable.
// It can be used as a set of worker threads for executing tasks sent by a thread that read periodically
// from the network (Used in dsr).
// It is also used to launch tasks that the user does not need to wait for to complete (Used in doublebuffer).
// Another use would be to spawn multiple tasks that can be executed in parallel and wait for all of them to
// be completed after that.
//Example 1:
//    void print_a(int a) {
//        std::cout << a << std::endl;
//    }
//   ...
//   ThreadPool worker(1); //Creates a threadpool with one worker thread.
//    wroker.spawn task(print_a, 24); //Add task to the work queue.
//
//Example 2: when using a lambda is important to capture by this, move or copy the values
//that are going to be used to avoid problems with objets lifetimes.
// ...
//void do_something(const std::vector<int> &vec) {
//    this is ok.
//    worker.spawn_task([vec = vec](){
//        auto x = 0;
//        for (auto &e : vec) {
//            x+=e;
//        }
//    });
//
//    //this is not ok. The tasks can be executed when vec is no longer valid.
//    worker.spawn_task([](){
//        auto x = 0;
//        for (auto &e : vec) {
//            x+=e;
//        }
//    });
//
//    //this is not ok. The tasks can be executed when vec is no longer valid.
//    worker.spawn_task([](const std::vector<int> &vec){
//        auto x = 0;
//        for (auto &e : vec) {
//            x+=e;
//        }
//    }, vec);
//}
//...
//Example 3: Whe should move values when possible. We change the ownership of the objects so they not are destroyed.
//BifObject bo;
//  worker.spawn_task([std::move(bo)]() mutable {
//      ...;
//  });
//or
//  worker.spawn_task([](BigObject &&bo) {
//     ...;
//  }, std::move(bo));
//
//Example 4: waitable tasks.
//ThreadPool tp();
//   std::vector<std::future<int>> futs;
//   for (int i = 0; i < 10; i++ ) {
//     futs.push_back(tp.spawn_task_waitable([]() -> int {
//      ...
//     }));
//   }
//
//   Wait until all tasks return.
//   for (auto &f : futs) {
//      f.get();
//   }
//
//Example 5: When we want to execute a member function of an object we have to do it like this.
//   std::string s = "aaa";
//   auto f = tp.spawn_task_waitable(empty, s);
//or if need to use this.
//   tp.spawn_task(DSRGraph::join_delta_node_att, this, ...);
//and in a lambda:
//   tp.spawn_task([this]() { ... });


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


template<typename ... T>
concept only_rvalues  = (std::negation< std::bool_constant<std::is_lvalue_reference<T&&>::value> >::value  && ...);


//Base Virtual Object to store any kind of callable objects and its arguments.
class function_wrapper_base
{
public:
    virtual ~function_wrapper_base(){};
    virtual void operator()() {};
};

//Specialization
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

    //The threadpool can't be copied.
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

    template <typename Function, typename... Arguments>
    void spawn_task(Function &&fn, Arguments &&... args)
        requires (only_rvalues<Function&&>,  only_rvalues<Arguments&& ...>, std::is_invocable<Function &&, Arguments &&...>::value)
    {
        std::unique_lock<std::mutex> task_queue_lock(tp_mutex, std::defer_lock);
        task_queue_lock.lock();
        auto tmp_ptr = std::unique_ptr<function_wrapper_base>(new function_wrapper<Function, Arguments...>(std::forward<Function>(fn), std::forward_as_tuple(std::move(args)...)));
        tasks.emplace(std::move(tmp_ptr));
        task_queue_lock.unlock();
        cv.notify_one();
    }

    template <typename Function, typename... Arguments>
    auto spawn_task_waitable(Function &&fn, Arguments &&... args)
        requires (only_rvalues<Function&&>,  only_rvalues<Arguments&& ...>, std::is_invocable<Function &&, Arguments &&...>::value)
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
    }

    std::vector<std::thread> threads;
    std::queue<std::unique_ptr<function_wrapper_base>> tasks;
    static thread_local uint32_t thread_index;
    std::condition_variable cv;
    std::atomic_bool done = false;
    mutable std::mutex tp_mutex;
};


#endif