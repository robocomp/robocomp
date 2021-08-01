//
// Created by juancarlos on 6/5/21.
//

#ifndef DSR_AGENTINFO_API_H
#define DSR_AGENTINFO_API_H


#include <dsr/core/types/type_checking/dsr_node_type.h>
#include <dsr/core/types/type_checking/dsr_attr_name.h>
#include <dsr/api/dsr_api.h>
#include <sys/syscall.h>
#include <sys/types.h>

namespace DSR {

    class DSRGraph;

    class Timer {

        std::function<void()> Fn;
        std::thread thr;
        std::atomic_bool done{ false };
        std::atomic_int period_ms;

    public:
        Timer(int period_ms_, std::function<void()> fn) :  Fn(std::move(fn)), period_ms(period_ms_)
        {

            thr = std::thread([this](){
                auto now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()); //Time point before calling Fn.
                while (!done.load(std::memory_order_acquire)) {
                    auto period_iter = std::chrono::microseconds {period_ms.load(std::memory_order_acquire) * 1000};
                    Fn();
                    auto end_call =  std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()); //Time point after calling Fn.
                    std::chrono::microseconds wait_time = (end_call - now);
                    if (auto t = period_iter - wait_time; t.count() > 0 )
                    {
                        qDebug() << "[TIMER - DEBUG] Execution time was: "<<  static_cast<double>(wait_time.count())/1000
                               << "ms. Sleeping " << t.count()/1000
                               << "ms. Total: " << static_cast<double>(wait_time.count())/1000+ static_cast<double>(t.count())/1000;
                        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(t/1000));
                    } else {
                        qWarning() << "[TIMER] Execution time it's longer than period.";
                    }
                    now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());;
                }
            });
        }

        ~Timer() {
            stop_timer();
        }

        void stop_timer()
        {
            bool f = false;
            while (!done.compare_exchange_strong(f, true, std::memory_order_acq_rel)){}
            if (thr.joinable()) thr.join();
            qDebug("Timer Stopped");
        }

        void period(int period)
        {
            period_ms.store(period, std::memory_order_acq_rel);
        }

        bool is_running()
        {
            return !thr.joinable();
        }

    };


    class AgentInfoAPI {

    public:
        explicit AgentInfoAPI(DSR::DSRGraph *g, uint32_t period_ = 1000)
        : G(g), period(period_), timer(period, [this]() { create_or_update_agent();})
        {}

        void stopTimer();
        bool isRunning();
        //void setPriod(uint32_t period_);

    private:

        static std::string exec(const char* cmd);
        void create_or_update_agent();

        DSRGraph *G;
        uint64_t timestamp_start{0};
        uint32_t period;
        Timer timer;
    };

}

#endif //DSR_AGENTINFO_API_H
