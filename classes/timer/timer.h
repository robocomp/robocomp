// Use: 
/* 
  	Timer clock; // Timer<milliseconds, steady_clock>
	clock.tick();
		code you want to measure 
	clock.tock();
*/

#ifndef ROBOCOMP_TIMER_H
#define ROBOCOMP_TIMER_H

#include <chrono>
namespace rc
{

    template<class DT = std::chrono::milliseconds,
            class ClockT = std::chrono::system_clock>
    class Timer
    {
        using timep_t = typename ClockT::time_point;
        timep_t _start = ClockT::now(), _end = {};

    public:
        static auto now()
        {
            return std::chrono::duration_cast<DT>(ClockT::now().time_since_epoch()).count();
        }

        void tick()
        {
            _end = timep_t{};
            _start = ClockT::now();
        }

        void tock()
        { _end = ClockT::now(); }

        template<class T = DT>
        auto duration() const
        {
            //gsl_Expects(_end != timep_t{} && "toc before reporting");
            return std::chrono::duration_cast<T>(_end - _start).count();
        }
    };
}
#endif

