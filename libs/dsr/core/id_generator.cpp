//
// Created by juancarlos on 19/1/21.
//

#include <dsr/core/id_generator.h>
#include <thread>
#include <iostream>
#include <iomanip>

inline std::chrono::system_clock::time_point id_generator::get_time_point()
{
    auto now = std::chrono::system_clock::now();
    std::chrono::time_point pt_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    if (pt_ns < last_point)
    {
        return last_point;
    }
    last_point = pt_ns;
    return pt_ns;
}

uint64_t id_generator::generate()
{
    constexpr uint16_t mask_counter = (1 << counter_size) - 1;
    std::unique_lock<id_generator::mtx_type> lock(mtx);
    uint64_t current = get_time_point().time_since_epoch().count() / time_unit - start_time / time_unit;
    if (elapsed_time < current)
    {
        elapsed_time = current;
        counter = 0;
    }
    else
    {
        counter = (counter + 1) & mask_counter;
        if (counter == 0)
        {
            elapsed_time += 1;
            auto overtime = elapsed_time - current;
            auto tp = std::chrono::system_clock::time_point{
                    std::chrono::milliseconds{overtime * 10}} -
                      std::chrono::system_clock::time_point {std::chrono::nanoseconds{std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() % (int64_t)time_unit}};

            /*std::cout << "BLOCK  " << tp.count()
                      << std::endl;*/
            std::this_thread::sleep_for(tp);
        }
    }

    if (elapsed_time >= (static_cast<uint64_t>(1) << time_size))
    {
        throw std::logic_error("time is over the limit, restart the agent");
    }

    return (static_cast<uint64_t>(elapsed_time) << (agent_id_size + counter_size)) |
           (static_cast<uint64_t>(counter) << agent_id_size) |
           static_cast<uint64_t>(agent_id);
}

std::tuple<uint64_t, uint16_t, uint16_t> id_generator::parse(uint64_t id)
{
    constexpr uint64_t mask_counter  = ((1ULL << counter_size) - 1) << agent_id_size;
    constexpr uint64_t mask_agent_id = (1ULL << agent_id_size) - 1;

    uint64_t time = id >> (agent_id_size + counter_size);
    uint16_t counter = (id & mask_counter) >> counter_size;
    uint16_t agent_id = id & mask_agent_id;

    return std::make_tuple(time, counter, agent_id);
}

std::string id_generator::hex_string(uint64_t id)
{
    auto [time, counter, agent] = id_generator::parse(id);

    std::stringstream stream;
    stream << std::setfill('0') << std::setw(10)
           << std::hex << time;

    stream << "-";
    stream << std::setfill('0') << std::setw(3)
           << std::hex << counter;

    stream << "-";
    stream << std::setfill('0') << std::setw(3)
           << std::hex << agent;

    return stream.str();
}