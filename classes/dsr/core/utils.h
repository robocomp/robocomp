//
// Created by juancarlos on 21/8/20.
//

#ifndef UTILS_H
#define UTILS_H



class hash_tuple {

    template<class T>
    struct component {
        const T &value;

        component(const T &value) : value(value) {}

        uintmax_t operator,(uintmax_t n) const {
            n ^= std::hash<T>()(value);
            n ^= n << (sizeof(uintmax_t) * 4 - 1);
            return n ^ std::hash<uintmax_t>()(n);
        }
    };

public:
    template<class Tuple>
    size_t operator()(const Tuple &tuple) const {
        return std::hash<uintmax_t>()(
                std::apply([](const auto &... xs) { return (component(xs), ..., 0); }, tuple));
    }
};


struct hash_pair
{
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

static uint64_t get_unix_timestamp() { //Move to utilities?
    std::time_t st = std::time(nullptr) ;
    auto secs = static_cast<std::chrono::seconds>(st).count();
    return static_cast<uint64_t>(secs);
}

#endif //UTILS_H
