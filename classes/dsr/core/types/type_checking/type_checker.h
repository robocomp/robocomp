//
// Created by juancarlos on 6/8/20.
//

#ifndef TYPE_CHECKER_H
#define TYPE_CHECKER_H

#include<unordered_map>
#include<string_view>
#include<functional>
#include<any>
#include<typeindex>
#include "dsr_node_type.h"
#include "dsr_edge_type.h"


class ATTRIBUTE_TYPES
{
public:
    static std::unordered_map<std::string_view, std::function<bool(const std::any&)>> map_fn_;

    static bool REGISTER(std::string_view s, const std::any& type )
    {
        map_fn_.emplace(std::make_pair(s, [/*s,*/ t = std::type_index(type.type()) ](const std::any &el) -> bool {
            //std::cout << t.name() <<  "  " <<  std::type_index(el.type()).name() << std::endl;
            return t == std::type_index(el.type());
        }));
        return true;
    }

    static bool CHECKTYPE(std::string_view s, const std::any& val)
    {
        if (map_fn_.find(s) != map_fn_.end()) {
            return map_fn_.at(s)(val);
        } else  {
            REGISTER(s, val);
            return map_fn_.at(s)(val);
        }
    }
};

class NODE_TYPES
{
    template<std::size_t... I>
    inline static constexpr bool valid_name(std::string_view s, std::index_sequence<I...>)
    {
        return ((s == node_type_names[I]) || ...);
    }

public:

    static constexpr bool find(std::string_view v)
    {
        return valid_name(v, std::make_integer_sequence<std::size_t, node_type_names.size()>{});
    }
};

class EDGE_TYPES
{

    template<std::size_t... I>
    inline static constexpr bool valid_name(std::string_view s, std::index_sequence<I...>)
    {
        return ((s == edge_type_names[I]) || ...);
    }
public:

    static constexpr bool find(std::string_view v)
    {
        return valid_name(v, std::make_integer_sequence<std::size_t, edge_type_names.size()>{});
    }
};

#endif //TYPE_CHECKER_H
