//
// Created by juancarlos on 6/8/20.
//

#ifndef TYPE_CHECKER_H
#define TYPE_CHECKER_H

#include<unordered_map>
#include<unordered_set>
#include<string_view>
#include<functional>
#include<any>
#include<typeindex>



class attribute_types
{
public:
    static std::unordered_map<std::string_view, std::function<bool(const std::any&)>> map_fn_;

    static bool register_type(std::string_view s, const std::any& type , bool stream_type = false)
    {
        map_fn_.emplace(std::make_pair(s, [t = std::type_index(type.type()) ](const std::any &el) -> bool {
            return t == std::type_index(el.type());
        }));
        return true;
    }

    static bool check_type(std::string_view s, const std::any& val)
    {
        if (map_fn_.find(s) != map_fn_.end()) {
            return map_fn_.at(s)(val);
        } else  {
            register_type(s, val);
            return map_fn_.at(s)(val);
        }
    }

    static std::unordered_set<std::string_view> get_all()
    {
        std::unordered_set<std::string_view> types;
        for(auto const& type_t: map_fn_)
            types.emplace(type_t.first);
        return types;
    }

};

class node_types
{
    static std::unordered_set<std::string_view> set_type_;

public:

    static bool register_type(std::string_view s)
    {
        set_type_.emplace(s);
        return true;
    }

    static bool check_type(std::string_view v)
    {
        return set_type_.find(v) != set_type_.end();
    }

    static std::unordered_set<std::string_view> get_all()
    {
        return set_type_;
    }
};

class edge_types
{
    static std::unordered_set<std::string_view> set_type_;

public:

    static bool register_type(std::string_view s)
    {
        set_type_.emplace(s);
        return true;
    }

    static bool check_type(std::string_view v)
    {
        return set_type_.find(v) != set_type_.end();
    }

    static std::unordered_set<std::string_view> get_all()
    {
        return set_type_;
    }
};

#endif //TYPE_CHECKER_H
