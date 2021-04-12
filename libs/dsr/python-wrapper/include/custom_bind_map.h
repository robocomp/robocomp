//
// Created by juancarlos on 12/4/21.
//

#ifndef DSR_CUSTOM_BIND_MAP_H
#define DSR_CUSTOM_BIND_MAP_H


#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

#pragma pop_macro("slots")

// Map assignment when copy-assignable: just copy the value
template <typename Map, typename Class_>
void map_assignment
    ( std::enable_if_t<std::is_same<DSR::Attribute, typename Map::mapped_type>::value && std::is_same<std::string, typename Map::key_type>::value, Class_> &cl) {
    using KeyType = std::string;
    using MappedType = DSR::Attribute;

    cl.def("__setitem__",
           [](Map &m, const KeyType &k, const MappedType &v) {
               if (attribute_types::check_type(k.data(), v))
               {
                    auto it = m.find(k);
                    if (it != m.end()) it->second = v;
                    else m.emplace(k, v);
                } else
                {
                    throw std::runtime_error("Invalid attribute type for attribute ." + k);
                }
           }
    );
}



#endif //DSR_CUSTOM_BIND_MAP_H
