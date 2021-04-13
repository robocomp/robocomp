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

namespace pybind11 {

    template<typename Map, typename holder_type = std::unique_ptr<Map>, typename... Args>
    class_ <Map, holder_type> bind_dsr_map(handle scope, const std::string &name, Args &&... args) {
        using KeyType = typename Map::key_type;
        using MappedType = typename Map::mapped_type;
        using Class_ = class_<Map, holder_type>;

        // If either type is a non-module-local bound type then make the map binding non-local as well;
        // otherwise (e.g. both types are either module-local or converting) the map will be
        // module-local.
        auto tinfo = detail::get_type_info(typeid(MappedType));
        bool local = !tinfo || tinfo->module_local;
        if (local) {
            tinfo = detail::get_type_info(typeid(KeyType));
            local = !tinfo || tinfo->module_local;
        }

        Class_ cl(scope, name.c_str(), pybind11::module_local(local), std::forward<Args>(args)...);

        cl.def(init<>());

        // Register stream insertion operator (if possible)
        detail::map_if_insertion_operator<Map, Class_>(cl, name);

        cl.def("__bool__",
               [](const Map &m) -> bool { return !m.empty(); },
               "Check whether the map is nonempty"
        );

        cl.def("__iter__",
               [](Map &m) { return make_key_iterator(m.begin(), m.end()); },
               keep_alive<0, 1>() /* Essential: keep list alive while iterator exists */
        );

        cl.def("items",
               [](Map &m) { return make_iterator(m.begin(), m.end()); },
               keep_alive<0, 1>() /* Essential: keep list alive while iterator exists */
        );

        cl.def("__getitem__",
               [](Map &m, const KeyType &k) -> MappedType & {
                   auto it = m.find(k);
                   if (it == m.end())
                       throw key_error();
                   return it->second;
               },
               return_value_policy::reference_internal // ref + keepalive
        );

        cl.def("__contains__",
               [](Map &m, const KeyType &k) -> bool {
                   auto it = m.find(k);
                   if (it == m.end())
                       return false;
                   return true;
               }
        );


        cl.def("__setitem__",
               [](Map &m, const KeyType &k, const MappedType &v) {

                   bool correct_type = false;
                   switch (v.selected())
                   {
                       case 0:
                           correct_type = attribute_types::check_type((k).data(), v.str());
                           break;
                       case 1: //probably not going through this branch
                           correct_type = attribute_types::check_type((k).data(), v.dec());
                           break;
                       case 2: //probably not going through this branch
                           correct_type = attribute_types::check_type((k).data(), v.fl());
                           break;
                       case 3:
                           correct_type = attribute_types::check_type((k).data(), v.float_vec());
                           break;
                       case 4:
                           correct_type = attribute_types::check_type((k).data(), v.bl());
                           break;
                       case 5:
                           correct_type = attribute_types::check_type((k).data(), v.byte_vec());
                           break;
                       case 6: //probably not going through this branch
                           correct_type = attribute_types::check_type((k).data(), v.uint());
                           break;
                       case 7:
                           if (attribute_types::check_type((k).data(), v.uint64())){
                               correct_type = true;
                           }
                           else if (attribute_types::check_type((k).data(), static_cast<uint32_t>(v.uint64())))
                           {
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.uint(v.uint64());
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           } else if (attribute_types::check_type((k).data(), static_cast<int32_t>(v.uint64())))
                           {
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.dec(v.uint64());
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           }
                           break;
                       case 8:
                           if (attribute_types::check_type((k).data(), v.dob()))
                           {
                               correct_type = true;
                           } else if (attribute_types::check_type((k).data(), static_cast<float>(v.dob())))
                           {
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.fl(v.dob());
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           }
                           break;
                       default:
                           throw std::runtime_error("Invalid attribute type for attribute ." + k);
                   }

                    if (correct_type) {
                        auto it = m.find(k);
                        if (it != m.end()) it->second = v;
                        else m.emplace(k, v);
                    } else {
                        throw std::runtime_error("Invalid attribute type for attribute ." + k);

                    }

               }
        );

        cl.def("__delitem__",
               [](Map &m, const KeyType &k) {
                   auto it = m.find(k);
                   if (it == m.end())
                       throw key_error();
                   m.erase(it);
               }
        );

        cl.def("__len__", &Map::size);

        return cl;
    }

}
#endif //DSR_CUSTOM_BIND_MAP_H
