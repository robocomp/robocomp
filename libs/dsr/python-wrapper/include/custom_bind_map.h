//
// Created by juancarlos on 12/4/21.
//

#ifndef DSR_CUSTOM_BIND_MAP_H
#define DSR_CUSTOM_BIND_MAP_H

#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

#pragma pop_macro("slots")

inline static constexpr std::array<std::string_view, 17> attribute_type_TYPENAMES_UNION =
        { "STRING", "BOOL", "NUMPY_BYTE_VEC", "NUMPY_FLOAT_VEC",
          "NUMPY_U64_VEC","BYTE_VEC", "FLOAT_VEC", "UINT64_VEC",
          "FLOAT_VEC2", "FLOAT_VEC3","FLOAT_VEC4", "FLOAT_VEC6",
          "UINT64", "DOUBLE", "FLOAT","INT", "UINT",
         };



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

        cl.def("__repr__",
               [](const Map &m) {
                   std::stringstream out;
                   for (const auto& [k, v] : m)
                       out << "      [" << k << "] -- " << v << std::endl;
                   return out.str();
                }
        );

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
                   //std::cout <<"[SET MAP ITEM] " << k << " "<< v.selected() << " [ "<< v <<" ]" <<std::endl;
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
                           if (attribute_types::check_type((k).data(), v.float_vec())) {
                               correct_type = true;
                           } else if (std::array<float, 2> tmp{}; attribute_types::check_type(k.data(), tmp))
                           {
                               tmp = {v.float_vec()[0], v.float_vec()[1]};
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.vec2(tmp);
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           } else if (std::array<float, 3> tmp{}; attribute_types::check_type(k.data(), tmp))
                           {
                               tmp = {v.float_vec()[0], v.float_vec()[1], v.float_vec()[2]};
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.vec3(tmp);
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           } else if (std::array<float, 4> tmp{}; attribute_types::check_type(k.data(), tmp))
                           {
                               tmp = {v.float_vec()[0], v.float_vec()[1], v.float_vec()[2],v.float_vec()[3]};
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.vec4(tmp);
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           } else if (std::array<float, 6> tmp{}; attribute_types::check_type(k.data(), tmp))
                           {
                               tmp = {v.float_vec()[0], v.float_vec()[1], v.float_vec()[2],v.float_vec()[3], v.float_vec()[4], v.float_vec()[5]};
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.vec6(tmp);
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           }
                           break;
                       case 4:
                           correct_type = attribute_types::check_type((k).data(), v.bl());
                           break;
                       case 5:
                           if (attribute_types::check_type((k).data(), v.byte_vec())) {
                               correct_type = true;
                           } else if (std::vector<uint64_t> tmp{}; attribute_types::check_type(k.data(), tmp))
                           {
                               tmp = std::vector<uint64_t>{v.byte_vec().begin(), v.byte_vec().end()};
                               auto it = m.find(k);
                               auto new_val = v;
                               new_val.u64_vec(tmp);
                               if (it != m.end()) it->second = new_val;
                               else m.emplace(k, new_val);
                               return;
                           }
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
                       case 9:
                           correct_type = attribute_types::check_type((k).data(), v.u64_vec());
                           break;
                       case 10:
                           correct_type = attribute_types::check_type((k).data(), v.vec2());
                           break;
                       case 11:
                           correct_type = attribute_types::check_type((k).data(), v.vec3());
                           break;
                       case 12:
                           correct_type = attribute_types::check_type((k).data(), v.vec4());
                           break;
                       case 13:
                           correct_type = attribute_types::check_type((k).data(), v.vec6());
                           break;
                       default:
                           throw /*std::runtime_error*/ pybind11::type_error("[Not implemented type] Invalid type for attribute [" + k + "]. Selected type is: " + DSR::TYPENAMES_UNION[v.selected()].data());
                   }

                    if (correct_type) {
                        auto it = m.find(k);
                        if (it != m.end()) it->second = v;
                        else m.emplace(k, v);
                    } else {
                        throw /*std::runtime_error*/ pybind11::type_error("Invalid type for attribute [" + k + "]. Selected type is: " + DSR::TYPENAMES_UNION[v.selected()].data());
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
