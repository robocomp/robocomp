//
// Created by juancarlos on 22/6/21.
//

#ifndef DSR_CUSTOM_BOOL_CAST_H
#define DSR_CUSTOM_BOOL_CAST_H

#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

#pragma pop_macro("slots")

#include <iostream>

namespace py = pybind11;


struct no_int_cast_bool
{
    bool b;
    no_int_cast_bool(){}
    no_int_cast_bool(bool b_) : b(b_){}
    bool operator()() const { return b; }
};


namespace pybind11::detail{

    template <>
    struct type_caster<no_int_cast_bool> {
    public:
        bool load(handle src, bool convert) {
            if (!src) return false;
            else if (src.ptr() == Py_True) {  value = true; return true; }
            else if (src.ptr() == Py_False) { value = false; return true; }
            return false;
        }
        static handle cast(no_int_cast_bool src, return_value_policy /* policy */, handle /* parent */) {
            return handle(src() ? Py_True : Py_False).inc_ref();
        }
        PYBIND11_TYPE_CASTER(no_int_cast_bool, _("bool"));
    };

}
#endif //DSR_CUSTOM_BOOL_CAST_H
