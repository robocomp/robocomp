//
// Created by juancarlos on 17/12/20.
//

#ifndef PYTHON_WRAPPER_SIGNAL_FUNCTION_CASTER_H
#define PYTHON_WRAPPER_SIGNAL_FUNCTION_CASTER_H


#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

#pragma pop_macro("slots")

namespace py = pybind11;

using callback_types = std::variant<
        std::function<void(std::uint64_t, const std::string &)>,
        std::function<void(std::uint64_t, const std::vector<std::string> &)>,
        std::function<void(std::uint64_t, std::uint64_t, const std::string &)>,
        std::function<void(std::uint64_t, std::uint64_t, const std::vector<std::string> &)>,
        std::function<void(std::uint64_t)>
>;


namespace pybind11::detail {

    template<>
    struct variant_caster<callback_types>
    {
        using Type = callback_types;
        PYBIND11_TYPE_CASTER(Type, _("Union[") +
        detail::concat(
                make_caster<std::function<void(std::uint64_t, const std::string &)>>::name,
        make_caster<std::function<void(std::uint64_t,
        const std::vector<std::string> &)>>::name,
        make_caster<std::function<void(std::uint64_t, std::uint64_t,
        const std::string &)>>::name,
        make_caster<std::function<void(std::uint64_t, std::uint64_t,
        const std::vector<std::string> &)>>::name,
        make_caster<std::function<void(std::uint64_t)>>::name) + _("]"));


        template<typename U>
        bool load_alternative(handle src, bool convert) {
            auto caster = make_caster<U>();
            if (caster.load(src, convert)) {
                value = cast_op<U>(caster);
                std::cout << value.index() << " " << convert << std::endl;

                return true;
            }
            return false;
        }


        bool load(handle src, bool convert) {

            PyObject *ptr = src.ptr();
            if (!ptr) return false; //If its none we don't do anything.

            int res = PyCallable_Check(ptr);
            if (not res) return false; //If it is not callable we return false.

            PyObject *annotations;
            if (PyMethod_Check(ptr))
            {
                PyObject * fn = PyMethod_Function(ptr);
                if (not fn) return false; //This shouldn't return null, but we check it anyway.
                annotations = PyFunction_GetAnnotations(fn);

            } else
            {
                annotations = PyFunction_GetAnnotations(ptr);
            }

            if (not annotations) return false; //We need to know attribute parameters
            auto dict = reinterpret_borrow<py::dict>(py::handle(annotations));

            if (dict.size() == 1) {
                auto str = py::str(dict.begin()->second).cast<std::string>();
                if ("<class 'int'>" == str)
                {
                    return load_alternative<std::function<void(std::uint64_t)>>(src, convert);
                }
            } else if (dict.size() >= 2) {
                auto itr = dict.begin();
                if ("<class 'int'>" != py::str(itr->second).cast<std::string>()) return false;
                itr++;
                if ("[<class 'str'>]" == py::str(itr->second).cast<std::string>())
                {
                    return load_alternative<std::function<void(std::uint64_t, const std::vector<std::string> &)>>(src, convert);
                } else if ("<class 'str'>" == py::str(itr->second).cast<std::string>())
                {
                    return load_alternative<std::function<void(std::uint64_t, const std::string &)>>(src, convert);
                } else if ("<class 'int'>" == py::str(itr->second).cast<std::string>()) {
                    itr++;
                    if ("[<class 'str'>]" == py::str(itr->second).cast<std::string>())
                    {
                        return load_alternative<std::function<void(std::uint64_t, std::uint64_t, const std::vector<std::string> &)>>(src, convert);
                    } else if ("<class 'str'>" == py::str(itr->second).cast<std::string>())
                    {
                        return load_alternative<std::function<void(std::uint64_t, std::uint64_t, const std::string &)>>(src, convert);
                    }
                }
                return false;

            }
            return false;
        }

        template<typename Variant>
        static handle cast(Variant &&src, return_value_policy policy, handle parent) {
            return visit_helper<std::variant>::call(variant_caster_visitor{policy, parent},
                                                    std::forward<Variant>(src));
        }

    };
} // namespace pybind11::detail



#endif //PYTHON_WRAPPER_SIGNAL_FUNCTION_CASTER_H
