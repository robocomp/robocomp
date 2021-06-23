//
// Created by juancarlos on 20/4/21.
//

#ifndef DSR_VECTOR_CASTER_H
#define DSR_VECTOR_CASTER_H


#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/iostream.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#pragma pop_macro("slots")

#include <iostream>


namespace pybind11::detail {

    template <> struct list_caster<float, std::vector<float>::allocator_type > {
        using value_conv = make_caster<float>;

        bool load(handle src, bool convert) {
            if (!isinstance<sequence>(src) || isinstance<str>(src) || npy_api::get().PyArrayDescr_Check_(src.ptr())) {
                return false;
            }
            //std::cout <<" Casting std::vector from type "<< src.ptr()->ob_type->tp_name << " " << py::repr(src.ptr()) << std::boolalpha << convert <<  std::endl;

            auto s = reinterpret_borrow<sequence>(src);
            value.clear();
            reserve_maybe(s, &value);
            for (auto it : s) {
                value_conv conv;
                //std::cout << "\t "<<it.ptr()->ob_type->tp_name << " " << py::repr(it.ptr()) << std::endl;
                if (!conv.load(it, convert)) {
                    return false;
                }
                value.push_back(cast_op<float &&>(std::move(conv)));
            }
            return true;
        }

    private:
        template <typename T = float,
                enable_if_t<std::is_same<decltype(std::declval<T>().reserve(0)), void>::value, int> = 0>
        void reserve_maybe(sequence s, float *) { value.reserve(s.size()); }
        void reserve_maybe(sequence, void *) { }

    public:
        template <typename T>
        static handle cast(T &&src, return_value_policy policy, handle parent) {
            if (!std::is_lvalue_reference<T>::value)
                policy = return_value_policy_override<float>::policy(policy);
            list l(src.size());
            size_t index = 0;
            for (auto &&value : src) {
                auto value_ = reinterpret_steal<object>(value_conv::cast(forward_like<T>(value), policy, parent));
                if (!value_)
                    return handle();
                PyList_SET_ITEM(l.ptr(), (ssize_t) index++, value_.release().ptr()); // steals a reference
            }
            return l.release();
        }

    using vec_float = std::vector<float, std::vector<float>::allocator_type>;
    PYBIND11_TYPE_CASTER(vec_float, _("List[") + value_conv::name + _("]"));
    };

    template<>
    struct type_caster<std::vector<float, std::vector<float>::allocator_type>>
            : list_caster<float, std::vector<float>::allocator_type>
    {
    };
    //typename ArrayType, typename Value, bool Resizable, size_t Size = 0
    template <size_t size> struct array_caster<std::array<float, size>, float, false, size> {
        using value_conv = make_caster<float>;

    private:

        bool require_size(std::size_t s) {
            return size == 2;
        }

    public:
        bool load(handle src, bool convert) {
            if (!isinstance<sequence>(src))
                return false;
            //std::cout <<" Casting std::array from type "<< src.ptr()->ob_type->tp_name << " " << py::repr(src.ptr()) << std::boolalpha << convert << std::endl;
            auto l = reinterpret_borrow<sequence>(src);
            if (!require_size(l.size()))
                return false;
            size_t ctr = 0;
            for (auto it : l) {
                value_conv conv;
                if (!conv.load(it, convert)) {
                    //std::cout << "\t "<<it.ptr()->ob_type->tp_name << " " << py::repr(it.ptr()) << std::endl;
                    return false;
                }
                value[ctr++] = cast_op<float &&>(std::move(conv));
            }
            return true;
        }

        template <typename T>
        static handle cast(T &&src, return_value_policy policy, handle parent) {
            list l(src.size());
            size_t index = 0;
            for (auto &&value : src) {
                auto value_ = reinterpret_steal<object>(value_conv::cast(forward_like<T>(value), policy, parent));
                if (!value_)
                    return handle();
                PyList_SET_ITEM(l.ptr(), (ssize_t) index++, value_.release().ptr()); // steals a reference
            }
            return l.release();
        }

        using vec = std::array<float, size>;
        PYBIND11_TYPE_CASTER(vec, _("List[") + value_conv::name + _<false>(_(""), _("[") + _<size>() + _("]")) + _("]"));
    };

    template <> struct type_caster<std::array<float, 2>>
            : array_caster<std::array<float, 2>, float, false, 2> { };

    template <> struct type_caster<std::array<float, 3>>
            : array_caster<std::array<float, 3>, float, false, 3> { };

    template <> struct type_caster<std::array<float, 4>>
            : array_caster<std::array<float, 4>, float, false, 4> { };

    template <> struct type_caster<std::array<float, 6>>
            : array_caster<std::array<float, 6>, float, false, 6> { };





    template <int ExtraFlags>
    struct pyobject_caster<array_t<float, ExtraFlags>> {
        using type = array_t<float, ExtraFlags>;

        bool load(handle src, bool convert) {
            if (!convert && !type::check_(src))
                return false;
            if (std::string_view (src.ptr()->ob_type->tp_name) == "int") {
                return false;
            }
            auto npy = npy_api::get();
            //std::cout <<" Casting numpy.array(float) from type "<< src.ptr()->ob_type->tp_name << " " << py::repr(src.ptr()) <<" "<< std::boolalpha << npy.PyArray_Check_(src.ptr())<<" " << convert <<  std::endl;
            if (npy.PyArray_Check_(src.ptr()) &&  npy.PyArray_EquivTypes_( detail::array_proxy(src.ptr())->descr, dtype::of<float>().ptr())) {
                value = type::ensure(src);
                return static_cast<bool>(value);
            } else {
                //std::cout << "Invalid type casting to numpy.array(float)" << std::endl;
                return false;
            }
        }

        static handle cast(const handle &src, return_value_policy /* policy */, handle /* parent */) {
            return src.inc_ref();
        }

        PYBIND11_TYPE_CASTER(type, handle_type_name<type>::name);
    };

    template <int ExtraFlags>
    struct pyobject_caster<array_t<uint8_t , ExtraFlags>> {
        using type = array_t<uint8_t, ExtraFlags>;

        bool load(handle src, bool convert) {
            if (!convert && !type::check_(src))
                return false;
            if (std::string_view (src.ptr()->ob_type->tp_name) == "int") {
                return false;
            }
            auto npy = npy_api::get();
            //std::cout <<" Casting numpy.array(uint8_t) from type "<< src.ptr()->ob_type->tp_name << " " << py::repr(src.ptr()) <<" "<< std::boolalpha <<npy.PyArray_Check_(src.ptr())<<" " << convert<< std::endl;
            if (npy.PyArray_Check_(src.ptr()) &&  npy.PyArray_EquivTypes_( detail::array_proxy(src.ptr())->descr, dtype::of<uint8_t>().ptr())) {
                value = type::ensure(src);
                return static_cast<bool>(value);
            } else {
                //std::cout << "Invalid type casting to numpy.array(uint8_t)" << std::endl;
                return false;
            }
        }

        static handle cast(const handle &src, return_value_policy /* policy */, handle /* parent */) {
            return src.inc_ref();
        }

    PYBIND11_TYPE_CASTER(type, handle_type_name<type>::name);
    };

    template <int ExtraFlags>
    struct pyobject_caster<array_t<uint64_t , ExtraFlags>> {
        using type = array_t<uint64_t, ExtraFlags>;

        bool load(handle src, bool convert) {
            if (!convert && !type::check_(src))
                return false;
            if (std::string_view (src.ptr()->ob_type->tp_name) == "int") {
                return false;
            }
            auto npy = npy_api::get();
            //std::cout <<" Casting numpy.array(uint64_t) from type "<< src.ptr()->ob_type->tp_name << " " << py::repr(src.ptr()) <<" " << std::boolalpha <<npy.PyArray_Check_(src.ptr()) <<" " << convert<< std::endl;
            if (npy.PyArray_Check_(src.ptr()) &&  npy.PyArray_EquivTypes_( detail::array_proxy(src.ptr())->descr, dtype::of<uint64_t>().ptr())) {
                value = type::ensure(src);
                return static_cast<bool>(value);
            } else {
                //std::cout << "Invalid type casting to numpy.array(uint64_t)" << std::endl;
                return false;
            }
        }

        static handle cast(const handle &src, return_value_policy /* policy */, handle /* parent */) {
            return src.inc_ref();
        }

    PYBIND11_TYPE_CASTER(type, handle_type_name<type>::name);
    };
}
#endif //DSR_VECTOR_CASTER_H
