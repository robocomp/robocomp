//
// Created by juancarlos on 30/11/21.
//

#ifndef DSR_PYGHISTORY_H
#define DSR_PYGHISTORY_H

#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

#pragma pop_macro("slots")

#include <dsr/api/GHistorySaver.h>

namespace pybind11 { namespace detail {
template <> struct type_caster<std::monostate> {
public:
    /**
     * This macro establishes the name 'std::monostate' in
     * function signatures and declares a local variable
     * 'value' of type None
     */
    PYBIND11_TYPE_CASTER(std::monostate, _("monostate"));

    /**
     * Conversion part 1 (Python->C++): convert a PyObject into a inty
     * instance or return false upon failure. The second argument
     * indicates whether implicit conversions should be applied.
     */
    bool load(handle src, bool) {
        /* Extract PyObject from handle */
        PyObject *source = src.ptr();
        /* Try converting into a Python integer value */
        PyObject *tmp = PyNumber_Long(source);
        if (!tmp)
            return false;
        /* Now try to convert into a C++ int */
        value = {};
        Py_DECREF(tmp);
        /* Ensure return code was OK (to avoid out-of-range errors etc) */
        return !(!PyErr_Occurred());
    }

    /**
     * Conversion part 2 (C++ -> Python): convert an inty instance into
     * a Python object. The second and third arguments are used to
     * indicate the return value policy and parent object (for
     * ``return_value_policy::reference_internal``) and are generally
     * ignored by implicit casters.
     */
    static handle cast(std::monostate src, return_value_policy /* policy */, handle /* parent */) {
        return Py_None;
    }
};
}} // namespace pybind11::detail

void bind_ghistory(py::module &m) {


    py::enum_<ChangeInfo::OPER>(m, "Oper");

    py::class_<ChangeInfo>(m, "ChangeInfo")
            .def_property_readonly("op", [](ChangeInfo const &self) -> ChangeInfo::OPER {return self.op; })
            .def_property_readonly("agent_id", [](ChangeInfo const &self) {return self.agent_id; })
            .def_property_readonly("node_or_from_id", [](ChangeInfo const &self) {return self.node_or_from_id; })
            .def_property_readonly("maybe_to_id", [](ChangeInfo const &self) {return self.maybe_to_id; })
            .def_property_readonly("timestamp", [](ChangeInfo const &self) {return self.timestamp; })
            .def_property_readonly("maybe_edge_type", [](ChangeInfo const &self) {return self.maybe_edge_type; });

    py::class_<GSerializer>(m, "GHistory")
            .def_static("read_file", GSerializer::read_file)
            .def("initialize", &GSerializer::initialize)
            .def("save_file", &GSerializer::save_file);

}
#endif //DSR_PYGHISTORY_H
