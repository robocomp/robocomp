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
