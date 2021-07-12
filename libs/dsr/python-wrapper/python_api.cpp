//
// Created by juancarlos on 18/11/20.
//

#include "dsr/api/dsr_api.h"

using namespace DSR;

#include "include/silent_output.h"
#include "include/signal_function_caster.h"
#include "include/custom_bind_map.h"
#include "include/custom_bool_cast.h"
#include "include/custom_vector_cast.h"

#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#pragma pop_macro("slots")

#include <utility>

#include <memory>
#include <functional>



namespace py = pybind11;

using namespace py::literals;
//using namespace RoboCompDSRGetID;

using callback_types = std::variant<
        std::function<void(std::uint64_t, const std::string &)>,
        std::function<void(std::uint64_t, const std::vector<std::string> &)>,
        std::function<void(std::uint64_t, std::uint64_t, const std::string &)>,
        std::function<void(std::uint64_t, std::uint64_t, const std::string &, const std::vector<std::string> &)>,
        std::function<void(std::uint64_t)>
>;

enum ATT_ENUM: uint16_t {
    STRING_PY,
    BOOL_PY,
    NPYU8,
    NPYF,
    NPYU64,
    VECU8_PY,
    VECF_PY,
    VECU64_PY,
    VEC2_PY,
    VEC3_PY,
    VEC4_PY,
    VEC6_PY,
    U64_PY,
    DOUBLE_PY,
    FLOAT_PY,
    I32_PY,
    U32_PY
};

using attribute_type = std::variant<std::string,
                                    no_int_cast_bool,
                                    py::array_t<uint8_t>,
                                    py::array_t<float>,
                                    py::array_t<uint64_t>,
                                    std::vector<uint8_t>,
                                    std::vector<float>,
                                    std::vector<uint64_t>,
                                    std::array<float, 2>,
                                    std::array<float, 3>,
                                    std::array<float, 4>,
                                    std::array<float, 6>,
                                    uint64_t,
                                    double,
                                    float,
                                    int32_t,
                                    uint32_t>;




template<std::size_t idx, typename T>
ValType convert_variant_fn(const attribute_type & e)
{
    //std::cout << "[PYTHON_VARIANT -> VALTYPE] " << attribute_type_TYPENAMES_UNION[e.index()] << std::endl;
    ValType vout;
    if constexpr (std::is_same_v<T, py::array_t<uint8_t>>)
    {
        auto tmp = std::get<py::array_t<uint8_t>>(e);
        const auto size = tmp.size();
        py::buffer_info x = tmp.request();
        vout.emplace<idx>(std::vector<uint8_t>{static_cast<uint8_t *>(x.ptr), static_cast<uint8_t *>(x.ptr) + size});
    } else if constexpr (std::is_same_v<T, py::array_t<float>>)
    {
        auto tmp = std::get<py::array_t<float>>(e);
        //std::cout << tmp.dtype() << " " << tmp.size() << py::array_t<float>::ensure(tmp) << std::endl;
        const auto size = tmp.size();
        py::buffer_info x = tmp.request();
        vout.emplace<idx>(std::vector<float>{static_cast<float *>(x.ptr), static_cast<float *>(x.ptr) + size});
    } else if constexpr (std::is_same_v<T, py::array_t<uint64_t>>)
    {
        auto tmp = std::get<py::array_t<uint64_t>>(e);
        const auto size = tmp.size();
        py::buffer_info x = tmp.request();
        vout.emplace<idx>(std::vector<uint64_t>{static_cast<uint64_t *>(x.ptr), static_cast<uint64_t *>(x.ptr) + size});
    } else if constexpr (std::is_same_v<T, no_int_cast_bool>)
    {
        auto tmp = std::get<no_int_cast_bool>(e)();
        vout.emplace<idx>(tmp);
    }
    else
    {
        auto x = std::get<T>(e);
        vout.emplace<idx>(x);
    }

    return vout;
};



ValType convert_variant(const attribute_type & e)
{
    typedef ValType (*conver_fn) (const attribute_type &);
    constexpr std::array<conver_fn, 17> cast = { convert_variant_fn<0, std::string>,
                                                 convert_variant_fn<4, no_int_cast_bool>,
                                                 convert_variant_fn<5, py::array_t<uint8_t>>,
                                                 convert_variant_fn<3, py::array_t<float>>,
                                                 convert_variant_fn<9, py::array_t<uint64_t>>,
                                                 convert_variant_fn<5, std::vector<uint8_t>>,
                                                 convert_variant_fn<3, std::vector<float>>,
                                                 convert_variant_fn<9, std::vector<uint64_t>>,
                                                 convert_variant_fn<10, std::array<float, 2>>,
                                                 convert_variant_fn<11, std::array<float, 3>>,
                                                 convert_variant_fn<12, std::array<float, 4>>,
                                                 convert_variant_fn<13, std::array<float, 6>>,
                                                 convert_variant_fn<7, uint64_t>,
                                                 convert_variant_fn<8, double>,
                                                 convert_variant_fn<2, float>,
                                                 convert_variant_fn<1, int32_t>,
                                                 convert_variant_fn<6, uint32_t>
                                             };

    const auto idx = e.index(); //idx_ValType.at(e.index());
    return cast[idx](e);
}

PYBIND11_MAKE_OPAQUE(std::map<std::pair<uint64_t, std::string>, Edge>)
PYBIND11_MAKE_OPAQUE(std::map<std::string, Attribute>)

PYBIND11_MODULE(pydsr, m) {
    py::bind_map<std::map<std::pair<uint64_t, std::string>, Edge>>(m, "MapStringEdge");
    py::bind_dsr_map<std::map<std::string, Attribute>>(m, "MapStringAttribute");

    m.doc() = "DSR Api for python";

    uint64_t local_agent_id = -1;


    //Disable messages from Qt.
    qInstallMessageHandler([](QtMsgType type, const QMessageLogContext &context, const QString &msg) {
        if (type == QtCriticalMsg || type == QtFatalMsg) {
            fprintf(stderr, "%s", msg.toStdString().c_str());
        }
    });

    //Disable cout
    //m.attr("redirect_output") = py::capsule(new scoped_ostream_discard(),
    //                                        [](void *sor) { delete static_cast<scoped_ostream_discard *>(sor); });


    auto sig = m.def_submodule("signals",
    R""""(
    Connect functions to DSR signals. The types of the signals are defined in the "signal_type enum.

    In order to connect the signals you must annotate the types of function parameters to match those of the signals.
    The function signatures are the following:

    UPDATE_NODE: [[int, str], None]

    UPDATE_NODE_ATTR: [[int, [str]], None]

    UPDATE_EDGE: [[int, int, str], None]

    UPDATE_EDGE_ATTR: [[int, int, [str]], None]

    DELETE_EDGE: [[int, int, str], None]

    DELETE_NODE: [[int], None]


    ")"""");

    enum signal_type
    {
        UPDATE_NODE,
        UPDATE_NODE_ATTR,
        UPDATE_EDGE,
        UPDATE_EDGE_ATTR,
        DELETE_EDGE,
        DELETE_NODE
    };


    py::enum_<signal_type>(sig, "signal_type")
            .value("UPDATE_NODE", UPDATE_NODE)
            .value("UPDATE_NODE_ATTR", UPDATE_NODE_ATTR)
            .value("UPDATE_EDGE", UPDATE_EDGE)
            .value("UPDATE_EDGE_ATTR", UPDATE_EDGE_ATTR)
            .value("DELETE_EDGE", DELETE_EDGE)
            .value("DELETE_NODE", DELETE_NODE)
            .export_values();


    sig.def("connect", [](DSRGraph *G, signal_type type, callback_types fn_callback) {

        switch (type) {
            case UPDATE_NODE:
                try {
                    QObject::connect(G, &DSR::DSRGraph::update_node_signal,
                                     std::get<std::function<void(std::uint64_t, const std::string &)>>(fn_callback));

                } catch (std::exception &e) {
                    std::cout << "Update Node Callback must be (int, str)\n "  << std::endl;
                    throw e;
                }
                break;
            case UPDATE_NODE_ATTR:
                try {
                    QObject::connect(G, &DSR::DSRGraph::update_node_attr_signal,
                                     std::get<std::function<void(std::uint64_t, const std::vector<std::string> &)>>(
                                                 fn_callback));

                } catch (std::exception &e) {
                    std::cout << "Update Node Attribute Callback must be (int, [str])\n "  << std::endl;
                    throw e;
                }
                break;
            case UPDATE_EDGE:
                try {
                    QObject::connect(G, &DSR::DSRGraph::update_edge_signal,
                                     std::get<std::function<void(std::uint64_t, std::uint64_t, const std::string &)>>(
                                             fn_callback));
                } catch (std::exception &e) {
                    std::cout << "Update Edge Callback must be (int, int, str)\n "  << std::endl;
                    throw e;
                }
                break;
            case UPDATE_EDGE_ATTR:
                try {
                    QObject::connect(G, &DSR::DSRGraph::update_edge_attr_signal,
                                     std::get<std::function<void(std::uint64_t, std::uint64_t, const std::string&,
                                                                 const std::vector<std::string> &)>>(fn_callback));
                } catch (std::exception &e) {
                    std::cout << "Update Edge Attribute Callback must be (int, int, str, [str])\n " << std::endl;
                    throw e;
                }
                break;
            case DELETE_EDGE:
                try {
                    QObject::connect(G, &DSR::DSRGraph::del_edge_signal,
                                     std::get<std::function<void(std::uint64_t, std::uint64_t, const std::string &)>>(
                                             fn_callback));
                } catch (std::exception &e) {
                    std::cout << "Delete Edge Callback must be (int, int, str)\n "  << std::endl;
                    throw e;
                }
                break;
            case DELETE_NODE:
                try {
                    QObject::connect(G, &DSR::DSRGraph::del_node_signal,
                                     std::get<std::function<void(std::uint64_t)>>(fn_callback));
                } catch (std::exception &e) {
                    std::cout << "Delete Node Callback must be (int)\n "  << std::endl;
                    throw e;
                }
                break;
            default:
                throw std::logic_error("Invalid signal type");
        }
    });

    //DSR Attribute class
    py::class_<Attribute>(m, "Attribute")
            .def(py::init([&](attribute_type const& v, uint64_t t, uint32_t agent_id){
                    //Use another variant type to avoid problems with implicit conversions.
                    return Attribute(convert_variant(v), t, agent_id);
                }),
                 "value"_a, "timestamp"_a, "agent_id"_a)
            .def(py::init([&](attribute_type const& v , uint32_t agent_id) {
                //Comprobar tipos en ValType. Como se convien los arrays de numpy, las listas, los doubles, etc.

                return Attribute(convert_variant(v), get_unix_timestamp(), agent_id);
            }),"value"_a, "agent_id"_a)
            .def("__repr__", [](Attribute const &self) {

                std::stringstream out;
                out << " < ";
                out << self;
                out << " >";
                return out.str();
            })
            .def_property_readonly("agent_id", [](Attribute &self) { return self.agent_id(); },
                                   "read the agent_id attribute. This property is readonly ans it is updated when a change is made in the value property.")
            .def_property_readonly("timestamp", [](Attribute &self) { return self.timestamp(); },
                                   "read the timestamp (ns) attribute. This property is readonly and it is updated when a change is made in the value property.")
            .def_property("value", [](Attribute &self) -> attribute_type {

                              switch (self.selected()) {
                                  case 0:
                                      return self.str();
                                  case 1:
                                      return self.dec();
                                  case 2:
                                      return self.fl();
                                  case 3:
                                      return py::array_t<float> { static_cast<ssize_t>(self.float_vec().size()), self.float_vec().data()};
                                  case 4:
                                      return self.bl();
                                  case 5:
                                      return py::array_t<uint8_t> {static_cast<ssize_t>(self.byte_vec().size()), self.byte_vec().data()};
                                  case 6:
                                      return self.uint();
                                  case 7:
                                      return self.uint64();
                                  case 8:
                                      return self.dob();
                                  case 9:
                                      return py::array_t<uint64_t> {static_cast<ssize_t>(self.u64_vec().size()), self.u64_vec().data()};
                                  case 10:
                                      return py::array_t<float> { 2, self.vec2().data()};
                                  case 11:
                                      return py::array_t<float> { 3, self.vec3().data()};
                                  case 12:
                                      return py::array_t<float> { 4, self.vec4().data()};
                                  case 13:
                                      return py::array_t<float> { 6, self.vec6().data()};
                                  default:
                                      throw /*std::runtime_error*/ pybind11::type_error("Unreachable");
                              }

                },
                          [&](Attribute &self, const attribute_type &val) {
                              auto excep = std::string("Attributes cannot change type. Selected type is " + std::string{DSR::TYPENAMES_UNION[self.selected()]} + " and used type is " + std::string{attribute_type_TYPENAMES_UNION[val.index()]});
                              try {
                                  //std::cout << "[SET MAP VALUE] " << self.selected() << ", " << val.index() << std::endl;
                                  switch (self.selected()) {
                                      case 0:
                                          self.str(std::get<std::string>(val));
                                          break;
                                      case 1:
                                          if (val.index() == ATT_ENUM::U64_PY) self.dec(std::get<uint64_t>(val)); //This is intentional
                                          else self.dec(std::get<int32_t>(val));
                                          break;
                                      case 2:
                                          if (val.index() == ATT_ENUM::DOUBLE_PY) self.fl(std::get<double>(val)); //This is intentional
                                          else self.fl(std::get<float>(val));
                                          break;
                                      case 3:
                                          if (val.index() == ATT_ENUM::NPYF) {
                                              auto tmp = std::get<py::array_t<float>>(val);
                                              const auto size = tmp.size();
                                              py::buffer_info x = tmp.request();
                                              self.float_vec(std::vector<float>{static_cast<float *>(x.ptr), static_cast<float *>(x.ptr) + size});
                                          }
                                          else self.float_vec(std::get<std::vector<float>>(val));
                                          break;
                                      case 4:
                                          self.bl(std::get<no_int_cast_bool>(val)());
                                          break;
                                      case 5:
                                          if (val.index() == ATT_ENUM::NPYU8) {
                                              auto tmp = std::get<py::array_t<uint8_t>>(val);
                                              const auto size = tmp.size();
                                              py::buffer_info x = tmp.request();
                                              self.byte_vec(std::vector<uint8_t>{static_cast<uint8_t *>(x.ptr), static_cast<uint8_t *>(x.ptr) + size});
                                          }
                                          else self.byte_vec(std::get<std::vector<uint8_t>>(val));
                                          break;
                                      case 6:
                                          if (val.index() == ATT_ENUM::U64_PY) self.uint(std::get<uint64_t>(val));
                                          else self.uint(std::get<uint32_t>(val));
                                          break;
                                      case 7:
                                          self.uint64(std::get<uint64_t>(val));
                                          break;
                                      case 8:
                                          self.dob(std::get<double>(val));
                                          break;
                                      case 9:
                                          if (val.index() == ATT_ENUM::NPYU64) {
                                              auto tmp = std::get<py::array_t<uint64_t>>(val);
                                              const auto size = tmp.size();
                                              py::buffer_info x = tmp.request();
                                              self.u64_vec(std::vector<uint64_t>{static_cast<uint64_t *>(x.ptr), static_cast<uint64_t *>(x.ptr) + size});
                                          } else if (val.index() == ATT_ENUM::VECU8_PY) {
                                              auto tmp = std::get<std::vector<uint8_t>>(val);
                                              self.u64_vec(std::vector<uint64_t>{tmp.begin(), tmp.end()});
                                          }
                                          else self.u64_vec(std::get<std::vector<uint64_t>>(val));
                                          break;
                                      case 10:
                                          if (val.index() == ATT_ENUM::NPYF) {
                                              auto tmp = std::get<py::array_t<float>>(val);
                                              if (tmp.size() == 2) {
                                                  py::buffer_info x = tmp.request();
                                                  self.vec2(std::array<float, 2>{static_cast<float *>(x.ptr)[0],
                                                                                 static_cast<float *>(x.ptr)[1]});

                                              } else throw;
                                          }else if (val.index() == ATT_ENUM::VECF_PY) {
                                              auto tmp = std::get<std::vector<float>>(val);
                                              if (tmp.size() == 2) {
                                                  self.vec2(std::array<float, 2>{tmp[0], tmp[1]});
                                              } else throw pybind11::type_error(excep);
                                          }
                                          else self.vec2(std::get<std::array<float, 2>>(val));
                                          break;
                                      case 11:
                                          if (val.index() == ATT_ENUM::NPYF) {
                                              auto tmp = std::get<py::array_t<float>>(val);
                                              if (tmp.size() == 3) {
                                                  py::buffer_info x = tmp.request();
                                                  self.vec3(std::array<float, 3>{static_cast<float *>(x.ptr)[0],
                                                                                 static_cast<float *>(x.ptr)[1],
                                                                                 static_cast<float *>(x.ptr)[2]});

                                              } else throw pybind11::type_error(excep);
                                          } else if (val.index() == ATT_ENUM::VECF_PY) {
                                              auto tmp = std::get<std::vector<float>>(val);
                                              if (tmp.size() == 3) {
                                                  self.vec3(std::array<float, 3>{tmp[0],
                                                                                 tmp[1],
                                                                                 tmp[2]});
                                              } else throw pybind11::type_error(excep);

                                          }
                                          else self.vec3(std::get<std::array<float, 3>>(val));
                                          break;
                                      case 12:
                                          if (val.index() == ATT_ENUM::NPYF) {
                                              auto tmp = std::get<py::array_t<float>>(val);
                                              if (tmp.size() == 4) {
                                                  py::buffer_info x = tmp.request();
                                                  self.vec4(std::array<float, 4>{static_cast<float *>(x.ptr)[0],
                                                                                 static_cast<float *>(x.ptr)[1],
                                                                                 static_cast<float *>(x.ptr)[2],
                                                                                 static_cast<float *>(x.ptr)[3]});

                                              } else throw pybind11::type_error(excep);
                                          }else if (val.index() == ATT_ENUM::VECF_PY) {
                                              auto tmp = std::get<std::vector<float>>(val);
                                              if (tmp.size() == 4) {
                                                  self.vec4(std::array<float, 4>{tmp[0],
                                                                                 tmp[1],
                                                                                 tmp[2],
                                                                                 tmp[3]});
                                              } else throw pybind11::type_error(excep);
                                          }
                                          else self.vec4(std::get<std::array<float, 4>>(val));
                                          break;
                                      case 13:
                                          if (val.index() == ATT_ENUM::NPYF) {
                                              auto tmp = std::get<py::array_t<float>>(val);
                                              if (tmp.size() == 6) {
                                                  py::buffer_info x = tmp.request();
                                                  self.vec6(std::array<float, 6>{static_cast<float *>(x.ptr)[0],
                                                                                 static_cast<float *>(x.ptr)[1],
                                                                                 static_cast<float *>(x.ptr)[2],
                                                                                 static_cast<float *>(x.ptr)[3],
                                                                                 static_cast<float *>(x.ptr)[4],
                                                                                 static_cast<float *>(x.ptr)[5]});

                                              } else throw pybind11::type_error(excep);
                                          } else if (val.index() == ATT_ENUM::VECF_PY) {
                                              auto tmp = std::get<std::vector<float>>(val);
                                              if (tmp.size() == 6) {
                                                  self.vec6(std::array<float, 6>{tmp[0],
                                                                                 tmp[1],
                                                                                 tmp[2],
                                                                                 tmp[3],
                                                                                 tmp[4],
                                                                                 tmp[5]});
                                              } else throw pybind11::type_error(excep);
                                          }
                                          else self.vec6(std::get<std::array<float, 6>>(val));
                                          break;
                                      default:
                                          throw /*std::runtime_error*/ pybind11::type_error(excep);
                                      };
                                      self.timestamp(get_unix_timestamp());
                                      self.agent_id(local_agent_id);
                              } catch (...) {
                                  throw /*std::runtime_error*/ pybind11::type_error(excep);
                              }
                          },
                          py::return_value_policy::reference, "read or assign a new value to the Attribute object.");

    //DSR Edge class
    py::class_<Edge>(m, "Edge")
            .def(py::init<uint64_t, uint64_t, std::string, uint32_t>(),
                 "to"_a, "from"_a, "type"_a, "agent_id"_a)
            .def("__repr__", [](Edge const &self) {
                std::stringstream out;
                out << "------------------------------------" << std::endl;
                out << "Type: " << self.type() << " from: " << self.from() << " to: " << self.to() << std::endl;
                for (auto& [k, v] : self.attrs())
                    out << "      [" << k << "] -- " << v << std::endl;
                out << "------------------------------------" << std::endl;
                return out.str();
            })
            .def_property_readonly("type", [](Edge const &self) { return self.type(); },
                                   "read the type of the edge. This property is readonly")
            .def_property_readonly("origin", [](Edge const &self) { return self.from(); },
                                   "read the origen node of the edge. This property is readonly")
            .def_property_readonly("destination", [](Edge const &self) { return self.to(); },
                                   "read the destination node of the edge. This property is readonly")
            .def_property("agent_id", [](Edge const &self) { return self.agent_id(); },
                          [](Edge &self, uint32_t val) { self.agent_id(val); },
                          "read or assign a new value to the agent_id attribute.")

            .def_property("attrs", [](Edge &self) -> std::map<std::string, Attribute> & { return self.attrs(); },
                          [](Edge &self, const std::map<std::string, Attribute> &at) { self.attrs(at); },
                          py::return_value_policy::reference, "read or write in the attribute map of the edge.");



    //DSR Node class
    py::class_<Node>(m, "Node")
            .def(py::init([](uint32_t agent_id, const std::string &type,
                             const std::string &name = "") -> std::unique_ptr<Node> {
                auto tmp = std::make_unique<Node>(agent_id, type);
                tmp->name(name);
                return tmp;
            }), "agent_id"_a, "type"_a, "name"_a = "")
            .def("__repr__", [](Node const &self) {
                std::stringstream out;

                out << "------------------------------------" << std::endl;
                out << "Node: " << self.id() << std::endl;
                out << "  Type: " << self.type() << std::endl;
                out << "  Name: " << self.name() << std::endl;
                out << "  Agent id: " << self.agent_id() << std::endl;
                out << "  ATTRIBUTES: [\n";
                for (auto& [key, val] : self.attrs())
                    out << "      [" << key << "] -- " << val << std::endl;
                out << "   ]" << std::endl;
                out << "  EDGES: [" << std::endl;
                for (auto& [key, val] : self.fano()) {
                    out << "          Edge type: " << val.type() << " from: " << val.from() << " to: " << val.to()
                        << std::endl;
                    for (auto& [k, v] : val.attrs())
                        out << "              [" << k << "] -- " << v << std::endl;
                }
                out << "          ]" << std::endl;
                out << "------------------------------------" << std::endl;

                return out.str();
            })
            .def_property_readonly("id", [](Node const &self) { return self.id(); },
                                   "read the id of the node. This property is readonly and is generated by the idserver agent when the node is inserted")
            .def_property_readonly("name", [](Node const &self) { return self.name(); },
                                   "read the name of the node. This property is readonly. If the name is not provided to the constructor or the name already exist in G, the name is generated by the idserver agent with a combination of the type and the id when the node is inserted")
            .def_property_readonly("type", [](Node const &self) { return self.type(); },
                                   "read the type of the node. This property is readonly")
            .def_property("agent_id", [](Node const &self) { return self.agent_id(); },
                          [](Node &self, uint32_t val) { self.agent_id(val); },
                          "read or assign a new value to the agent_id attribute.")
            .def_property("attrs", [](Node &self) -> std::map<std::string, Attribute> & { return self.attrs(); },
                          [](Node &self, const std::map<std::string, Attribute> &at) { self.attrs(at); },
                          py::return_value_policy::reference, "read or write in the attribute map of the node.")
            .def_property("edges",
                          [](Node &self) -> std::map<std::pair<uint64_t, std::string>, Edge> & { return self.fano(); },
                          [](Node &self, const std::map<std::pair<uint64_t, std::string>, Edge> &edges) {
                              return self.fano(edges);
                          },
                          py::return_value_policy::reference, "read or write in the edge map of the node.");



    //DSR DSRGraph class
    py::class_<DSRGraph>(m, "DSRGraph")
            .def(py::init([&](int root, const std::string &name, int id,
                              const std::string &dsr_input_file = "",
                              bool all_same_host = true) -> std::unique_ptr<DSRGraph> {

                     //py::gil_scoped_release release;
                     local_agent_id = id;

                     auto g = std::make_unique<DSRGraph>(root, name, id, dsr_input_file, all_same_host);

                     return g;
                 }), "root"_a, "name"_a, "id"_a, "dsr_input_file"_a = "",
                 "all_same_host"_a = true, py::call_guard<py::gil_scoped_release>())

            .def("get_node", [](DSRGraph &self, uint64_t id) -> std::optional<Node> {
                return self.get_node(id);
            }, "id"_a, "return the node with the id passed as parameter. Returns None if the node does not exist.")
            .def("get_node", [](DSRGraph &self, const std::string &name) -> std::optional<Node> {
                return self.get_node(name);
            }, "name"_a, "return the node with the name passed as parameter. Returns None if the node does not exist.")
            .def("delete_node", static_cast<bool (DSRGraph::*)(uint64_t)>(&DSRGraph::delete_node), "id"_a,
                 "delete the node with the given id. Returns a bool with the result o the operation.")
            .def("delete_node",
                 static_cast<bool (DSRGraph::*)(const std::basic_string<char> &)>(&DSRGraph::delete_node), "name"_a,
                 "delete the node with the given name. Returns a bool with the result o the operation.")
            .def("insert_node", [](DSRGraph &g, Node &n) -> std::optional<uint64_t> {
                     return g.insert_node(n);
                 }, "node"_a,
                 "Insert in the graph the new node passed as parameter. Returns the id of the node or None if the Node alredy exist in the map.")
            .def("update_node", &DSRGraph::update_node<DSR::Node&>, "node"_a, "Update the node in the graph. Returns a bool.")
            .def("get_edge", [](DSRGraph &self, const std::string &from, const std::string &to,
                                const std::string &key) -> std::optional<Edge> {
                     return self.get_edge(from, to, key);
                 }, "from"_a, "to"_a, "type"_a,
                 "Return the edge with the parameters from, to, and type passed as parameter. If the edge does not exist it return None")
            .def("get_edge",
                 [](DSRGraph &self, uint64_t from, uint64_t to, const std::string &key) -> std::optional<Edge> {
                     return self.get_edge(from, to, key);
                 }, "from"_a, "to"_a, "type"_a,
                 "Return the edge with the parameters from, to, and type passed as parameter.  If the edge does not exist it return None")
            .def("insert_or_assign_edge", &DSRGraph::insert_or_assign_edge<DSR::Edge&>, "edge"_a,
                 "Insert or updates and edge. returns a bool")
            .def("delete_edge", static_cast<bool (DSRGraph::*)(uint64_t, uint64_t,
                                                               const std::basic_string<char> &)>(&DSRGraph::delete_edge),
                 "from"_a, "to"_a, "type"_a, "Removes the edge and returns a bool")
            .def("delete_edge",
                 static_cast<bool (DSRGraph::*)(const std::basic_string<char> &, const std::basic_string<char> &,
                                                const std::basic_string<char> &)>(&DSRGraph::delete_edge), "from"_a,
                 "to"_a, "type"_a, "Removes the edge and returns a bool")

            .def("get_node_root", &DSRGraph::get_node_root, "Return the root node.")
            .def("get_nodes_by_type", &DSRGraph::get_nodes_by_type, "type"_a, "Return all the nodes with a given type.")
            .def("get_name_from_id", &DSRGraph::get_name_from_id, "id"_a, "Return the name of a node given its id")
            .def("get_id_from_name", &DSRGraph::get_id_from_name, "name"_a, "Return the id from a node given its name")
            .def("get_edges_by_type", &DSRGraph::get_edges_by_type, "type"_a, "Return all the edges with a given type.")
            .def("get_edges_to_id", &DSRGraph::get_edges_to_id, "id"_a, "Return all the edges that point to the node");
    //DSR RT_API class
    py::class_<RT_API>(m, "rt_api")
            .def(py::init([](DSRGraph &g) -> std::unique_ptr<RT_API> {
                return g.get_rt_api();
            }))
            .def("insert_or_assign_edge_RT", [](RT_API &self, Node &n, uint64_t to,
                                                const std::vector<float> &translation,
                                                const std::vector<float> &rotation_euler
            ) {
                self.insert_or_assign_edge_RT(n, to, translation, rotation_euler);
            }, "node"_a, "to"_a, "trans"_a, "rot_euler"_a)
            .def("get_edge_RT", &RT_API::get_edge_RT, "node"_a, "to"_a)
            .def("get_RT_pose_from_parent", [](RT_API &self, Node &e) -> std::optional<Eigen::Matrix<double, 4, 4>> {
                auto tmp = self.get_RT_pose_from_parent(e);
                if (tmp.has_value()) {
                    Eigen::Matrix<double, 4, 4> Trans;
                    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
                    Trans.block<3, 3>(0, 0) = tmp.value().rotation();
                    Trans.block<3, 1>(0, 3) = tmp.value().translation();
                    return Trans;
                } else {
                    return std::nullopt;
                }
            }, "node"_a)
            .def("get_edge_RT_as_rtmat", [](RT_API &self, Edge &e, std::uint64_t t) -> std::optional<Eigen::Matrix<double, 4, 4>> {
                auto tmp = self.get_edge_RT_as_rtmat(e, t);
                if (tmp.has_value()) {
                    Eigen::Matrix<double, 4, 4> Trans;
                    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
                    Trans.block<3, 3>(0, 0) = tmp.value().rotation();
                    Trans.block<3, 1>(0, 3) = tmp.value().translation();
                    return Trans;
                } else {
                    return std::nullopt;
                }
            }, "edge"_a, "timestamp"_a=0)
            .def("get_translation", static_cast<std::optional<Eigen::Vector3d> (RT_API::*)(std::uint64_t,
                                                                                           std::uint64_t,
                                                                                           std::uint64_t timestamp)>(&RT_API::get_translation),
                 "node_id"_a, "to"_a, "timestamp"_a=0);

    py::class_<InnerEigenAPI>(m, "inner_api")
            .def(py::init([](DSRGraph &g) -> std::unique_ptr<InnerEigenAPI> {
                return g.get_inner_eigen_api();
            }), "graph"_a)
            .def("transform", static_cast<std::optional<Eigen::Vector3d> (InnerEigenAPI::*)(const std::string &,
                                                                                            const std::string &,
                                                                                            std::uint64_t timestamp)>(&InnerEigenAPI::transform),
                 "orig"_a, "dest"_a, "timestamp"_a=0)
            .def("transform", static_cast<std::optional<Eigen::Vector3d> (InnerEigenAPI::*)(const std::string &,
                                                                                            const Mat::Vector3d &,
                                                                                            const std::string &,
                                                                                            std::uint64_t timestamp)>(&InnerEigenAPI::transform),
                 "orig"_a, "vector"_a, "dest"_a, "timestamp"_a=0)

            .def("transform_axis", static_cast<std::optional<Mat::Vector6d> (InnerEigenAPI::*)(const std::string &,
                                                                                               const std::string &,
                                                                                               std::uint64_t timestamp)>(&InnerEigenAPI::transform_axis),
                 "orig"_a, "dest"_a, "timestamp"_a=0)
            .def("transform_axis",
                 static_cast<std::optional<Mat::Vector6d> (InnerEigenAPI::*)(const std::string &, const Mat::Vector6d &,
                                                                             const std::string &,
                                                                             std::uint64_t timestamp)>(&InnerEigenAPI::transform_axis),
                 "orig"_a, "vector"_a, "dest"_a, "timestamp"_a=0)


            .def("get_transformation_matrix",
                 [](InnerEigenAPI &self, const std::string &dest,
                    const std::string &orig, std::uint64_t timestamp) -> std::optional<Eigen::Matrix<double, 4, 4>> {
                     auto tmp = self.get_transformation_matrix(dest, orig, timestamp);
                     if (tmp.has_value()) {
                         Eigen::Matrix<double, 4, 4> Trans;
                         Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
                         Trans.block<3, 3>(0, 0) = tmp.value().rotation();
                         Trans.block<3, 1>(0, 3) = tmp.value().translation();
                         return Trans;
                     } else {
                         return std::nullopt;
                     }
                 }, "orig"_a, "dest"_a, "timestamp"_a=0)
            .def("get_rotation_matrix", &InnerEigenAPI::get_rotation_matrix, "orig"_a, "dest"_a, "timestamp"_a=0)
            .def("get_translation_vector", &InnerEigenAPI::get_translation_vector, "orig"_a, "dest"_a, "timestamp"_a=0)
            .def("get_euler_xyz_angles", &InnerEigenAPI::get_euler_xyz_angles, "orig"_a, "dest"_a, "timestamp"_a=0);


    /*
    py::class_<CameraAPI>(m, "camera_api")
            .def(py::init([](DSRGraph &g, Node &cam) -> std::unique_ptr<CameraAPI> {
                return g.get_camera_api(cam);
            }))
            .def_property_readonly("id", [](CameraAPI& self) { return self.get_id();})
            .def_property_readonly("focal", [](CameraAPI& self) { return self.get_focal();})
            .def_property_readonly("focal_x", [](CameraAPI& self) { return self.get_focal_x();})
            .def_property_readonly("focal_y", [](CameraAPI& self) { return self.get_focal_y();})
            .def_property_readonly("height", [](CameraAPI& self) { return self.get_height();})
            .def_property_readonly("width", [](CameraAPI& self) { return self.get_width();})
            .def("get_roi_depth", &CameraAPI::get_roi_depth)
            .def("project", &CameraAPI::project)

            .def("reload_camera", &CameraAPI::reload_camera)
            .def("get_rgb_image", &CameraAPI::get_rgb_image)
            .def("get_depth_image",
                    static_cast<std::optional<std::vector<float>> (CameraAPI::*)()>(&CameraAPI::get_depth_image))
            .def("get_depth_image",
                 static_cast<std::optional<std::reference_wrapper<const std::vector<uint8_t>>> (CameraAPI::*)() const>(&CameraAPI::get_depth_image))
            .def("get_pointcloud", &CameraAPI::get_pointcloud)
            .def("get_depth_as_gray_image", &CameraAPI::get_depth_as_gray_image)

    ;
    */
}
