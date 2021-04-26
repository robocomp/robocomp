//
// Created by juancarlos on 18/11/20.
//

#include "dsr/api/dsr_api.h"

using namespace DSR;

#include "include/silent_output.h"
#include "include/signal_function_caster.h"
#include "include/custom_bind_map.h"
//#include "include/vector_caster.h"

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
        std::function<void(std::uint64_t, std::uint64_t, const std::vector<std::string> &)>,
        std::function<void(std::uint64_t)>
>;


using attribute_type = std::variant<std::string,
                                    bool,
                                    py::array_t<uint8_t>,
                                    std::vector<uint8_t>,
                                    py::array_t<float>,
                                    std::vector<float>,
                                    uint64_t,
                                    double,
                                    float,
                                    int32_t,
                                    uint32_t>;

static constexpr std::array<std::string_view, 11> attribute_type_TYPENAMES_UNION = { "STRING", "BOOL", "NUMPY_BYTE_VEC", "BYTE_VEC",
                                                                                    "NUMPY_FLOAT_VEC", "FLOAT_VEC", "UINT64", "DOUBLE", "FLOAT",
                                                                     "INT", "UINT"};


template<typename T>
ValType convert_variant_fn(const attribute_type & e)
{
    if constexpr (std::is_same_v<T, py::array_t<uint8_t>>)
    {
        auto tmp = std::get<py::array_t<uint8_t>>(e);
        const auto size = tmp.size();
        py::buffer_info x = tmp.request();
        return ValType(std::vector<uint8_t>{static_cast<uint8_t *>(x.ptr), static_cast<uint8_t *>(x.ptr) + size});
    } else if constexpr (std::is_same_v<T, py::array_t<float>>)
    {
        auto tmp = std::get<py::array_t<float>>(e);
        const auto size = tmp.size();
        py::buffer_info x = tmp.request();
        return ValType(std::vector<float>{static_cast<float *>(x.ptr), static_cast<float *>(x.ptr) + size});
    }
    else
    {
        auto x = std::get<T>(e);
        return ValType(x);
    }
};



ValType convert_variant(const attribute_type & e)
{
    typedef ValType (*conver_fn) (const attribute_type &);
    //using t = DSR::Types;

    /*constexpr std::array<uint8_t, 9> idx_ValType = {t::STRING,
                                                               t::BOOL,
                                                               t::BYTE_VEC,
                                                               t::FLOAT_VEC,
                                                               t::UINT64,
                                                               t::DOUBLE,
                                                               t::FLOAT,
                                                               t::INT,
                                                               t::UINT };
    */
    constexpr std::array<conver_fn, 11> cast = { convert_variant_fn<std::string>,
                                                 convert_variant_fn<bool>,
                                                 convert_variant_fn<py::array_t<uint8_t>>,
                                                 convert_variant_fn<std::vector<uint8_t>>,
                                                 convert_variant_fn<py::array_t<float>>,
                                                 convert_variant_fn<std::vector<float>>,
                                                 convert_variant_fn<uint64_t>,
                                                 convert_variant_fn<double>,
                                                 convert_variant_fn<float>,
                                                 convert_variant_fn<int32_t>,
                                                 convert_variant_fn<uint32_t>
                                             };

    const auto idx = e.index(); //idx_ValType.at(e.index());
    return cast[idx](e);
}

PYBIND11_MAKE_OPAQUE(std::map<std::pair<uint64_t, std::string>, Edge>)
PYBIND11_MAKE_OPAQUE(std::map<std::string, Attribute>)
//(std::vector<uint8_t>)
//PYBIND11_MAKE_OPAQUE(std::vector<float>)


PYBIND11_MODULE(pydsr, m) {

    py::bind_map<std::map<std::pair<uint64_t, std::string>, Edge>>(m, "MapStringEdge");
    py::bind_dsr_map<std::map<std::string, Attribute>>(m, "MapStringAttribute");
    //py::bind_vector<std::vector<uint8_t>>(m, "ByteVec");

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
                                     std::get<std::function<void(std::uint64_t, std::uint64_t,
                                                                 const std::vector<std::string> &)>>(fn_callback));
                } catch (std::exception &e) {
                    std::cout << "Update Edge Attribute Callback must be (int, int, [str])\n " << std::endl;
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
                switch (self.selected()) {
                    case 0:
                        out << std::get<std::string>(self.value());
                        break;
                    case 1:
                        out << std::to_string(std::get<int32_t>(self.value()));
                        break;
                    case 2:
                        out << std::get<float>(self.value());
                        break;
                    case 3:
                        out << "[ ";
                        for (const auto &k: std::get<std::vector<float>>(self.value()))
                            out << std::to_string(k) + ", ";
                        out << "] ";
                        break;
                    case 4:
                        out << (std::get<bool>(self.value()) ? " TRUE" : " FALSE");
                        break;
                    case 5:
                        out << "[ ";
                        for (const uint8_t k: std::get<std::vector<uint8_t>>(self.value()))
                            out << std::to_string(static_cast<uint32_t>(k)) + ", ";
                        out << "] ";
                        break;
                    case 6:
                        out << std::to_string(std::get<uint32_t>(self.value()));
                        break;
                    case 7:
                        out << std::to_string(std::get<uint64_t>(self.value()));
                        break;
                    case 8:
                        out << std::to_string(std::get<double>(self.value()));
                        break;
                }
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
                                  default:
                                      throw std::runtime_error("Unreachable");
                              }

                },
                          [&](Attribute &self, const attribute_type &val) {
                              try {
                                  //std::cout << "val idx: " << std::string{attribute_type_TYPENAMES_UNION[val.index()]} << " selected: " << std::string{DSR::TYPENAMES_UNION[self.selected()]} << std::endl;
                                  switch (self.selected()) {
                                      case 0:
                                          self.str(std::get<std::string>(val));
                                          break;
                                      case 1:
                                          if (val.index() == 6) self.dec(std::get<uint64_t>(val));
                                          else self.dec(std::get<int32_t>(val));
                                          break;
                                      case 2:
                                          if (val.index() == 7) self.fl(std::get<double>(val));
                                          else self.fl(std::get<float>(val));
                                          break;
                                      case 3:
                                          if (val.index() == 4) {
                                              auto tmp = std::get<py::array_t<float>>(val);
                                              const auto size = tmp.size();
                                              py::buffer_info x = tmp.request();
                                              self.float_vec(std::vector<float>{static_cast<float *>(x.ptr), static_cast<float *>(x.ptr) + size});
                                          }
                                          else self.float_vec(std::get<std::vector<float>>(val));
                                          break;
                                      case 4:
                                          self.bl(std::get<bool>(val));
                                          break;
                                      case 5:
                                          if (val.index() == 2) {
                                              auto tmp = std::get<py::array_t<uint8_t>>(val);
                                              const auto size = tmp.size();
                                              py::buffer_info x = tmp.request();
                                              self.byte_vec(std::vector<uint8_t>{static_cast<uint8_t *>(x.ptr), static_cast<uint8_t *>(x.ptr) + size});
                                          }
                                          else self.byte_vec(std::get<std::vector<uint8_t>>(val));
                                          break;
                                      case 6:
                                          if (val.index() == 6) self.uint(std::get<uint64_t>(val));
                                          else self.uint(std::get<uint32_t>(val));
                                          break;
                                      case 7:
                                          self.uint64(std::get<uint64_t>(val));
                                          break;
                                      case 8:
                                          self.dob(std::get<double>(val));
                                          break;
                                      default:
                                          throw std::runtime_error("Attributes cannot change type. Selected type is " +
                                                                   std::string{DSR::TYPENAMES_UNION[self.selected()]} +
                                                                   " and used type is " + std::string{
                                                  attribute_type_TYPENAMES_UNION[val.index()]});
                                      };
                                      self.timestamp(get_unix_timestamp());
                                      self.agent_id(local_agent_id);
                              } catch (...) {
                                  throw std::runtime_error("Attributes cannot change type. Selected type is " + std::string{DSR::TYPENAMES_UNION[self.selected()]} + " and used type is " + std::string{attribute_type_TYPENAMES_UNION[val.index()]});
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
            .def("update_node", &DSRGraph::update_node, "node"_a, "Update the node in the graph. Returns a bool.")
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
            .def("insert_or_assign_edge", &DSRGraph::insert_or_assign_edge, "edge"_a,
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
            .def("get_edge_RT_as_rtmat", [](RT_API &self, Edge &e) -> std::optional<Eigen::Matrix<double, 4, 4>> {
                auto tmp = self.get_edge_RT_as_rtmat(e);
                if (tmp.has_value()) {
                    Eigen::Matrix<double, 4, 4> Trans;
                    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
                    Trans.block<3, 3>(0, 0) = tmp.value().rotation();
                    Trans.block<3, 1>(0, 3) = tmp.value().translation();
                    return Trans;
                } else {
                    return std::nullopt;
                }
            }, "edge"_a)
            .def("get_translation", static_cast<std::optional<Eigen::Vector3d> (RT_API::*)(std::uint64_t,
                                                                                           std::uint64_t)>(&RT_API::get_translation),
                 "node_id"_a, "to"_a);

    py::class_<InnerEigenAPI>(m, "inner_api")
            .def(py::init([](DSRGraph &g) -> std::unique_ptr<InnerEigenAPI> {
                return g.get_inner_eigen_api();
            }), "graph"_a)
            .def("transform", static_cast<std::optional<Eigen::Vector3d> (InnerEigenAPI::*)(const std::string &,
                                                                                            const std::string &)>(&InnerEigenAPI::transform),
                 "orig"_a, "dest"_a)
            .def("transform", static_cast<std::optional<Eigen::Vector3d> (InnerEigenAPI::*)(const std::string &,
                                                                                            const Mat::Vector3d &,
                                                                                            const std::string &)>(&InnerEigenAPI::transform),
                 "orig"_a, "vector"_a, "dest"_a)

            .def("transform_axis", static_cast<std::optional<Mat::Vector6d> (InnerEigenAPI::*)(const std::string &,
                                                                                               const std::string &)>(&InnerEigenAPI::transform_axis),
                 "orig"_a, "dest"_a)
            .def("transform_axis",
                 static_cast<std::optional<Mat::Vector6d> (InnerEigenAPI::*)(const std::string &, const Mat::Vector6d &,
                                                                             const std::string &)>(&InnerEigenAPI::transform_axis),
                 "orig"_a, "vector"_a, "dest"_a)


            .def("get_transformation_matrix",
                 [](InnerEigenAPI &self, const std::string &dest,
                    const std::string &orig) -> std::optional<Eigen::Matrix<double, 4, 4>> {
                     auto tmp = self.get_transformation_matrix(dest, orig);
                     if (tmp.has_value()) {
                         Eigen::Matrix<double, 4, 4> Trans;
                         Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
                         Trans.block<3, 3>(0, 0) = tmp.value().rotation();
                         Trans.block<3, 1>(0, 3) = tmp.value().translation();
                         return Trans;
                     } else {
                         return std::nullopt;
                     }
                 }, "orig"_a, "dest"_a)
            .def("get_rotation_matrix", &InnerEigenAPI::get_rotation_matrix, "orig"_a, "dest"_a)
            .def("get_translation_vector", &InnerEigenAPI::get_translation_vector, "orig"_a, "dest"_a)
            .def("get_euler_xyz_angles", &InnerEigenAPI::get_euler_xyz_angles, "orig"_a, "dest"_a);




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
