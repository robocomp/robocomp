//
// Created by juancarlos on 18/11/20.
//

#include "dsr/api/dsr_api.h"

using namespace DSR;

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <utility>

#include <memory>
#include <functional>




namespace py = pybind11;

using namespace py::literals;
using namespace RoboCompDSRGetID;

class scoped_ostream_discard
{
protected:
    std::streambuf *old;
    std::ostream &costream;

public:
    explicit scoped_ostream_discard(
            std::ostream &costream = std::cout)
            : costream(costream) {
        old = costream.rdbuf(nullptr);
    }

    ~scoped_ostream_discard() {
        costream.rdbuf(old);
    }

    scoped_ostream_discard(const scoped_ostream_discard &) = delete;

    scoped_ostream_discard(scoped_ostream_discard &&other) = default;

    scoped_ostream_discard &operator=(const scoped_ostream_discard &) = delete;

    scoped_ostream_discard &operator=(scoped_ostream_discard &&) = delete;
};

PYBIND11_MAKE_OPAQUE(std::map<std::pair<uint32_t, std::string>, Edge>)
PYBIND11_MAKE_OPAQUE(std::map<std::string, Attribute>)

PYBIND11_MODULE(pydsr, m) {

    py::bind_map<std::map<std::pair<uint32_t, std::string>, Edge>>(m, "MapStringEdge");
    py::bind_map<std::map<std::string, Attribute>>(m, "MapStringAttribute");

    m.doc() = "Documentation here";


    uint32_t local_agent_id = -1;


    //Disable messages from Qt.
    qInstallMessageHandler([](QtMsgType type, const QMessageLogContext &context, const QString &msg) {});

    //Disable cout
    m.attr("redirect_output") = py::capsule(new scoped_ostream_discard(),
                                            [](void *sor) { delete static_cast<scoped_ostream_discard *>(sor); });

    //DSR Attribute class
    py::class_<Attribute>(m, "Attribute")
            .def(py::init<ValType, uint64_t, uint32_t>(),
                 "value"_a, "timestamp"_a, "agent_id"_a)
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
                        for (const auto &k: std::get<std::vector<uint8_t>>(self.value()))
                            out << std::to_string(k) + ", ";
                        out << "] ";
                        break;
                    case 6:
                        out << std::to_string(std::get<uint32_t>(self.value()));
                        break;
                }
                out << " >";
                return out.str();
            })
            .def_property_readonly("agent_id", [](Attribute &self) { return self.agent_id(); },
                                   "read the agent_id attribute. This property is readonly ans it is updated when a change is made in the value property.")
            .def_property_readonly("timestamp", [](Attribute &self) { return self.timestamp(); },
                                   "read the timestamp (ns) attribute. This property is readonly and it is updated when a change is made in the value property.")
            .def_property("value", [](Attribute &self) -> ValType { return self.value(); },
                          [&](Attribute &self, const ValType &val) {
                              if (val.index() == static_cast<std::size_t>(self.selected())) {
                                  self.value(val);
                                  self.timestamp(get_unix_timestamp());
                                  self.agent_id(local_agent_id);
                              } else {
                                  throw std::runtime_error("Attributes cannot change type.");
                              }
                          },
                          py::return_value_policy::copy, "read or assign a new value to the Attribute object.");

    //DSR Edge class
    py::class_<Edge>(m, "Edge")
            .def(py::init<uint32_t, uint32_t, std::string, uint32_t>(),
                 "to"_a, "from"_a, "type"_a, "agent_id"_a)
            .def("__repr__", [](Edge const &self) {
                std::stringstream out;
                out << "------------------------------------" << std::endl;
                out << "Type: " << self.type() << " from: " << self.from() << " to: " << self.to() << std::endl;
                for (auto[k, v] : self.attrs())
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
            .def(py::init([](uint32_t agent_id, const std::string &type, py::object *proxy,
                             const std::string &name = "") -> std::unique_ptr<Node> {
                auto tmp = std::make_unique<Node>(agent_id, type);
                tmp->name(name);
                if (!proxy)  throw std::runtime_error("Proxy cannot be None");
                PyObject *fn = PyObject_GetAttrString(proxy->ptr(), "getID");
                if (!fn)     throw std::runtime_error("Proxy does not have method getID");
                PyObject *result = PyObject_CallMethod(proxy->ptr(), "getID", nullptr);
                if (!result) throw std::runtime_error("Cannot get new id from idserver, check config file");
                tmp->id(py::cast<std::uint32_t>(result));

                return tmp;
            }), "agent_id"_a, "type"_a, "dsr_idproxy"_a, "name"_a = "")
            .def("__repr__", [](Node const &self) {
                std::stringstream out;

                out << "------------------------------------" << std::endl;
                out << "Node: " << self.id() << std::endl;
                out << "  Type: " << self.type() << std::endl;
                out << "  Name: " << self.name() << std::endl;
                out << "  Agent id: " << self.agent_id() << std::endl;
                out << "  ATTRIBUTES: [";
                for (auto[key, val] : self.attrs())
                    out << "      [" << key << "] -- " << val << std::endl;
                out << "   ]" << std::endl;
                out << "  EDGES: [" << std::endl;
                for (auto[key, val] : self.fano()) {
                    out << "          Edge type: " << val.type() << " from: " << val.from() << " to: " << val.to()
                        << std::endl;
                    for (auto[k, v] : val.attrs())
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
                          [](Node &self) -> std::map<std::pair<uint32_t, std::string>, Edge> & { return self.fano(); },
                          [](Node &self, const std::map<std::pair<uint32_t, std::string>, Edge> &edges) {
                              return self.fano(edges);
                          },
                          py::return_value_policy::reference, "read or write in the edge map of the node.");



    //DSR DSRGraph class
    py::class_<DSRGraph>(m, "DSRGraph")
            .def(py::init([&](int root, const std::string &name, int id,
                              const std::string &dsr_input_file = "",
                              bool all_same_host = true) -> std::unique_ptr<DSRGraph> {

                     py::gil_scoped_release release;
                     local_agent_id = id;

                     auto g = std::make_unique<DSRGraph>(root, name, id, dsr_input_file, nullptr, all_same_host);

                     return g;
                 }), "root"_a, "name"_a, "id"_a, "dsr_input_file"_a = "",
                 "all_same_host"_a = true )
            .def("get_node", [](DSRGraph &self, uint32_t id) -> std::optional<Node> {
                return self.get_node(id);
            }, "id"_a, "return the node with the id passed as parameter. Returns None if the node does not exist.")
            .def("get_node", [](DSRGraph &self, const std::string &name) -> std::optional<Node> {
                return self.get_node(name);
            }, "name"_a, "return the node with the name passed as parameter. Returns None if the node does not exist.")
            .def("delete_node", static_cast<bool (DSRGraph::*)(uint32_t)>(&DSRGraph::delete_node), "id"_a,
                 "delete the node with the given id. Returns a bool with the result o the operation.")
            .def("delete_node",
                 static_cast<bool (DSRGraph::*)(const std::basic_string<char> &)>(&DSRGraph::delete_node), "name"_a,
                 "delete the node with the given name. Returns a bool with the result o the operation.")
            .def("insert_node", [](DSRGraph &g, Node&n) {
                     return PY_INSERT_API().insert_node_python(g, n);
                }, "node"_a,
                 "Insert in the graph the new node passed as parameter. Returns the id of the node or None if the Node alredy exist in the map.")
            .def("update_node", &DSRGraph::update_node, "node"_a, "Update the node in the graph. Returns a bool.")
            .def("get_edge", [](DSRGraph &self, const std::string &from, const std::string &to,
                                const std::string &key) -> std::optional<Edge> {
                     return self.get_edge(from, to, key);
                 }, "from"_a, "to"_a, "type"_a,
                 "Return the edge with the parameters from, to, and type passed as parameter. If the edge does not exist it return None")
            .def("get_edge",
                 [](DSRGraph &self, uint32_t from, uint32_t to, const std::string &key) -> std::optional<Edge> {
                     return self.get_edge(from, to, key);
                 }, "from"_a, "to"_a, "type"_a,
                 "Return the edge with the parameters from, to, and type passed as parameter.  If the edge does not exist it return None")
            .def("insert_or_assign_edge", &DSRGraph::insert_or_assign_edge, "edge"_a,
                 "Insert or updates and edge. returns a bool")
            .def("delete_edge", static_cast<bool (DSRGraph::*)(uint32_t, uint32_t,
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
            .def("insert_or_assign_edge_RT", [](RT_API &self, Node &n, uint32_t to,
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
            .def("get_translation", static_cast<std::optional<Eigen::Vector3d> (RT_API::*)(std::uint32_t,
                                                                                           std::uint32_t)>(&RT_API::get_translation),
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