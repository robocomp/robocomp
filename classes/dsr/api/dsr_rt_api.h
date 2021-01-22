#ifndef RTAPI
#define RTAPI

#include <cassert>
#include <QObject>
#include <qmat/QMatAll>
#include "../core/topics/IDLGraphPubSubTypes.h"
#include "../core/types/user_types.h"
#include "../core/types/type_checking/dsr_attr_name.h"
#include "dsr_eigen_defs.h"

namespace DSR
{
    class DSRGraph;

    class RT_API : public QObject
    {
        public:
            explicit RT_API(DSRGraph *G_);

            void insert_or_assign_edge_RT(Node &n, uint64_t to, const std::vector<float> &trans, const std::vector<float> &rot_euler);
            void insert_or_assign_edge_RT(Node &n, uint64_t to, std::vector<float> &&trans, std::vector<float> &&rot_euler);
            std::optional<Edge> get_edge_RT(const Node &n, uint64_t to);
            std::optional<Mat::RTMat> get_RT_pose_from_parent(const Node &n);
            std::optional<Mat::RTMat> get_edge_RT_as_rtmat(const Edge &edge);
            std::optional<Eigen::Vector3d> get_translation(const Node &n, uint64_t to);
            std::optional<Eigen::Vector3d> get_translation(uint64_t node_id, uint64_t to);
            // std::optional<Mat::RTMat> get_edge_RT_as_rtmat(const Node &n, uint32_t to);
            // std::optional<std::tuple<Mat::Vector3d, Mat::Quaterniond>> get_edge_RT_as_tr_plus_quaternion(const Edge &edge);
            // std::optional<Mat::MatXX> get_jacobian(const Node &base, const Node &tip)

        public slots:
//            void add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& edge_type);
//            void del_node_slot(const std::int32_t id);
//            void del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &edge_type);
        private:
            DSR::DSRGraph *G;
    };
}

#endif
