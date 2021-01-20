#ifndef INNER_EIGEN_API
#define INNER_EIGEN_API

#include <QObject>
#include "../core/topics/IDLGraphPubSubTypes.h"
#include "dsr_eigen_defs.h"
#include "dsr_rt_api.h"

namespace DSR
{
    class DSRGraph;

    class InnerEigenAPI : public QObject
    {
        Q_OBJECT
        using KeyTransform = std::tuple<std::string, std::string>;
        using NodeReference = std::map<uint64_t , std::list<KeyTransform>>;
        using TransformCache = std::map<KeyTransform, Mat::RTMat>;
        using NodeMatrix = std::tuple<uint64_t , Mat::RTMat>;

        public:
            explicit InnerEigenAPI(DSRGraph *G_);

            /////////////////////////////////////////////////
            /// Kinematic transformation methods
            ////////////////////////////////////////////////
            std::optional<Mat::Vector3d> transform( const std::string &dest, const std::string &orig);
            std::optional<Mat::Vector3d> transform( const std::string &dest, const Mat::Vector3d &vector, const std::string &orig);
            std::optional<Mat::Vector6d> transform_axis(const std::string &dest, const std::string & orig);
            std::optional<Mat::Vector6d> transform_axis(const std::string &dest, const Mat::Vector6d &vector, const std::string &orig);

            ////////////////////////////////////////////////
            /// Transformation matrix retrieval methods
            ////////////////////////////////////////////////
            std::optional<Mat::RTMat> get_transformation_matrix(const std::string &dest, const std::string &orig);
            std::optional<Mat::Rot3D> get_rotation_matrix(const std::string &dest, const std::string &orig);
            std::optional<Mat::Vector3d> get_translation_vector(const std::string &dest, const std::string &orig);
            std::optional<Mat::Vector3d> get_euler_xyz_angles(const std::string &dest, const std::string &orig);

        public slots:
            void add_or_assign_edge_slot(const uint64_t from, const uint64_t to, const std::string& edge_type);
            void del_node_slot(const uint64_t id);
            void del_edge_slot(const uint64_t from, const uint64_t to, const std::string &edge_type);

        private:
            DSR::DSRGraph *G;
            std::unique_ptr<DSR::RT_API> rt;
            TransformCache cache;
            NodeReference node_map;
            void remove_cache_entry(const uint64_t id);
    };
}

#endif