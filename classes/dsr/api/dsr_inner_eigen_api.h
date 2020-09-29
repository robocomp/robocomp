#ifndef INNER_EIGEN_API
#define INNER_EIGEN_API

#include <QObject>
#include "../core/topics/IDLGraphPubSubTypes.h"
#include <Eigen/Geometry>

namespace DSR
{
    namespace Mat
    {
        using RTMat = Eigen::Transform<double, 3, Eigen::Affine>;
        using Rot3D = Eigen::Matrix3d;
        using Vector6d = Eigen::Matrix<double, 6, 1>;
    };

    class DSRGraph;

    class InnerEigenAPI : public QObject
    {
        Q_OBJECT
        using key_transform = std::tuple<std::string, std::string>;
        using node_reference = std::map<int32_t, std::list<key_transform>>;
        using transform_cache = std::map<key_transform, Mat::RTMat>;
        using node_matrix = std::tuple<int32_t, Mat::RTMat>;
        using Lists = std::tuple<std::list<node_matrix>, std::list<node_matrix>>;
        public:
            explicit InnerEigenAPI(DSRGraph *G_);

            /////////////////////////////////////////////////
            /// Kinematic transformation methods
            ////////////////////////////////////////////////
            std::optional<Eigen::Vector3d> transform( const std::string &dest, const std::string &orig);
            std::optional<Eigen::Vector3d> transform( const std::string &dest, const Eigen::Vector3d &vector, const std::string &orig);
            std::optional<Mat::Vector6d> transform_axis(const std::string &dest, const std::string & orig);
            std::optional<Mat::Vector6d> transform_axis(const std::string &dest, const Mat::Vector6d &vector, const std::string &orig);

            ////////////////////////////////////////////////
            /// Transformation matrix retrieval methods
            ////////////////////////////////////////////////
            //std::optional<Mat::RTMat> getTransformationMatrix(const QString &dest, const QString &orig);
            //std::optional<Mat::RTMat> getTransformationMatrixS(const std::string &dest, const std::string &orig);
            //QMat getRotationMatrixTo(const QString &to, const QString &from);
            //QVec getTranslationVectorTo(const QString &to, const QString &from);
            //QVec rotationAngles(const QString & dest, const QString & orig);
            std::optional<Mat::RTMat> get_transformation_matrix(const std::string &dest, const std::string &orig);
            std::optional<Mat::RTMat> get_rotation_matrix(const std::string &dest, const std::string &orig){ return{};};
            std::optional<Mat::RTMat> get_translation_vector(const std::string &dest, const std::string &orig){return{};};
            std::optional<Mat::RTMat> get_euler_xyz_angles(const std::string &dest, const std::string &orig){return{};};


        ////////////////////////////////////////////////
            /// Update transform methods
            ////////////////////////////////////////////////
//            void updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
//            void updateTransformValues(QString transformId, QVec v, QString parentId="");
//            void updateTransformValuesS(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId="");
//            void updateTransformValuesS(std::string transformId, QVec v, std::string parentId="");
//            void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
//            void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");

        public slots:
            void add_or_assign_node_slot(const std::int32_t id, const std::string &type);
            void add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& edge_type);
            void del_node_slot(const std::int32_t id);
            void del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &edge_type);

        private:
            DSR::DSRGraph *G;
            transform_cache cache;
            node_reference node_map;
            std::optional<InnerEigenAPI::Lists> setLists(const std::string &orig, const std::string &dest);
            void remove_cache_entry(const std::int32_t id);
    };
}

#endif