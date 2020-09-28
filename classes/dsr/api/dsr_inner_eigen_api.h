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
            //std::optional<QVec> transform(const QString & destId, const QVec &origVec, const QString & origId);
            //std::optional<QVec> transform( const QString &destId, const QString & origId);
            std::optional<Eigen::VectorXd> transformS( const std::string &destId, const Eigen::VectorXd &origVec, const std::string & origId);
            //std::optional<QVec> transformS( const std::string &destId, const std::string &origId);
            //std::optional<QVec> transform6D(const QString &destId, const QVec &origVec, const QString & origId);
            //std::optional<QVec> transform6D(const QString &destId, const QString & origId);
            //std::optional<QVec> transformS6D(const std::string &destId, const std::string & origId);
            //std::optional<QVec> transformS6D(const std::string &destId, const QVec &origVec, const std::string & origId);

            ////////////////////////////////////////////////
            /// Transformation matrix retrieval methods
            ////////////////////////////////////////////////
            //std::optional<RTMat> getTransformationMatrix(const QString &destId, const QString &origId);
            std::optional<Mat::RTMat> getTransformationMatrixS(const std::string &destId, const std::string &origId);
            //QMat getRotationMatrixTo(const QString &to, const QString &from);
            //QVec getTranslationVectorTo(const QString &to, const QString &from);
            //QVec rotationAngles(const QString & destId, const QString & origId);

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
            std::optional<InnerEigenAPI::Lists> setLists(const std::string &origId, const std::string &destId);
            void remove_cache_entry(const std::int32_t id);
    };
}

#endif