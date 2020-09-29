#ifndef INNER_API
#define INNER_API

#include <cassert>
#include <qmat/QMatAll>
#include "../core/topics/IDLGraphPubSubTypes.h"

namespace DSR
{
    class DSRGraph;
    class VEdge;

    class InnerAPI : public QObject
    {
        Q_OBJECT
        using key_transform = std::tuple<std::string, std::string>;
        using node_reference = std::map<int32_t,std::list<key_transform>>;
        using transform_cache = std::map<key_transform, RMat::RTMat>;
        using node_matrix = std::tuple<int32_t, RMat::RTMat>;
        using Lists = std::tuple<std::list<node_matrix>, std::list<node_matrix>>;
        public:
            explicit InnerAPI(DSRGraph *G_);

            /////////////////////////////////////////////////
            /// Kinematic transformation methods
            ////////////////////////////////////////////////
            std::optional<QVec> transform( const std::string &destId, const std::string &origId);
            std::optional<QVec> transform( const std::string &destId, const QVec &origVec, const std::string &origId);
            std::optional<QVec> transform_axis(const std::string &destId, const std::string & origId);
            std::optional<QVec> transform_axis(const std::string &destId, const QVec &origVec, const std::string & origId);

            ////////////////////////////////////////////////
            /// Transformation matrix retrieval methods
            ////////////////////////////////////////////////
            std::optional<RTMat> get_transformation_matrix(const std::string &destId, const std::string &origId);
            QMat get_rotation_matrix(const std::string &destId, const std::string &origId){ return{};};
            QVec get_translation_vector(const std::string &destId, const std::string &origId){return{};};
            QVec get_euler_xyz_angles(const std::string &destId, const std::string &origId){return{};};

        public slots:
            void add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& edge_type);
            void del_node_slot(const std::int32_t id);
            void del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &edge_type);
        private:
            DSR::DSRGraph *G;
            transform_cache cache;
            node_reference node_map;
            std::optional<InnerAPI::Lists> set_lists(const std::string &origId, const std::string &destId);
            void remove_cache_entry(const std::int32_t id);
    };
}

#endif
