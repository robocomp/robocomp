
#ifndef DSR_UTILS
#define DSR_UTILS

#include <string>
#include <optional>
#include <dsr/core/topics/IDLGraphPubSubTypes.h>
#include <dsr/core/types/crdt_types.h>
#include <dsr/core/types/user_types.h>
#include <dsr/api/dsr_exceptions.h>

namespace DSR
{
    class DSRGraph;
    class Utilities
    {
        public:
            explicit Utilities(DSRGraph *G_);
            void read_from_json_file(const std::string &json_file_path, const std::function<std::optional<uint64_t>(const Node&)>& insert_node);
			void write_to_json_file(const std::string &json_file_path, const std::vector<std::string> &skip_node_content);
			static QJsonObject Node_to_QObject(const Node& node, bool skip_content);
			static QJsonObject Edge_to_QObject(const Edge& edge);
			static QJsonDocument file_to_QJsonDocument(const std::string &json_file_path);
			static QJsonDocument DSRGraph_to_QJsonDocument(DSR::DSRGraph *G_, const std::vector<std::string> &skip_node_content);


            void print();
            static void print_edge(const Edge &edge);
            static void print_node(const Node &node);
            void print_node(uint64_t id);
            void print_RT(uint64_t root);

        private:
            DSR::DSRGraph *G;
            void print_RT(const Node& node);

    };
}

#endif