
#ifndef DSR_UTILS
#define DSR_UTILS

#include <string>
#include <optional>
#include "../core/topics/IDLGraphPubSubTypes.h"
#include "../core/types/crdt_types.h"
#include "../core/types/user_types.h"
#include "dsr_exceptions.h"

namespace DSR
{
    class DSRGraph;
    class Utilities
    {
        public:
            explicit Utilities(DSRGraph *G_);
            void read_from_json_file(const std::string &json_file_path, const std::function<std::optional<int>(const Node&)>& insert_node);
			void write_to_json_file(const std::string &json_file_path);
			static QJsonObject Node_to_QObject(const Node& node);
			static QJsonObject Edge_to_QObject(const Edge& edge);
			static QJsonDocument file_to_QJsonDocument(const std::string &json_file_path);
			static QJsonDocument DSRGraph_to_QJsonDocument(DSR::DSRGraph *G_);


            void print();
            static void print_edge(const Edge &edge);
            static void print_node(const Node &node);
            void print_node(int id);
            void print_RT(std::int32_t root);

        private:
            DSR::DSRGraph *G;
            void print_RT(const Node& node);

    };
}

#endif