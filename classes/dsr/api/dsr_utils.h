
#ifndef DSR_UTILS
#define DSR_UTILS

#include <string>
#include <optional>
#include "../core/topics/DSRGraphPubSubTypes.h"
#include "dsr_exceptions.h"

namespace DSR
{
    class DSRGraph;
    class Utilities
    {
        public:
            Utilities(DSRGraph *G_);
            void read_from_json_file(const std::string &json_file_path, std::function<std::optional<int>(const Node&)> insert_node);
            void write_to_json_file(const std::string &json_file_path);
            void print();
            void print_edge(const Edge &edge);
            void print_node(const Node &node);
            void print_node(int id);
            void print_RT(std::int32_t root);
    
        private:
            DSR::DSRGraph *G;
            void print_RT(const Node& node);

    };
};

#endif