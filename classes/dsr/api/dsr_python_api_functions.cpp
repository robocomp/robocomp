
#include "dsr_python_api_functions.h"
#include "dsr_api.h"

using namespace DSR;


std::optional<std::uint32_t> PY_INSERT_API::insert_node_python(DSR::DSRGraph &g,  Node &node)
{
    std::optional<IDL::Mvreg> aw;
    bool r = false;
    {
        std::unique_lock<std::shared_mutex> lock(g._mutex);

        if (node.name().empty() or g.name_map.find(node.name()) != g.name_map.end())
            node.name(node.type() + "_" + std::to_string(node.id()));

        std::tie(r, aw) = g.insert_node_(user_node_to_crdt(node));
    }
    if (r)
    {
        if (!g.copy)
        {
            if (aw.has_value())
            {
                g.dsrpub_node.write(&aw.value());
                emit g.update_node_signal(node.id(), node.type());
                for (const auto &[k, v] : node.fano())
                {
                    emit g.update_edge_signal(node.id(), k.first, k.second);
                }
            }
        }
        return node.id();
    }
    return {};
}