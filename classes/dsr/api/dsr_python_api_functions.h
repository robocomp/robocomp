#ifndef DSR_PYTHON_API_INSERT
#define DSR_PYTHON_API_INSERT

#include <cassert>
#include "../core/topics/IDLGraphPubSubTypes.h"
#include "../core/types/user_types.h"
#include <optional>

namespace DSR
{
    class DSRGraph;

    /*
    This class is used to allow the insertion of nodes from the python api, 
    because it is not possible to load the Ice libraries into the wrapper 
    (there are conflicts with the libIce++11 and IcePy symbols). 
    */

    class PY_INSERT_API 
    {
        public:
        /*
        When a Node is created in the python api we get the id from idserver, so we don't need to do that here.
        */
        std::optional<uint64_t> insert_node_python(DSR::DSRGraph &g, Node &n);

    };

} // namespace DSR


#endif