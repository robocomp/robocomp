
#include "dsr_camera_api.h"

using namespace DSR;

//InnerAPI::InnerAPI(std::shared_ptr<DSR::DSRGraph> _G)
CameraAPI::CameraAPI(DSR::DSRGraph *G_, const DSR::Node &node)
{
    G = G_;
}
