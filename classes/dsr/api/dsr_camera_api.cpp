
#include "dsr_camera_api.h"
#include "dsr_api.h"

using namespace DSR;

//InnerAPI::InnerAPI(std::shared_ptr<DSR::DSRGraph> _G)
CameraAPI::CameraAPI(DSR::DSRGraph *G_, const DSR::Node &camera)
{
    G = G_;
    if( auto o_focal_x = G->get_attrib_by_name<focalx_att>(camera); o_focal_x.has_value())
        focal_x = o_focal_x.value();
    if( auto o_focal_y = G->get_attrib_by_name<focalx_att>(camera); o_focal_y.has_value())
        focal_x = o_focal_y.value();
    if( auto o_width = G->get_attrib_by_name<rgb_width_att>(camera); o_width.has_value())
    {
        width = o_width.value();
        centre_x = width / 2;
    }
    if( auto o_height = G->get_attrib_by_name<rgb_height_att>(camera); o_height.has_value())
    {
        height = o_height.value();
        centre_y = height/2;
    }
    if( auto o_depth = G->get_attrib_by_name<rgb_depth_att>(camera); o_depth.has_value())
        depth = o_depth.value();
    if( auto o_id = G->get_attrib_by_name<rgb_cameraID_att>(camera); o_id.has_value())
        depth = o_id.value();
}

Eigen::Vector2d CameraAPI::project( const Eigen::Vector3d & p) const
{
    Eigen::Vector2d proj;
    proj << focal_x * p.y() / p.x() + centre_x, focal_y * p.z() / p.x() + centre_y;
    return proj;
}