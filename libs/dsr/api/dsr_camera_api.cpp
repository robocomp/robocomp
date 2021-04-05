
#include <dsr/api/dsr_camera_api.h>
#include <dsr/api/dsr_api.h>

using namespace DSR;

CameraAPI::CameraAPI(DSR::DSRGraph *G_, const DSR::Node &camera)
{
    G = G_;
    id = camera.id();
    if( auto o_focal_x = G->get_attrib_by_name<cam_rgb_focalx_att>(camera); o_focal_x.has_value())
        focal_x = o_focal_x.value();
    else qFatal("CameraAPI constructor: aborting since no focal_x attr found in camera");
    if( auto o_focal_y = G->get_attrib_by_name<cam_rgb_focalx_att>(camera); o_focal_y.has_value())
        focal_y = o_focal_y.value();
    else qFatal("CameraAPI constructor: aborting since no focal_y attr found in camera");
    if( auto o_width = G->get_attrib_by_name<cam_rgb_width_att>(camera); o_width.has_value())
    {
        width = o_width.value();
        centre_x = width / 2;
    }
    else qFatal("CameraAPI constructor: aborting since no width attr found in camera");
    if( auto o_height = G->get_attrib_by_name<cam_rgb_height_att>(camera); o_height.has_value())
    {
        height = o_height.value();
        centre_y = height/2;
    }
    else qFatal("CameraAPI constructor: aborting since no height attr found in camera");
    if( auto o_depth = G->get_attrib_by_name<cam_rgb_depth_att>(camera); o_depth.has_value())
        depth = o_depth.value();
    else qFatal("CameraAPI constructor: aborting since no depth attr found in camera");
    if( auto o_id = G->get_attrib_by_name<cam_rgb_cameraID_att>(camera); o_id.has_value())
        cameraID = o_id.value();
}

void CameraAPI::set_focal( float f)
{
  focal = f;  focal_x = f; focal_y = f;
}

void CameraAPI::set_focal_x( float fx)
{
    focal_x = fx;
}

void CameraAPI::set_focal_y( float fy)
{
    focal_y = fy;
}

void CameraAPI::set_width( std::uint32_t w)
{
    width = w;
    centre_x = w/2;
}

void CameraAPI::set_height( std::uint32_t h)
{
    height = h;
    centre_y = h/2;
}

Eigen::Vector2d CameraAPI::project(const Eigen::Vector3d & p, int cx, int  cy) const
{
    Eigen::Vector2d proj;
    if(cx==-1) cx=centre_x;
    if(cy==-1) cy=centre_y;
    proj << focal_x * p.x() / p.y() + cx, -focal_y * p.z() / p.y() + cy;
    //proj << focal_x * /*(608/640) */ p.x() / p.y() + cx, -focal_y * (416./480) * p.z() / p.y() + cy;  //FIXXXXX IT


    return proj;
}

//std::optional<std::reference_wrapper<const std::vector<uint8_t>>> CameraAPI::get_existing_rgb_image() const
//{
//    auto &attrs = node.attrs();
//    if (auto value = attrs.find("cam_rgb"); value != attrs.end())
//        return value->second.byte_vec();
//    else
//    {
//        qWarning() << __FUNCTION__ << "No rgb attribute found in node " << QString::fromStdString(node.name()) << ". Returning empty";
//        return {};
//    }
//}

/*
std::optional<std::reference_wrapper<const std::vector<uint8_t>>> CameraAPI::get_rgb_image() const
{
    if( const auto n = G->get_node(id); n.has_value())
    {
        auto &attrs = n.value().attrs();
        if (auto value = attrs.find("cam_rgb"); value != attrs.end())
            return value->second.byte_vec();
        else
        {
            qWarning() << __FUNCTION__ << "No rgb attribute found in node " << QString::fromStdString(n.value().name()) << ". Returning empty";
            return {};
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << "No camera node found in G. Returning empty";
        return {};
    }
}*/

std::optional<std::vector<uint8_t>> CameraAPI::get_rgb_image()
{
    if( const auto n = G->get_node(id); n.has_value())
    {
        auto &attrs = n.value().attrs();
        if (auto value = attrs.find("cam_rgb"); value != attrs.end())
            return value->second.byte_vec();
        else
        {
            qWarning() << __FUNCTION__ << "No rgb attribute found in node " << QString::fromStdString(n.value().name()) << ". Returning empty";
            return {};
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << "No camera node found in G. Returning empty";
        return {};
    }
}

std::optional<std::vector<float>> CameraAPI::get_depth_image()
{
    if( const auto n = G->get_node(id); n.has_value())
    {
        auto &attrs = n.value().attrs();
        if (auto value = attrs.find("cam_depth"); value != attrs.end())
        {
            const std::size_t SIZE = value->second.byte_vec().size() / sizeof(float);
            float *depth_array = (float *) value->second.byte_vec().data();
            std::vector<float> res{depth_array, depth_array + SIZE};
            return res;
        } else
        {
            qWarning() << __FUNCTION__ << "No depth attribute found in node " << QString::fromStdString(n.value().name())
                       << ". Returning empty";
            return {};
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << "No camera node found in G. Returning empty";
        return {};
    }
}

//std::optional<std::vector<float>> CameraAPI::get_existing_depth_image()
//{
//    auto &attrs = node.attrs();
//    if (auto value = attrs.find("cam_depth"); value != attrs.end())
//    {
//        const std::size_t SIZE = value->second.byte_vec().size() / sizeof(float);
//        float *depth_array = (float *) value->second.byte_vec().data();
//        std::vector<float> res{depth_array, depth_array + SIZE};
//        return res;
//    }
//    else
//    {
//        qWarning() << __FUNCTION__ << "No camera node found in G. Returning empty";
//        return {};
//    }
//}
//
//std::optional<std::tuple<float,float,float>> CameraAPI::get_existing_roi_depth(const Eigen::AlignedBox<float, 2> &roi)
//{
//    auto &attrs = node.attrs();
//    if (auto value = attrs.find("cam_depth"); value != attrs.end())
//    {
//        auto left = (int)roi.min().x(); auto bot = (int)roi.min().y();
//        auto right = (int)roi.max().x(); auto top = (int)roi.max().y();  // botom has higher numeric value. rows start in 0 up
//        if(left<right and bot>top)
//        {
//            float *depth_array = (float *) value->second.byte_vec().data();
//            auto size = (right - left) * (bot - top);
//            std::vector<float> values(size);
//            std::size_t k = 0;
//            for (int i = left; i < right; i++)
//                for (int j = top; j < bot; j++)
//                    values[k++] = depth_array[i * this->width + j];
//
//            auto mv = std::min(values.begin(), values.end());
//            auto Y = *mv * 1000;
//            auto X = (right - left) / 2 * Y / this->focal_x;
//            auto Z = (bot - top) / 2 * Y / this->focal_y;
//            qInfo() << size << X << Y << Z;
//            return std::make_tuple(X, Y, Z);
//        }
//        else
//        {
//            qWarning() << __FUNCTION__ << "Incorrect ROI dimensions l r t b: " << left << right << top << bot << ". Returning empty";
//            return {};
//        }
//    }
//    else
//    {
//        qWarning() << __FUNCTION__ << "No depth attribute found in node " << QString::fromStdString(node.name()) << ". Returning empty";
//        return {};
//    }
//}
//
/*
std::optional<std::reference_wrapper<const std::vector<uint8_t>>> CameraAPI::get_depth_image() const
{
    if( const auto n = G->get_node(id); n.has_value())
    {
        auto &attrs = n.value().attrs();
        if (auto value = attrs.find("cam_depth"); value != attrs.end())
        {
            return value->second.byte_vec();
        } else
        {
            qWarning() << __FUNCTION__ << "No depth attribute found in node " << QString::fromStdString(n.value().name())
                       << ". Returning empty";
            return {};
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << "No camera node found in G. Returning empty";
        return {};
    }
}*/
//std::optional<std::reference_wrapper<const std::vector<uint8_t>>> CameraAPI::get_existing_depth_image() const
//{
//    auto &attrs = node.attrs();
//    if (auto value = attrs.find("cam_depth"); value != attrs.end())
//    {
//        return value->second.byte_vec();
//    } else
//    {
//        qWarning() << __FUNCTION__ << "No depth attribute found in node " << QString::fromStdString(node.name()) << ". Returning empty";
//        return {};
//    }
//}

///
/// Computes the point clound [X,Y,X] in the target_frame_node coordinate system. Subsampling: 1,2,3.. means all, one of two, one of three, etc
///
std::optional<std::vector<std::tuple<float,float,float>>>  CameraAPI::get_pointcloud(const std::string& target_frame_node, unsigned short subsampling)
{
    if( const auto n = G->get_node(id); n.has_value())
    {
        auto &attrs = n.value().attrs();
        if (auto value = attrs.find("cam_depth"); value != attrs.end())  //in metres
        {
            if (auto width = attrs.find("cam_depth_width"); width != attrs.end())
            {
                if (auto height = attrs.find("cam_depth_height"); height != attrs.end())
                {
                    if (auto focal = attrs.find("cam_depth_focalx"); focal != attrs.end())
                    {
                        const std::vector<uint8_t> &tmp = value->second.byte_vec();
                        if (subsampling == 0 or subsampling > tmp.size())
                        {
                            qWarning("DSRGraph::get_pointcloud: subsampling parameter < 1 or > than depth size");
                            return {};
                        }
                        // cast to float
                        float *depth_array = (float *) value->second.byte_vec().data();
                        const int WIDTH = width->second.dec();
                        const int HEIGHT = height->second.dec();
                        int FOCAL = focal->second.dec();
                        FOCAL = (int) ((WIDTH / 2) / atan(0.52));  // ÑAPA QUITAR
                        int STEP = subsampling;
                        float depth, X, Y, Z;
                        int cols, rows;
                        std::size_t SIZE = tmp.size() / sizeof(float);
                        std::vector<std::tuple<float, float, float>> result(SIZE);
                        std::unique_ptr<InnerEigenAPI> inner_eigen;
                        if (!target_frame_node.empty())  // do the change of coordinate system
                        {
                            inner_eigen = G->get_inner_eigen_api();
                            for (std::size_t i = 0; i < SIZE; i += STEP)
                            {
                                //depth = depth_array[i];
                                cols = (i % WIDTH) - (WIDTH / 2);
                                rows = (HEIGHT / 2) - (i / WIDTH);
                                // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
                                Y = depth_array[i] * 1000;
                                X = cols * Y / FOCAL;
                                Z = rows * Y / FOCAL;
                                auto r = inner_eigen->transform(target_frame_node, Mat::Vector3d(X, Y, Z),
                                                                n.value().name()).value();
                                result[i] = std::make_tuple(r[0], r[1], r[2]);
                            }
                        } else
                            for (std::size_t i = 0; i < tmp.size() / STEP; i++)
                            {
                                cols = (i % WIDTH) - (WIDTH / 2);
                                rows = (HEIGHT / 2) - (i / WIDTH);
                                // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
                                Y = depth_array[i] * 1000;
                                X = cols * Y / FOCAL;
                                Z = rows * Y / FOCAL;
                                // we transform measurements to millimeters
                                result[i] = std::make_tuple(X, Y, Z);
                            }
                        return result;
                    } else
                    {
                        qWarning() << __FUNCTION__ << "No focal attribute found in node "
                                   << QString::fromStdString(n.value().name()) << ". Returning empty";
                        return {};
                    }
                } else
                {
                    qWarning() << __FUNCTION__ << "No HEIGHT attribute found in node "
                               << QString::fromStdString(n.value().name())
                               << ". Returning empty";
                    return {};
                }
            } else
            {
                qWarning() << __FUNCTION__ << "No WIDTH attribute found in node " << QString::fromStdString(n.value().name())
                           << ". Returning empty";
                return {};
            }
        } else
        {
            qWarning() << __FUNCTION__ << "No depth attribute found found in node " << QString::fromStdString(n.value().name())
                       << ". Returning empty";
            return {};
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << "No camera node found in G. Returning empty";
        return {};
    }
}
//
//std::optional<std::vector<std::tuple<float,float,float>>>  CameraAPI::get_existing_pointcloud(const std::string target_frame_node, unsigned short subsampling)
//{
//    auto &attrs = node.attrs();
//    if (auto value = attrs.find("cam_depth"); value != attrs.end())  //in metres
//    {
//        const std::vector<uint8_t> &tmp = value->second.byte_vec();
//        if (subsampling == 0 or subsampling > tmp.size())
//        {
//            qWarning("DSRGraph::get_pointcloud: subsampling parameter < 1 or > than depth size");
//            return {};
//        }
//        // cast to float
//        float *depth_array = (float *) value->second.byte_vec().data();
//        const int WIDTH = this->width;
//        const int HEIGHT = this->height;
//        int FOCAL = this->focal;
//        int STEP = subsampling;
//        float depth, X, Y, Z;
//        int cols, rows;
//        std::size_t SIZE = tmp.size() / sizeof(float);
//        std::vector<std::tuple<float, float, float>> result(SIZE);
//        std::unique_ptr<InnerEigenAPI> inner_eigen;
//        if (target_frame_node != "")  // do the change of coordinate system
//        {
//            inner_eigen = G->get_inner_eigen_api();  //MOVE TO A CLASS VARIABLE
//            for (std::size_t i = 0; i < SIZE; i += STEP)
//            {
//                //depth = depth_array[i];
//                cols = (i % WIDTH) - (WIDTH / 2);
//                rows = (HEIGHT / 2) - (i / WIDTH);
//                // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
//                Y = depth_array[i] * 1000;
//                X = cols * Y / FOCAL;
//                Z = rows * Y / FOCAL;
//                auto r = inner_eigen->transform(target_frame_node, Mat::Vector3d(X, Y, Z),
//                                                node.name()).value();
//                result[i] = std::make_tuple(r[0], r[1], r[2]);
//            }
//        } else  // compute in camera coordinate system
//            for (std::size_t i = 0; i < tmp.size() / STEP; i++)
//            {
//                depth = depth_array[i];
//                cols = (i % WIDTH) - 320;
//                rows = 240 - (i / WIDTH);
//                X = cols * depth / FOCAL * 1000;
//                Y = depth * 1000;
//                Z = rows * depth / FOCAL * 1000;
//                // we transform measurements to millimeters
//                result[i] = std::make_tuple(X, Y, Z);
//            }
//        return result;
//
//    } else
//    {
//        qWarning() << __FUNCTION__ << "No depth attribute found found in node " << QString::fromStdString(node.name()) << ". Returning empty";
//        return {};
//    }
//}

std::optional<std::vector<uint8_t>> CameraAPI::get_depth_as_gray_image() const
{
    if( const auto n = G->get_node(id); n.has_value())
    {
        auto &attrs = n.value().attrs();
        if (auto value = attrs.find("cam_depth"); value != attrs.end())
        {
            const std::vector<uint8_t> &tmp = value->second.byte_vec();
            float *depth_array = (float *) value->second.byte_vec().data();
            const auto STEP = sizeof(float);
            std::vector<std::uint8_t> gray_image(tmp.size() / STEP);
            for (std::size_t i = 0; i < tmp.size() / STEP; i++)
                gray_image[i] = (int) (depth_array[i] * 15);  // ONLY VALID FOR SHORT RANGE, INDOOR SCENES
            return gray_image;
        } else
        {
            qWarning() << __FUNCTION__ << "No depth attribute found in node " << QString::fromStdString(n.value().name())
                       << ". Returning empty";
            return {};
        };
    }
    else
    {
        qWarning() << __FUNCTION__ << "No camera node found in G. Returning empty";
        return {};
    }
}

std::optional<std::tuple<float,float,float>> CameraAPI::get_roi_depth(const std::vector<float> &depth, const Eigen::AlignedBox<float, 2> &roi)
{
    auto left = (int)roi.min().x(); auto bot = (int)roi.min().y();
    auto right = (int)roi.max().x(); auto top = (int)roi.max().y();  // botom has higher numeric value. rows start in 0 up
    if(left<right and bot>top)
    {
        auto size = (right - left) * (bot - top);
        std::vector<float> values(size);
        std::size_t k = 0;
        for (int i = left; i < right; i++)
            for (int j = top; j < bot; j++)
                values[k++] = depth[i * width + j];

        //auto mv = std::ranges::min(values);
        std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
        const auto mv = values[values.size()/2];  //median
        const auto Y = mv * 1000;
        const float cols = left + (right - left) / 2;
        const float rows = top + (bot - top) / 2;
        float X = (cols - this->width/2) * Y / this->focal_x;
        float Z = (this->height/2 - rows)  * Y /  this->focal_y;
        return std::make_tuple(X, Y, Z);
    }
    else
    {
        qWarning() << __FUNCTION__ << "Incorrect ROI dimensions l r t b: " << left << right << top << bot << ". Returning empty";
        return {};
    }
}