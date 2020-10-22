#ifndef CAMERA_API
#define CAMERA_API

#include "../core/topics/IDLGraphPubSubTypes.h"
#include "../core/types/user_types.h"
#include <Eigen/Dense>

namespace DSR
{
    class DSRGraph;

    // The class is created with info from Node n. The id of the camera node is stored to provide new values from the rgb and depth streams.
    class CameraAPI
    {
        public:
            explicit CameraAPI(DSRGraph *G_, const DSR::Node &n);
            //explicit CameraAPI(DSRGraph *G_, const std::uint32_t id);
            //explicit CameraAPI(DSRGraph *G_, const std::string &name);

            std::optional<std::reference_wrapper<const std::vector<uint8_t>>> get_rgb_image() const;
            std::optional<std::vector<float>> get_depth_image(); //return a copy
            std::optional<std::reference_wrapper<const std::vector<uint8_t>>> get_depth_image() const;
            // computes the point clound [X,Y,X] in the target_frame_node coordinate system. Subsampling: 1,2,3.. means all, one of two, one of three, etc
            std::optional<std::vector<std::tuple<float,float,float>>>  get_pointcloud(const std::string& target_frame_node = "", unsigned short subsampling=1);
            std::optional<std::vector<uint8_t>> get_depth_as_gray_image() const;

            bool reload_camera(const DSR::Node &n);
            inline std::uint32_t  get_id() const { return id;};
            Eigen::Vector3d get_angles( const Eigen::Vector3d & p) const;
            Eigen::Vector3d get_angles_homogeneous( const Eigen::Vector3d & p) const;
            float get_focal() const;
            float get_focal_x() const;
            float get_focal_y() const;
            inline std::uint32_t get_height() const {return height;};
            int get_size_in_bytes() const;
            int get_depth() const;
            Eigen::Vector3d get_ray_homogeneous( const Eigen::Vector3d & p) const;
            Eigen::Vector3d get_ray(const Eigen::Vector3d & p) const;
            inline std::uint32_t get_width() const {return width;};
            Eigen::Matrix3d polar_3D_to_camera(const Eigen::Matrix3d& p) const ;
            Eigen::Vector2d project( const Eigen::Vector3d & p, int cx=-1, int cy=-1) const;
            //void set( T Fx, T Fy, T Ox, T Oy );
            void set_focal( float f);
            void set_focal_x( float fx);
            void set_focal_y( float fy);
            void set_width( std::uint32_t w);
            void set_height( std::uint32_t h);
            Eigen::Vector3d to_zero_center( const Eigen::Vector3d &p) const;
            Eigen::Vector3d to_cero_center_homogeneous( const Eigen::Vector3d &p) const;

        private:
            DSR::DSRGraph *G;
            std::uint32_t id;
            float focal_x;		        //!< Horizontal focus
            float focal_y;		        //!< Vertical focus
            float focal;
            float centre_x;		        //!< Horizontal position of imagen center in pixel coordinates.
            float centre_y;		        //!< Vertical position of imagen center in pixel coordinates.
            std::uint32_t width;		//!<
            std::uint32_t height;		//!<
            std::uint32_t depth;		//!<
            std::uint32_t cameraID;
    };
}

#endif