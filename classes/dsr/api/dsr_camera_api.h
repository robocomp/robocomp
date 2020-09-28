#ifndef CAMERA_API
#define CAMERA_API

#include "../core/topics/IDLGraphPubSubTypes.h"
#include "../core/types/user_types.h"
#include <Eigen/Dense>

namespace DSR
{
    class DSRGraph;
    class CameraAPI
    {
        public:
            explicit CameraAPI(DSRGraph *G_, const DSR::Node &n);
            //explicit CameraAPI(DSRGraph *G_, const std::uint32_t id);
            //explicit CameraAPI(DSRGraph *G_, const std::string &name);
            bool reload_camera(const DSR::Node &n);
            Eigen::Vector3d get_angles( const Eigen::Vector3d & p) const;
            Eigen::Vector3d get_angles_homogeneous( const Eigen::Vector3d & p) const;
            float get_focal() const;
            float get_focal_x() const;
            float get_focal_y() const;
            int get_height() const;
            int get_size_in_bytes() const;
            int get_depth() const;
            Eigen::Vector3d get_ray_homogeneous( const Eigen::Vector3d & p) const;
            Eigen::Vector3d get_ray(const Eigen::Vector3d & p) const;
            int get_width() const;
            Eigen::Matrix3d polar_3D_to_camera(const Eigen::Matrix3d& p) const ;
            Eigen::Vector2d project( const Eigen::Vector3d & p) const;
            //void set( T Fx, T Fy, T Ox, T Oy );
            void set_focal( const int f) const;
            void set_focal_x( const int fx);
            void set_focal_y( const int fy);
            Eigen::Vector3d to_zero_center( const Eigen::Vector3d &p) const;
            Eigen::Vector3d to_cero_center_homogeneous( const Eigen::Vector3d &p) const;

        private:
            DSR::DSRGraph *G;
            float focal_x;		//!< Horizontal focus
            float focal_y;		//!< Vertical focus
            float centre_x;		//!< Horizontal position of imagen center in pixel coordinates.
            float centre_y;		//!< Vertical position of imagen center in pixel coordinates.
            int width;			//!<
            int height;			//!<
            int depth;			//!<
    };
}

#endif