#ifndef CAMERA_API
#define CAMERA_API

#include "dsr_api.h"
#include <qmat/QMatAll>
#include "../core/topics/IDLGraphPubSubTypes.h"

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
            QVec get_angles( const QVec & p) const;
            QVec get_angles_homogeneous( const QVec & p) const;
            float get_focal() const;
            float get_focal_x() const;
            float get_focal_y() const;
            int get_height() const;
            int get_size_in_bytes() const;
            int get_depth() const;
            QVec get_ray_homogeneous( const QVec & p) const;
            QVec get_ray(const QVec & p) const;
            int get_width() const;
            QMat polar_3D_to_camera(const QMat & p) const ;
            QVec project( const QVec & p) const;
            void set( T Fx, T Fy, T Ox, T Oy );
            void set_focal( const int f) const;
            void set_focal_x( const int fx);
            void set_focal_y( const int fy);
            QVec to_zero_center( const QVec &p) const;
            QVec to_cero_center_homogeneous( const QVec &p) const;

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