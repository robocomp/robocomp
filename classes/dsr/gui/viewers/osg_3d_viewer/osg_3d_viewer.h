/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DSR_TO_OSG_VIEWER_H
#define DSR_TO_OSG_VIEWER_H

#include <chrono>
#include <osg/ref_ptr>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Image>
#include <osg/LineSegment>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/Quat>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/TexMat>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/GraphicsWindow>
#include <QOpenGLWidget>
#include <QResizeEvent>
#include "../../../api/dsr_api.h"

using namespace std::chrono_literals;

namespace DSR
{
    enum CameraView { BACK_POV, FRONT_POV, LEFT_POV, RIGHT_POV, TOP_POV };
    class OSG3dViewer : public QOpenGLWidget
    {
        Q_OBJECT
        public:
            OSG3dViewer(std::shared_ptr<DSR::DSRGraph> G_, float scaleX, float scaleY, QWidget *parent=0);
			~OSG3dViewer();
        
        protected:  
            virtual void resizeEvent(QResizeEvent *e);
            void paintGL();
            virtual void resizeGL( int width, int height );
            // virtual void initializeGL();
            void mouseMoveEvent(QMouseEvent* event) override;        
            void mousePressEvent(QMouseEvent* event) override;
            void mouseReleaseEvent(QMouseEvent* event) override;
            void wheelEvent(QWheelEvent* event) override;
            bool event(QEvent* event) override;
            void initializeGL() override;

        private:
            std::shared_ptr<DSR::DSRGraph> G;
            osgGA::EventQueue* getEventQueue() const ;
            osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _mGraphicsWindow;
            osg::ref_ptr<osgViewer::Viewer> _mViewer;
            
            qreal m_scaleX, m_scaleY;
            osg::ref_ptr<osg::Group> root;

            //Hashes
            //using OsgTypes = std::variant<osg::, osg::MatrixTransform*>;
            // //std::map<std::int32_t, osg::Group*> osg_map;
            std::map<std::tuple<std::int32_t, std::int32_t>, osg::Group*> osg_map;
            //std::map<std::int32_t, std::int32_t>, osg::Group*> osg_transform;
            
            
            //std::map<std::string, IMVMesh> meshMap;
            osg::Vec3 QVecToOSGVec(const QVec &vec) const ;
            osg::Matrix QMatToOSGMat4(const RTMat &nodeB);
            osg::Vec4 htmlStringToOsgVec4(const std::string &color);
            osg::ref_ptr<osg::Group> createGraph();
            void setMainCamera(osgGA::TrackballManipulator *manipulator, CameraView pov) const;
            osgGA::TrackballManipulator* manipulator;

            void add_or_assign_node_slot(const Node &node);
            void add_or_assign_edge_slot(const Node &from, const Node& to);

            void add_or_assign_box(const Node &node, const Node& parent);
            void add_or_assign_mesh(const Node &node, const Node& parent);
            void add_or_assign_transform(const Node &from, const Node& to);
            void add_or_assign_person(const Node& node, const Node& parent);

            void traverse_RT_tree(const Node& node);
            void print_RT_subtree(const Node& node);
            void analyse_osg_graph(osg::Node *nd);
        public slots:
            void reload(QWidget* widget);
    };
};
#endif