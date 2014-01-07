/*
 *    Copyright (C) 2010-2013 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../innermodel.h"

// OSG includes
#include <osg/MatrixTransform>

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// Camera
// ------------------------------------------------------------------------------------------------

Camera::Camera(
	QString id_,
	uint32_t port_,
	float width_,
	float height_,
	float focal_,
	QString ifconfig_,
	Node *parent_ )
: Sensor( id_, port_, parent_ )
{
	im_width = width_;
	im_height = height_;
	im_focal = focal_;
	
	im_camera = Cam( im_focal, im_focal, im_width/2., im_height/2.);
	im_camera.setSize(im_width, im_height);
	im_camera.print(id_);
}



Camera::~Camera()
{
}



void Camera::print(bool verbose)
{
	if (verbose) im_camera.print(QString("Camera: ")+id);
}



void Camera::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<camera id=\"" << id << "\" width=\"" << im_camera.getWidth() << "\" height=\"" << im_camera.getHeight() << "\" focal=\"" << QString::number(im_camera.getFocal(), 'g', 10) << "\" />\n";
}



void Camera::initOSG( osg::Group* graphicsScene )
{
	osg_transform = new osg::MatrixTransform;
	graphicsScene->addChild( osg_transform );
	
	if ( im_port ) {
		// Images
		osg_rgb = new osg::Image;
		osg_rgb->allocateImage( im_width, im_height, 1, GL_RGB, GL_UNSIGNED_BYTE );
		
		osg_depth = new osg::Image;
		osg_depth->allocateImage( im_width, im_height, 1, GL_DEPTH_COMPONENT,GL_FLOAT );
		
		// Viewer
		const double fov = 2. * atan2( im_height/2.0, im_focal );
		const double aspectRatio = im_width / im_height;
		const double zNear = 0.01;
		const double zFar = 10000.0;
		
		osg_camera = new osgViewer::Viewer();
		osg_camera->setSceneData( osg_transform );
		osg_camera->setUpViewInWindow( 0, 0, im_width, im_height );
		osg_camera->getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		osg_camera->getCamera()->attach( osg::Camera::COLOR_BUFFER, osg_rgb );
		osg_camera->getCamera()->attach( osg::Camera::DEPTH_BUFFER, osg_depth );
		osg_camera->getCamera()->setProjectionMatrix( ::osg::Matrix::perspective(fov*180./M_PIl, aspectRatio, zNear, zFar) );
		
		// Manipulator
		osg_manipulator = new osgGA::TrackballManipulator();
		osg_camera->setCameraManipulator( osg_manipulator );
		
// 		RTMat rt = graphicsScene->getInnerModel()->getTransformationMatrix( "root", id );
// 		osg_camera->getCameraManipulator()->setByMatrix( QMatToOSGMat4(rt) );
	}
	
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void Camera::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void Camera::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void Camera::postUpdate( const float elapsed )
{
	osg_transform->setMatrix( QMatToOSGMat4(im_pose) );
	
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}



CameraParams Camera::getParams() const
{
	CameraParams params;
	params.focal = im_focal;
	params.width = im_width;
	params.height = im_height;
	osg_camera->getCamera()->getProjectionMatrixAsPerspective (
		params.fovy,
		params.aspect,
		params.znear,
		params.zfar );
	return params;
}



const uint8_t* Camera::getColor() const
{
	return osg_rgb->data();
}



const float* Camera::getDepth() const
{
	return (const float*) osg_depth->data();
}

}
