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
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Point>

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// PointCloud
// ------------------------------------------------------------------------------------------------

PointCloud::PointCloud(QString id_, Node *parent_) : Primitive(id_, parent_)
{
}



PointCloud::~PointCloud()
{
}



void PointCloud::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<pointcloud id=\""<<id<<"\"/>\n";
}



void PointCloud::print(bool verbose)
{
	if (verbose) printf("Point Cloud: %s\n", qPrintable(id));
}



void PointCloud::initOSG( osg::Group* graphicsScene )
{
	osg_transform = new osg::MatrixTransform;
	graphicsScene->addChild( osg_transform );
	
	// Default point size
	osg_pointSize = 1.;
	
	// Vertex array
	osg_points = new osg::Vec3Array;
	osg_points->resize(64);
	
	/// Colors
	osg_colors = new osg::Vec4Array;
	osg_colors->resize(64);

	/// Index array
	osg_colorIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 4, 4>;
	osg_colorIndexArray->resize(64);
	
	/// DrawArrays
	osg_arrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, osg_points->size() );
	
	/// Geometry
	osg_geometry = new osg::Geometry();
	osg_geometry->setVertexArray( osg_points );
	osg_geometry->addPrimitiveSet( osg_arrays );
	osg_geometry->setColorArray( osg_colors );
	osg_geometry->setColorIndices( osg_colorIndexArray );
	osg_geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	
	/// Geode
	osg_geode = new osg::Geode;
	osg_geode->addDrawable( osg_geometry );
	
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void PointCloud::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void PointCloud::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void PointCloud::postUpdate( const float elapsed )
{
	osg_transform->setMatrix( QMatToOSGMat4(im_pose) );
	
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}



void PointCloud::reload( /*const osg::Vec3Array& points, const osg::Vec4Array& colors*/ )
{
	if ( osg_points->size() != osg_colors->size() )
		throw "points->size() != colors->size()";
	
// 	osg_points->resize( points.size() );
// 	osg_colors->resize( colors.size() );
// 	*osg_points = points;
// 	*osg_colors = colors;
	
	/// Geode 1
//	osg_geode->removeDrawable( osg_cloudGeometry);

	/// Arrays
	osg_colorIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 4, 4>;
	
	for (uint i=0; i<osg_points->size(); i++)
		osg_colorIndexArray->push_back(i);
	
	/// DrawArrays
	osg_arrays = new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, osg_points->size());
	
	/// Geometry
	//osg_geometry = new osg::Geometry();
	osg_geometry->setVertexArray( osg_points );
	//osg_cloudGeometry->addPrimitiveSet( osg_arrays );
	osg_geometry->getOrCreateStateSet()->setAttribute( new osg::Point(osg_pointSize), osg::StateAttribute::ON );
	osg_geometry->setColorArray( osg_colors );
	osg_geometry->setColorIndices( osg_colorIndexArray );
	osg_geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	
	/// Geode 2
	//osg_geode->addDrawable(osg_geometry);
}



float PointCloud::getPointSize() const
{
	return osg_pointSize;
}



void PointCloud::setPointSize(float p)
{
	osg_pointSize = p;
}

}
