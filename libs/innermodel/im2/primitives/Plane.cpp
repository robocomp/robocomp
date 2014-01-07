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
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osgDB/ReadFile>

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// Plane
// ------------------------------------------------------------------------------------------------

Plane::Plane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, Node *parent_)
: Primitive(id_, parent_)
{
	im_normal = QVec::vec3(nx_, ny_, nz_);
	im_point = QVec::vec3(px_, py_, pz_);
	
	im_texture = texture_;
	im_width = width_;
	im_height = height_;
	im_depth = depth_;
	im_repeat = repeat_;
}



Plane::~Plane()
{
}



void Plane::print(bool verbose)
{
	if (verbose) im_normal.print(QString("Plane: ")+id);
}



void Plane::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<plane id=\"" << id << "\" texture=\"" << im_texture << "\" repeat=\"" << QString::number(im_repeat, 'g', 10) << "\" nx=\"" << QString::number(im_normal(0), 'g', 10) << "\" ny=\"" << QString::number(im_normal(1), 'g', 10) << "\" nz=\"" << QString::number(im_normal(2), 'g', 10) << "\" px=\"" << QString::number(im_point(0), 'g', 10) << "\" py=\"" << QString::number(im_point(1), 'g', 10) << "\" pz=\"" << QString::number(im_point(2), 'g', 10) << "\" />\n";
}



void Plane::updateValues(float nx_, float ny_, float nz_, float px_, float py_, float pz_)
{
	im_normal(0) = nx_;
	im_normal(1) = ny_;
	im_normal(2) = nz_;
	im_point(0) = px_;
	im_point(1) = py_;
	im_point(2) = pz_;
// 	im_fixed = true;
}






void Plane::updateTexture( uint8_t* image, int32_t width, int32_t height )
{
	osg_data = image;
	osg_width = width;
	osg_height = height;
	osg_dirty = true;
}



void Plane::initOSG( osg::Group* graphicsScene )
{
	osg_transform = new osg::MatrixTransform;
	graphicsScene->addChild( osg_transform );
	
	osg_geode = new osg::Geode;
	osg_transform->addChild( osg_geode );
	
	osg_data = NULL;
	
	bool constantColor = false;
	const std::string imagenEntrada = im_texture.toStdString();
	if ( (imagenEntrada.size() == 7) && (imagenEntrada[0] == '#') ) {
		constantColor = true;
	}

	// Open image
	osg_image = NULL;
	if (imagenEntrada.size()>0 and not constantColor) {
		if (imagenEntrada == "custom") {
			osg_image = new osg::Image();
		}
		else {
			osg_image = osgDB::readImageFile(imagenEntrada);
			if (not osg_image) {
				qDebug() << "Couldn't load texture:" << imagenEntrada.c_str();
				throw "Couldn't load texture.";
			}
		}
	}
	osg::TessellationHints* hints = new osg::TessellationHints;
	hints->setDetailRatio(2.0f);

	osg::Box* myBox = new osg::Box(QVecToOSGVec(QVec::vec3(0,0,0)), im_width, -im_height, im_depth);
	osg_planeDrawable = new osg::ShapeDrawable(myBox, hints);
	osg_planeDrawable->setColor(htmlStringToOsgVec4(QString::fromStdString(imagenEntrada)));

	osg_geode->addDrawable( osg_planeDrawable );

	if (not constantColor) {
		// Texture
		osg_texture = new osg::Texture2D;
		if (osg_image) {
			osg_texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			osg_texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			osg_texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			osg_texture->setImage(osg_image);
			osg_texture->setDataVariance(osg::Object::DYNAMIC);
			osg_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
			osg_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
			osg_texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			osg_texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			osg_texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			osg_texture->setTextureWidth(1);
			osg_texture->setTextureHeight(1);
		}
		// Material
		osg::Material *material = new osg::Material();
		material->setTransparency( osg::Material::FRONT_AND_BACK, 0.0 );
		material->setEmission(osg::Material::FRONT, osg::Vec4(0.8, 0.8, 0.8, 0.5));
		// Assign the material and texture to the plane
		osg::StateSet *sphereStateSet = osg_transform->getOrCreateStateSet();
		sphereStateSet->ref();
		sphereStateSet->setAttribute(material);
		sphereStateSet->setTextureMode(0,GL_TEXTURE_GEN_R,osg::StateAttribute::ON);
		sphereStateSet->setTextureAttributeAndModes(0, osg_texture, osg::StateAttribute::ON);
	}
	
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void Plane::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void Plane::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void Plane::postUpdate( const float elapsed )
{
	static uint8_t *backData = NULL;

	if ( osg_dirty ) {
		if (backData != osg_data) {
			osg_image->setImage(
				osg_width,
				osg_height,
				3,
				GL_RGB8,
				GL_RGB,
				GL_UNSIGNED_BYTE,
				osg_data,
				osg::Image::NO_DELETE,
				1);
		}
		else {
			osg_image->dirty();
		}

		osg_dirty = false;
		backData = osg_data;
	}
	osg_transform->setMatrix( QMatToOSGMat4(im_pose) );
	
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}

}
