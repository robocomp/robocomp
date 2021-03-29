/*
 * Copyright 2016 <copyright holder> <email>
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
 * 
 */

#ifndef INNERMODELMESH_H
#define INNERMODELMESH_H

#include <innermodel/innermodelnode.h>
#include <osg/TriangleFunctor>
#include <osg/io_utils>
#include <osg/Geode>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>


#ifdef FCL_SUPPORT
struct IncludeTrianglesInFCL_functor
{
	IncludeTrianglesInFCL_functor()
	{
		vertices = NULL;
		triangles = NULL;
	}

	std::vector<fcl::Vec3f> *vertices;
	std::vector<fcl::Triangle> *triangles;
	osg::Matrix tm;

	void set(std::vector<fcl::Vec3f> *vertices_, std::vector<fcl::Triangle> *triangles_, osg::Matrix transformMatrix)
	{
		vertices = vertices_;
		triangles = triangles_;
		tm = transformMatrix;
	}
	void clear()
	{
		if (vertices == NULL or triangles == NULL)
		{
			fprintf(stderr, "IncludeTrianglesInFCL_functor not initialized!\n");
			throw "IncludeTrianglesInFCL_functor not initialized!";
		}
		vertices->clear();
		triangles->clear();
	}
	void operator() (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3)
	{
		if (vertices == NULL or triangles == NULL)
		{
			fprintf(stderr, "IncludeTrianglesInFCL_functor not initialized!\n");
			throw "IncludeTrianglesInFCL_functor not initialized!";
		}
		osg::Vec3 v1p = tm * v1;
		osg::Vec3 v2p = tm * v2;
		osg::Vec3 v3p = tm * v3;
		vertices->push_back(fcl::Vec3f(v1p.x(), v1p.y(), v1p.z()));
		vertices->push_back(fcl::Vec3f(v2p.x(), v2p.y(), v2p.z()));
		vertices->push_back(fcl::Vec3f(v3p.x(), v3p.y(), v3p.z()));
		triangles->push_back(fcl::Triangle(vertices->size()-3, vertices->size()-2, vertices->size()-1));
	}
	
	// TODO: Remove when support for FCL in ubuntu 18.04 is not needed anymore.
	void operator() (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool /* treatVertexDataAsTemporary */)
	{
		operator()(v1, v2, v3);
	}
};

class CalculateTriangles : public osg::NodeVisitor
{
	public:
		CalculateTriangles(std::vector<fcl::Vec3f> *vertices_, std::vector<fcl::Triangle> *triangles_) : NodeVisitor( NodeVisitor::TRAVERSE_ALL_CHILDREN )
		{
			vertices = vertices_;
			triangles = triangles_;
			transformMatrix.makeIdentity();
		}

		virtual void apply(osg::Geode &geode)
		{
			// Use an OSG triangle functor to gather the vertices and triangles
			std::vector<fcl::Vec3f> vs;
			std::vector<fcl::Triangle> ts;
			osg::TriangleFunctor<IncludeTrianglesInFCL_functor> tri;
			tri.set(&vs, &ts, transformMatrix);
			tri.clear();
			int D = geode.getNumDrawables();
			for (int d=0; d<D; d++)
			{
				geode.getDrawable(d)->accept(tri);
			}
			// Append new points
			vertices->insert(vertices->end(), vs.begin(), vs.end());
			for (uint t=0; t<ts.size(); t++)
			{
				ts[t].set(ts[t][0]+triangles->size(), ts[t][1]+triangles->size(), ts[t][2]+triangles->size());
			}
			triangles->insert(triangles->end(), ts.begin(), ts.end());
			// recursion
			traverse( geode );
		}

		virtual void apply(osg::MatrixTransform &node)
		{
			// update matrix
			transformMatrix *= node.getMatrix();
			// recursion
			traverse( node );
		}
	protected:
		// Pointers that we will be given and that we have to fill with the output data
		std::vector<fcl::Vec3f> *vertices;
		std::vector<fcl::Triangle> *triangles;
		// Transformation matrix
		osg::Matrix transformMatrix;
};
#endif

class InnerModelMesh : public InnerModelNode
{
	public:
			enum RenderingModes { NormalRendering=0, WireframeRendering=1};		
			InnerModelMesh(QString id_, QString meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable, InnerModelNode *parent_=NULL);
			InnerModelMesh(QString id_, QString meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable, InnerModelNode *parent_=NULL);
			void save(QTextStream &out, int tabs);
			void print(bool verbose);
			void update();
			void setScale(float x, float y, float z);
			bool normalRendering() const;
			bool wireframeRendering() const;
			virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

			RenderingModes render;
			QString meshPath;
			float scalex, scaley, scalez;
			float tx, ty, tz;
			float rx, ry, rz;
};
#endif // INNERMODELMESH_H



