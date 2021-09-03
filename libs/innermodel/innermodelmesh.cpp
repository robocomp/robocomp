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

#include "innermodel/innermodelmesh.h"


InnerModelMesh::InnerModelMesh(QString id_, QString meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable,  InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	InnerModelMesh(id_,meshPath_,scale,scale,scale,render_,tx_,ty_,tz_,rx_,ry_,rz_, collidable, parent_);
}

InnerModelMesh::InnerModelMesh(QString id_, QString meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	id = id_;
	render = render_;
	meshPath = meshPath_;
	scalex = scalex_;
	scaley = scaley_;
	scalez = scalez_;
	tx = tx_;
	ty = ty_;
	tz = tz_;
	rx = rx_;
	ry = ry_;
	rz = rz_;
	collidable = collidable_;

#if FCL_SUPPORT==1
	// Get to the OSG geode
	//osg::Node *osgnode_ = osgDB::readNodeFile(meshPath.toStdString());
	osg::ref_ptr<osg::Node> osgnode_ = osgDB::readNodeFile(meshPath.toStdString()); 
	if (not osgnode_) printf("Could not open: '%s'.\n", meshPath.toStdString().c_str());
	if (osgnode_ != NULL)
	{
		// Instanciate the vector of vertices and triangles (that's what we are looking for)
		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;
		CalculateTriangles calcTriangles(&vertices, &triangles);
		osgnode_->accept(calcTriangles);

// 		printf("id: %s\n", id.toStdString().c_str());
// 		printf("scale: %f %f %f\n", scalex, scaley, scalez);
// 		printf("points: %zu\n", vertices.size());
// 		printf("triangles: %zu\n", triangles.size());

		// Get the internal transformation matrix of the mesh
		RTMat rtm(rx, ry, rz, tx, ty, tz);
		// Transform each of the read vertices
		for (size_t i=0; i<vertices.size(); i++)
		{
			fcl::Vec3f v = vertices[i];
			const QMat v2 = (rtm * QVec::vec3(v[0]*scalex, v[1]*scaley, -v[2]*scalez).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
			vertices[i] = fcl::Vec3f(v2(0), v2(1), v2(2));
		}

// ////
// ////   UNCOMMENT THIS CODE TO GENERATE A POINTCLOUD OF THE POINTS IN THE MESHES
// ////
// std::ofstream outputFile;
// outputFile.open((id.toStdString()+".pcd").c_str());
// outputFile << "# .PCD v.7 - Point Cloud Data file format\n";
// outputFile << "VERSION .7\n";
// outputFile << "FIELDS x y z \n";
// outputFile << "SIZE 4 4 4\n";
// outputFile << "TYPE F F F\n";
// outputFile << "COUNT 1 1 1\n";
// outputFile << "WIDTH " << vertices.size() << "\n";
// outputFile << "HEIGHT 1\n";
// outputFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
// outputFile << "POINTS " << vertices.size() << "\n";
// outputFile << "DATA ascii\n";
// for (size_t i=0; i<vertices.size(); i++)
// {
// 	outputFile << vertices[i][0]/1000. << " " << vertices[i][1]/1000. << " " << vertices[i][2]/1000. << "\n";
// }
// outputFile.close();


		// Associate the read vertices and triangles vectors to the FCL collision model object
		fclMesh = FCLModelPtr(new FCLModel());
		fclMesh->beginModel();
		fclMesh->addSubModel(vertices, triangles);
		fclMesh->endModel();
		collisionObject = new fcl::CollisionObject(fclMesh);
		
	}
	else
	{
		QString error;
		error.sprintf("Failed to read mesh \"%s\" for collision support!\n", meshPath.toStdString().c_str());
		throw error;
	}
#endif
}

void InnerModelMesh::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<mesh id=\""<<id<<"\"" <<" file=\"" << meshPath 
	<< "\" scale=\"" << QString::number(scalex, 'g', 10) << ","<< QString::number(scaley, 'g', 10)<< ","<< QString::number(scalez, 'g', 10) 
	<< "\" tx=\"" << QString::number(tx, 'g', 10) << "\" ty=\"" << QString::number(ty, 'g', 10) << "\" tz=\"" << QString::number(tz, 'g', 10) 
	<< "\" rx=\"" << QString::number(rx, 'g', 10) << "\" ry=\"" << QString::number(ry, 'g', 10) << "\" rz=\"" << QString::number(rz, 'g', 10) 
	<<"\" collide=\""<< QString::number(collidable,'g',10)<< "\" />\n";
}

void InnerModelMesh::print(bool verbose)
{
	if (verbose) printf("Mesh: %s\n", qPrintable(id));
}


void InnerModelMesh::setScale(float x, float y, float z)
{
	scalex=x;
	scaley=y;
	scalez=z;
}

void InnerModelMesh::update()
{
	if (fixed)
	{
	}
	updateChildren();
}

bool InnerModelMesh::normalRendering() const
{
	return render == NormalRendering;
}

bool InnerModelMesh::wireframeRendering() const {
	return render == WireframeRendering;
}

InnerModelNode * InnerModelMesh::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelMesh *ret = new InnerModelMesh(id, meshPath, scalex, scaley, scalez, render, tx, ty, tz, rx, ry, rz, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;

#if FCL_SUPPORT==1
	// Associate the read vertices and triangles vectors to the FCL collision model object
	ret->fclMesh = FCLModelPtr(new FCLModel(*fclMesh.get()));
	ret->collisionObject = new fcl::CollisionObject(ret->fclMesh);
#endif


	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}

