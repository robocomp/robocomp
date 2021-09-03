/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <innermodel/innermodeldraw.h>

void InnerModelDraw::addMesh_ignoreExisting(InnerModelViewer *innerViewer, QString item, QString base, QVec t, QVec r, QString path, QVec scale)
{
	InnerModelTransform *parent = dynamic_cast<InnerModelTransform*>(innerViewer->innerModel->getNode(base));
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	

	if (im->getNode(item) != NULL)
	{
		removeNode(innerViewer, item);
	}

	InnerModelMesh *mesh = im->newMesh (
		item,
		parent,
		path,
		scale(0), scale(1), scale(2),
		0,
		t(0), t(1), t(2),
		r(0), r(1), r(2));
	mesh->setScale(scale(0), scale(1), scale(2));
	parent->addChild(mesh);

	innerViewer->recursiveConstructor(mesh, innerViewer->mts[parent->id], innerViewer->mts, innerViewer->meshHash);
}

bool InnerModelDraw::setScale(InnerModelViewer *innerViewer, const QString item, float scaleX, float scaleY, float scaleZ)
{
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	InnerModelMesh *aux = dynamic_cast<InnerModelMesh*>(im->getNode(item));
	aux->setScale(scaleX, scaleY, scaleZ);
	return true;
}

bool InnerModelDraw::addJoint(InnerModelViewer* innerViewer, const QString item, const QString base, QVec t, QVec r, QString axis)
{
	if (axis == "")
	{
		axis = "z";
	}
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	InnerModelTransform *parent=dynamic_cast<InnerModelTransform *>(im->getNode(base));
	InnerModelJoint *jN = im->newJoint(item,
					   parent,
					   0,0,0,
					   0,0,0,
				           t(0), t(1), t(2),
					   r(0), r(1), r(2),
					   -1000, 1000,
				           0,
				           axis.toStdString() );
	parent->addChild (jN);
	innerViewer->recursiveConstructor(jN, innerViewer->mts[parent->id], innerViewer->mts, innerViewer->meshHash);
	return true;
}

bool InnerModelDraw::setPlaneTexture(InnerModelViewer *innerViewer, const QString item, QString texture)
{
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	InnerModelPlane *aux = dynamic_cast<InnerModelPlane*>(im->getNode(item));

	aux->texture=texture;
	bool constantColor = false;
	if (texture.size() == 7)
	{
		if (texture[0] == '#')
		{
			constantColor = true;
		}
	}
	if (not constantColor)
	{
	  osg::Image *image=NULL;
		image = osgDB::readImageFile(texture.toStdString());
		if (not image)
		{
			throw "Couldn't load texture.";
		}
		innerViewer->planesHash[aux->id]->image =image;
		innerViewer->planesHash[aux->id]->texture->setImage(image);
	}
	else
	{
		innerViewer->planesHash[aux->id]->planeDrawable->setColor(htmlStringToOsgVec4(texture));
	}
	return true;
}

bool InnerModelDraw::addTransform_ignoreExisting(InnerModelViewer *innerViewer, QString item, QString base /*, parametros aqui*/)
{
	if (innerViewer->innerModel->getNode(base) == NULL)
	{
		throw QString("parent doesn't exist");
	}

	if (innerViewer->innerModel->getNode(item) != NULL)
	{
		removeNode(innerViewer, item);
	}
	addTransform(innerViewer, item, base);
	return true;
}

bool InnerModelDraw::addTransform(InnerModelViewer *innerViewer, QString item, QString base)
{
	InnerModelNode *parent = innerViewer->innerModel->getNode(base);
	if (parent == NULL)
		return false;

	InnerModelNode *node = innerViewer->innerModel->getNode(item);
	if (node != NULL)
	{
		printf("%s already exists!\n", item.toStdString().c_str());
		return false;
	}

	InnerModelTransform *tr;
	try
	{
		tr = innerViewer->innerModel->newTransform(item, "static", parent, 0,0,0, 0,0,0);
		parent->addChild(tr);
		innerViewer->recursiveConstructor(tr, innerViewer->mts[parent->id], innerViewer->mts, innerViewer->meshHash);

		return true;
	}
	catch (QString err)
	{
		printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.toStdString().c_str());
		throw;
	}
}

bool InnerModelDraw::addPlane_ignoreExisting(InnerModelViewer *innerViewer, const QString &item, const QString &base, const QVec &p, const QVec &n, const QString &texture, const QVec &size)
{
	if (innerViewer->innerModel->getNode(item))
	{
		removeNode(innerViewer, item);
	}
	addPlane_notExisting(innerViewer, item, base, p, n, texture, size);

	return true;
}

bool InnerModelDraw::addPlane_notExisting(InnerModelViewer *innerViewer, const QString &item, const QString &base, const QVec &p, const QVec &n, const QString &texture, const QVec &size)
{
	InnerModelNode *parent = innerViewer->innerModel->getNode(base);
	if (parent == NULL)
	{
		//printf("%s: parent does not exist\n", __FUNCTION__);
		return false;
	}
	InnerModelPlane *plane = innerViewer->innerModel->newPlane(item, parent, texture, size(0), size(1), size(2), 1, n(0), n(1), n(2), p(0), p(1), p(2));
	parent->addChild(plane);
	innerViewer->recursiveConstructor(plane, innerViewer->mts[parent->id], innerViewer->mts, innerViewer->meshHash);
	return true;
}

void InnerModelDraw::drawLine(InnerModelViewer *innerViewer, QString name, QString parent, const QVec& normalVector, const QVec &center, float length, float width, QString texture)
{
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, name, parent, center, normalVector, texture, QVec::vec3(length, width, width));
}

void InnerModelDraw::drawLine2Points(InnerModelViewer *innerViewer, QString name, QString parent, const QVec& p1, const QVec& p2, float width, QString texture)
{
	QLine2D line( p1 , p2 );	
	float dl = (p1-p2).norm2();
	QVec center = p2 + ((p1 - p2)*(float)0.5);
	InnerModelDraw::drawLine(innerViewer, name, parent, line.getNormalForOSGLineDraw(), center, dl, width, "#0000ff");
}

/**
 * @brief Removes the object name from InnerModelViewer instance
 * 
 * @param innerViewer ...
 * @param name ...
 * @return void
 */
bool InnerModelDraw::removeObject(InnerModelViewer *innerViewer, QString name)
{
	if (innerViewer->innerModel->getNode(name))
	{
		removeNode(innerViewer, name);
		return true;
	}
	else
	{
		qDebug() << __FUNCTION__ << "Object " << name << "does not exist. Could not be removed";
		return false;
	}
}

bool InnerModelDraw::removeNode(InnerModelViewer *innerViewer, const QString &item)
{
	if (item=="root")
	{
		qDebug() << "Can't remove root elements" << item;
		return false;
	}

	InnerModelNode *node = innerViewer->innerModel->getNode(item);
	if (node == NULL)
	{
		qDebug() << "Can't remove not existing elements" << item;
		return false;
	}

	QStringList l;
	innerViewer->innerModel->getSubTree(node, &l);
	innerViewer->innerModel->removeSubTree(node, &l);

	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(QString n, l)
	{
		/// Replicate mesh removals
		if (innerViewer->meshHash.contains(n))
		{
			while (innerViewer->meshHash[n].osgmeshPaths->getNumParents() > 0)
				innerViewer->meshHash[n].osgmeshPaths->getParent(0)->removeChild(innerViewer->meshHash[n].osgmeshPaths);
			while(innerViewer->meshHash[n].osgmeshes->getNumParents() > 0)
				innerViewer->meshHash[n].osgmeshes->getParent(0)->removeChild(innerViewer->meshHash[n].osgmeshes);
			while(innerViewer->meshHash[n].meshMts->getNumParents() > 0)
				innerViewer->meshHash[n].meshMts->getParent(0)->removeChild(innerViewer->meshHash[n].meshMts);
			innerViewer->meshHash.remove(n);
		}
		/// Replicate transform removals
		if (innerViewer->mts.contains(n))
		{
 			while (innerViewer->mts[n]->getNumParents() > 0)
				innerViewer->mts[n]->getParent(0)->removeChild(innerViewer->mts[n]);
 			innerViewer->mts.remove(n);
		}
		/// Replicate plane removals
		if (innerViewer->planeMts.contains(n))
		{
			while(innerViewer->planeMts[n]->getNumParents() > 0)
				((osg::Group *)(innerViewer->planeMts[n]->getParent(0)))->removeChild(innerViewer->planeMts[n]);
			innerViewer->planeMts.remove(n);
			innerViewer->planesHash.remove(n);
		}
	}
	return true;
}

