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

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// Node
// ------------------------------------------------------------------------------------------------

Node::Node(QString id_, Node *parent_) : id(id_)
{
// 	im_fixed = true;
	im_parent = parent_;
	if (im_parent)
		im_level = im_parent->im_level+1;
	else
		im_level = 0;
	im_attributes.clear();
}



Node::~Node()
{
}



void Node::treePrint(QString s, bool verbose)
{
	printf("%s%s l(%d) [%d] %p %p\n", qPrintable(s), qPrintable(id), im_level, im_children.size(), this, im_parent);
	QSet<Node*>::iterator i;
	for (i=im_children.begin(); i!=im_children.end(); i++)
	{
		if (verbose)
			(*i)->print(verbose);
		(*i)->treePrint(s+QString("  "), verbose);
		
	}
}



void Node::setParent(Node *parent_)
{
	if( im_parent != NULL )
		im_parent->im_children.remove( this );
	
	im_parent = parent_;
	if( im_parent != NULL )
		im_parent->im_children.insert( this );
	
	im_level = im_parent->im_level+1;
}



void Node::addChild(Node *child)
{
	im_children.insert(child);
	child->setParent( this );
}



void Node::setFixed(bool f)
{
	im_fixed = f;
}



bool Node::isFixed()
{
	return im_fixed;
}



void Node::computeAbsolute()
{
	// Compute the absolute node position
	if( im_parent == NULL ) {
		im_absolute = im_pose;
	}
	else {
		im_absolute = im_parent->im_absolute * im_pose;
	}
	
	foreach(Node *i, im_children)
		i->computeAbsolute();
}



void Node::computeRelative()
{
	// Compute the absolute node position
	if( im_parent == NULL ) {
		im_pose = im_absolute;
	}
	else {
		im_pose = im_absolute * im_parent->im_absolute.invert();
// 		im_pose = im_parent->im_absolute.invert() * im_absolute;
	}
	
	foreach(Node *i, im_children)
		i->computeRelative();
}

}
