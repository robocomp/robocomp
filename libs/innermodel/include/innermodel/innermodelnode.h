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

#ifndef INNERMODELNODE_H
#define INNERMODELNODE_H

// RoboComp includes
#include <qmat/QMatAll>

#ifdef FCL_SUPPORT
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/ccd/motion.h>
#include <fcl/BV/BV.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/traversal/traversal_node_bvh_shape.h>
#include <fcl/traversal/traversal_node_bvhs.h>
typedef fcl::BVHModel<fcl::OBBRSS> FCLModel;
typedef std::shared_ptr<FCLModel> FCLModelPtr;
#endif

class InnerModel;

class InnerModelNode : public RTMat
{
		friend class InnerModelCamera;
		friend class InnerModelRGBD;
		friend class InnerModelReader;

	public:
		InnerModelNode(QString id_, InnerModelNode *parent_=NULL);
		virtual ~InnerModelNode();
	
		struct AttributeType
		{
			QString type;
			QString value;
		};
		
		void treePrint(QString s, bool verbose=false);
		virtual void print(bool verbose) = 0;
		virtual void update() = 0;
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent) = 0;
		virtual void save(QTextStream &out, int tabs) = 0;
		void setParent(InnerModelNode *parent_);
		void addChild(InnerModelNode *child);
		void setFixed(bool f=true);
		bool isFixed();
		void updateChildren();
		
		//protected:
		QString id;
		int level;
		bool fixed;
		InnerModel *innerModel;
		InnerModelNode *parent;
		QList<InnerModelNode *> children;
		QHash<QString,AttributeType> attributes;

		// FCLModel
		bool collidable;
        #ifdef FCL_SUPPORT
			FCLModelPtr fclMesh;
			fcl::CollisionObject *collisionObject;
		#endif
};

#endif // INNERMODELNODE_H
