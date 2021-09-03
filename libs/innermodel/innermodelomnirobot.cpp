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

#include "innermodel/innermodelomnirobot.h"

InnerModelOmniRobot::InnerModelOmniRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_, float noise_, bool collide_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
	#if FCL_SUPPORT==1
		collisionObject = NULL;   //inherited from InnerModelNode
	#endif
	port = port_;
	noise = noise_;
	collide = collide_;
}

InnerModelNode * InnerModelOmniRobot::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelOmniRobot *ret = new InnerModelOmniRobot(id, backtX, backtY, backtZ, backrX, backrY, backrZ, port, noise, (InnerModelTransform *)parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;

	ret->innerModel = parent->innerModel;
	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}

