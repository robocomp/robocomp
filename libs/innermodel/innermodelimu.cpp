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

#include "innermodel/innermodelimu.h"

InnerModelIMU::InnerModelIMU(QString id_, uint32_t _port, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	port = _port;
}

void InnerModelIMU::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<imu id=\"" << id << "\" />\n";
}

void InnerModelIMU::print(bool verbose)
{
	if (verbose) printf("IMU.");
}

void InnerModelIMU::update()
{
	if (fixed)
	{
	}
	updateChildren();
}

InnerModelNode * InnerModelIMU::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelIMU *ret = new InnerModelIMU(id, port, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}
