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

#include "innermodellaser.h"

InnerModelLaser::InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	port = _port;
	min = _min;
	max = _max;
	measures = _measures;
	angle = _angle;
	ifconfig = _ifconfig;
}

void InnerModelLaser::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";

	out << "<laser id=\"" << id <<"\" port=\""<<port<<"\" min=\""<< QString::number(min,'g',10)<<"\" max=\""<<QString::number(max,'g',10)
	<<"\" measures=\""<<QString::number(measures,'g',10)<<"\" angle=\""<<QString::number(angle,'g',10)
	<<"\" ifconfig=\""<<ifconfig<< "\" />\n";
}

void InnerModelLaser::print(bool verbose)
{
	if (verbose) printf("LASER.");
}

void InnerModelLaser::update()
{
	if (fixed)
	{
	}
	updateChildren();
}

InnerModelNode * InnerModelLaser::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelLaser *ret = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}

