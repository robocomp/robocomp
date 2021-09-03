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

#include "innermodel/innermodellaser.h"
#include <innermodel/innermodel.h>

InnerModelLaser::InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModel *innermodel_, InnerModelNode *parent_) :  InnerModelNode(id_, parent_) , innermodel(innermodel_)
{
	
 #if FCL_SUPPORT==1
 	collisionObject = NULL;    //inherited from InnerModelNode
 #endif
	port = _port;
	min = _min;
	max = _max;
	measures = _measures;
	angle = _angle;
	ifconfig = _ifconfig;
	
	//qDebug() << __FUNCTION__ << id << port << min << max << angle << measures;
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
	InnerModelLaser *ret = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, innermodel, parent);
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

QVec InnerModelLaser::laserTo(const QString &dest, float r, float alpha)
{
	QVec p(3);
	p(0) = r * sin(alpha);
	p(1) = 0;
	p(2) = r * cos(alpha);
	return innermodel->transform(dest, p, this->id);
}

QVec InnerModelLaser::laserTo(const std::string &dest, float r, float alpha)
{
	QVec p(3);
	p(0) = r * sin(alpha);
	p(1) = 0;
	p(2) = r * cos(alpha);
	return innermodel->transformS(dest, p, this->id.toStdString());
}
