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

#ifndef INNERMODELPRISMATICJOINT_H
#define INNERMODELPRISMATICJOINT_H

#include <innermodel/innermodeltransform.h>

class InnerModelPrismaticJoint : public InnerModelTransform
{
	public:
		friend class InnerModel;
		friend class InnerModelReader;		
		InnerModelPrismaticJoint(QString id_, float min_, float max_, float val_, float offset_, uint32_t port_=0, std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL);
		void print(bool verbose);
		void save(QTextStream &out, int tabs);
		void update();
		float getPosition();
		float setPosition(float v);
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

		float value, offset;
		float min, max;
		float home;
		uint32_t port;
		std::string axis;
};

#endif // INNERMODELPRISMATICJOINT_H
