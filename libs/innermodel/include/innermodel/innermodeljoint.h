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

#ifndef INNERMODELJOINT_H
#define INNERMODELJOINT_H

#include "innermodeltransform.h"

class InnerModelJoint : public InnerModelTransform
{
	public:
		InnerModelJoint();
		InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_=-INFINITY, float max_=INFINITY, uint32_t port_=0,std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL);

		void print(bool verbose);
		void save(QTextStream &out, int tabs);
		void update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_);
		float getAngle();
		float setAngle(float angle, bool force=false);
		QVec unitaryAxis();
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

		float backl;
		float backh;

		float min, max;
		float home;
		uint32_t port;
		std::string axis;
};

#endif // INNERMODELJOINT_H
