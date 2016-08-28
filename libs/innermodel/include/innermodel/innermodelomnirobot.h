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

#ifndef INNERMODELOMNIROBOT_H
#define INNERMODELOMNIROBOT_H

#include <innermodel/innermodeltransform.h>

class InnerModelOmniRobot : public InnerModelTransform
{
	public:
		InnerModelOmniRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, float noise=0, bool collide=false, InnerModelTransform *parent_=NULL);
	public:
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	uint32_t port;
	float noise;
	bool collide;
};


#endif // INNERMODELOMNIROBOT_H
