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

#ifndef INNERMODELLASER_H
#define INNERMODELLASER_H

#include <innermodel/innermodelnode.h>

class InnerModelLaser : public InnerModelNode
{
	public:
		InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModelNode *parent_=NULL);
		void save(QTextStream &out, int tabs);
		void print(bool verbose);
		void update();
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

		uint32_t port;
		uint32_t min, max;
		float angle;
		uint32_t measures;
		QString ifconfig;
};


#endif // INNERMODELLASER_H
