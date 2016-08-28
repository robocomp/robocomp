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

#ifndef INNERMODELRGBD_H
#define INNERMODELRGBD_H

#include <innermodel/innermodelcamera.h>

class InnerModel;

class InnerModelRGBD : public InnerModelCamera
{
	public:
		InnerModelRGBD(QString id_, float width, float height, float focal, float _noise, uint32_t _port, QString _ifconfig, InnerModel *innermdoel_, InnerModelNode *parent_=NULL);
		void save(QTextStream &out, int tabs);
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);
	
		float noise;
		uint32_t port;
		QString ifconfig;
};

#endif // INNERMODELRGBD_H
