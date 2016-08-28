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

#ifndef INNERMODELCAMERA_H
#define INNERMODELCAMERA_H

#include <innermodel/innermodelnode.h>

class InnerModel;

class InnerModelCamera : public InnerModelNode
{
	public:
		InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModel *innermodel_, InnerModelNode *parent_= NULL);
		void print(bool verbose);
		void save(QTextStream &out, int tabs);
		void update();
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

		Cam camera;
		float width, height, focal;
		float getWidth()  const { return width; }
		float getHeight() const { return height; }
		float getFocal()  const { return focal; }
		float getSize()   const { return getWidth()*getHeight(); }
		
		QVec project(QString reference, QVec origVec, QString cameraId);
	
		InnerModel *innermodel;
};

#endif // INNERMODELCAMERA_H
