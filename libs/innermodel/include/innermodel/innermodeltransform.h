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

#ifndef INNERMODELTRANSFORM_H
#define INNERMODELTRANSFORM_H

#include "innermodelnode.h"

		
class InnerModelTransform : public InnerModelNode
{
	public:
		friend class InnerModel;
		friend class InnerModelJoint;
		friend class InnerModelPointCloud;
		friend class InnerModelTouchSensor;
		friend class InnerModelLaser;
		friend class InnerModelDifferentialRobot;
		friend class InnerModelOmniRobot;
		friend class InnerModelPrismaticJoint;
		friend class InnerModelReader;

		InnerModelTransform(QString id_, QString engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, InnerModelNode *parent_=NULL);
		virtual ~InnerModelTransform();

		void print(bool verbose);
		void save(QTextStream &out, int tabs);
		void setUpdatePointers(float *tx_, float *ty_, float *tz_, float *rx_, float *ry_, float *rz_);
		void setUpdateTranslationPointers(float *tx_, float *ty_, float *tz_);
		void setUpdateRotationPointers(float *rx_, float *ry_, float *rz_);
		void update();
		void update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_);
		virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

		float *tx, *ty, *tz;
		float *rx, *ry, *rz;
		float mass;
		float backtX, backtY, backtZ;
		float backrX, backrY, backrZ;
		bool gui_translation, gui_rotation;
		QString engine;
		QMutex *mutex;
};
	

#endif // INNERMODELTRANSFORM_H
