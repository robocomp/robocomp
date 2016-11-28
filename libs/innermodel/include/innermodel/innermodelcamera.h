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
		void updateValues(float width_, float height_, float focal_);
		
		QVec project(QString reference, QVec origVec);
		QVec project(const QVec &origVec);
		QVec backProject(const QString &cameraId, const QVec &coord) ;//const;
		void imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS);
		QVec anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS);
		QVec imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString to);
		QVec projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist);
		QVec horizonLine(QString planeId, QString cameraId, float heightOffset=0.);
		QMat getHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
		QMat getAffineHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
		QMat getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera);
		/// Stereo computations
		void updateStereoGeometry( const QString &firstCam, const QString & secondCam );
		QVec compute3DPointInCentral(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
		QVec compute3DPointInRobot(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
		QVec compute3DPointFromImageCoords(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);
		QVec compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);
		/// Frustrum
		struct TPlane { QVec n; float d; };
		struct TFrustrum { TPlane left; TPlane top; TPlane right; TPlane down; TPlane near; TPlane far;};
		TFrustrum frustrumLeft, frustrumThird, frustrumRight;
		
		mutable QMutex mutex;
		
		InnerModel *innermodel;
};

#endif // INNERMODELCAMERA_H
