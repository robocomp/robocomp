/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>
#include <qmat/QMatAll>

enum NodeType { IMTransform, IMRotation, IMTranslation, IMMesh, IMPlane, IMCamera, IMIMU, IMLaser, IMRGBD, IMJoint };

struct WorkerNode
{
	QString id;
	NodeType type;
	QTreeWidgetItem *item;
};


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();
	void currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
private:
	int Period;
	QTimer timer;
 	OsgView *world3D;
	// InnerModel *innerModel;
	InnerModelViewer *imv;
	WorkerNode currentNode;
    QMessageBox msgBox;
    int flag;
    int render1;

	QMap<QString, WorkerNode> nodeMap;
	QMap<QTreeWidgetItem *, WorkerNode> nodeMapByItem;

	void showAvailableGroups();
	void interfaceConnections(bool enable);
    void newnodeConnections(bool enable);
	void showTransform(QString id);
	void showRotation(QString id);
	void showJoint(QString id);
	void showTranslation(QString id);
	void showMesh(QString id);
	void showPlane(QString id);
	void showCamera(QString id);

private:
	void fillNodeMap(InnerModelNode *node, QTreeWidgetItem *parent);

public slots:
	void saveButtonClicked();
	void resetButtonClicked();
	void openFile();
	void cameraChanged();
	void meshChanged();
	void planeChanged();
	void translationChanged();
	void rotationChanged();
	void jointChanged();
    void shownode();
    void makenode();
	void create_new_node(bool);
	void remove_current_node(bool);

private:
	InnerModel *innerModel;

};

#endif
