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
#include "specificworker.h"

/**
 * \brief Default constructor
 */
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	mutex = new QMutex();


	QGLFormat fmt;
	fmt.setDoubleBuffer(true);
	QGLFormat::setDefaultFormat(fmt);
	groupBox_2->hide();
	connect(openpushButton,SIGNAL(clicked()),this, SLOT(openFile()));
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
	connect(create_new_nodepushButton, SIGNAL(clicked(bool)), this, SLOT(create_new_node(bool)));
    connect(remove_current_nodepushButton, SIGNAL(clicked()), this, SLOT(remove_current_node()));
	connect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
	connect(reloadpushButton,SIGNAL(clicked()),this,SLOT(reload_same()));
	showMaximized();
}

/**
 * \brief Default destructor
 */
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{




	return true;
}

void SpecificWorker::fillNodeMap(InnerModelNode *node, QTreeWidgetItem *parent)
{
	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelCamera *camera;
	InnerModelTransform *transformation;
	InnerModelIMU *imu;
	InnerModelLaser *laser;
	InnerModelRGBD *rgbd;
	InnerModelJoint *joint;
	// InnerModelDisplay *display;

	QTreeWidgetItem *item = new QTreeWidgetItem(QTreeWidgetItem::Type);

	WorkerNode wnode;
	wnode.id = node->id;
	wnode.item = item;

	item->setText(0, node->id);
	if (not parent)
	{
		treeWidget->addTopLevelItem(item);
	}
	else
	{
		parent->addChild(item);
	}

	// Find out which kind of node are we dealing with
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
	{
		if ((joint = dynamic_cast<InnerModelJoint *>(node)))
			wnode.type = IMJoint;
		else if (transformation->gui_translation and transformation->gui_rotation)
			wnode.type = IMTransform;
		else if (transformation->gui_translation)
			wnode.type = IMTranslation;
		else if (transformation->gui_rotation)
			wnode.type = IMRotation;
		else
			qFatal("Void transformation node (%s)?\n", transformation->id.toStdString().c_str());

		nodeMap[wnode.id] = wnode;
		nodeMapByItem[item] = wnode;
		for(int i=0; i<node->children.size(); i++)
		{
			fillNodeMap(node->children[i], item);
		}
	}
	else if ((camera = dynamic_cast<InnerModelCamera *>(node)))
	{
		if ((rgbd = dynamic_cast<InnerModelRGBD *>(node)))
			wnode.type = IMRGBD;
		else
			wnode.type = IMCamera;
		nodeMap[wnode.id] = wnode;
		nodeMapByItem[item] = wnode;
	}
	else if ((plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		wnode.type = IMPlane;
		nodeMap[wnode.id] = wnode;
		nodeMapByItem[item] = wnode;
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)))
	{
		wnode.type = IMMesh;
		nodeMap[wnode.id] = wnode;
		nodeMapByItem[item] = wnode;
	}
	else if ((imu = dynamic_cast<InnerModelIMU *>(node)))
	{
		wnode.type = IMIMU;
		nodeMap[wnode.id] = wnode;
		nodeMapByItem[item] = wnode;
	}
	else if ((laser = dynamic_cast<InnerModelLaser *>(node)))
	{
		wnode.type = IMLaser;
		nodeMap[wnode.id] = wnode;
		nodeMapByItem[item] = wnode;
	}
	else
	{
		qDebug() << "InnerModelReader::InnerModelReader(): Error: Unknown type of node (see node id=\n" << node->id << "\")";
		throw "InnerModelReader::InnerModelReader(): Error: Unknown type of node";
	}
}


void SpecificWorker::compute()
{
    imv->update();
	world3D->autoResize();
	world3D->frame();
}


void SpecificWorker::currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{
	interfaceConnections(false);
	currentNode = nodeMapByItem[current];
	showAvailableGroups();
    highlightNode();
	interfaceConnections(true);
}

void SpecificWorker::highlightNode()
{
	InnerModelPlane *plane;
	if(prevNode!=NULL)
	{
		if ((plane = dynamic_cast<InnerModelPlane *>(prevNode)))
		{
			plane->texture = prevTexture;
            imv->update();
		}
	}

	prevNode = (InnerModelNode *)innerModel->getNode(currentNode.id);
	InnerModelNode *HighNode = (InnerModelNode *)innerModel->getNode(currentNode.id);

	if ((plane = dynamic_cast<InnerModelPlane *>(HighNode)))
	{
		prevTexture = plane->texture;
		plane->texture = "/home/robocomp/robocomp/files/osgModels/textures/blue.jpg";
        imv->update();
	}

}

void SpecificWorker::showAvailableGroups()
{
	// Set node type and id
	NodeType type = currentNode.type;
	lineEdit_nodeId->setText(currentNode.id);

	// Treat 'root' special read-only special case
	if (currentNode.id == "root")
	{
		translationGroup->hide();
		rotationGroup->hide();
		meshGroup->hide();
		planeGroup->hide();
		cameraGroup->hide();
		jointGroup->hide();
		lineEdit_nodeId->setEnabled(false);
		nodeType->setText("<b>root</b>");
		return;
	}
	else
	{
		lineEdit_nodeId->setEnabled(true);
		nodeType->setText("<b>unknown</b>");
	}

	// Enable or disable GUI parts depending on the node type
	// and update node data in the interfaze
	switch (type)
	{
		case IMTransform:
			translationGroup->show();
			rotationGroup->show();
			meshGroup->hide();
			planeGroup->hide();
			cameraGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>T</b>");
			showTransform(currentNode.id);
			break;
		case IMRotation:
			rotationGroup->show();
			translationGroup->hide();
			meshGroup->hide();
			planeGroup->hide();
			cameraGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>r</b>");
			showRotation(currentNode.id);
			break;
		case IMTranslation:
			translationGroup->show();
			rotationGroup->hide();
			meshGroup->hide();
			planeGroup->hide();
			cameraGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>t</b>");
			showTranslation(currentNode.id);
			break;
		case IMMesh:
			translationGroup->show();
			rotationGroup->show();
			meshGroup->show();
			planeGroup->hide();
			cameraGroup->hide();
			jointGroup->hide();
			showMesh(currentNode.id);
			nodeType->setText("<b>mesh</b>");
			break;
		case IMPlane:
			planeGroup->show();
			translationGroup->hide();
			rotationGroup->hide();
			meshGroup->hide();
			cameraGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>plane</b>");
			showPlane(currentNode.id);
			break;
		case IMCamera:
			cameraGroup->show();
			translationGroup->hide();
			rotationGroup->hide();
			meshGroup->hide();
			planeGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>camera</b>");
			showCamera(currentNode.id);
			break;
		case IMIMU:
			cameraGroup->hide();
			translationGroup->hide();
			rotationGroup->hide();
			meshGroup->hide();
			planeGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>imu</b>");
			break;
		case IMLaser:
			cameraGroup->hide();
			translationGroup->hide();
			rotationGroup->hide();
			meshGroup->hide();
			planeGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>laser</b>");
			break;
		case IMRGBD:
			cameraGroup->show();
			translationGroup->hide();
			rotationGroup->hide();
			meshGroup->hide();
			planeGroup->hide();
			jointGroup->hide();
			nodeType->setText("<b>rgbd</b>");
			showCamera(currentNode.id);
			break;
		case IMJoint:
			jointGroup->show();
			translationGroup->hide();
			rotationGroup->hide();
			meshGroup->hide();
			planeGroup->hide();
			cameraGroup->hide();
			nodeType->setText("<b>joint</b>");
			showJoint(currentNode.id);
	}
}


void SpecificWorker::showTransform(QString id)
{
	showTranslation(id);
	showRotation(id);
}

void SpecificWorker::showRotation(QString id)
{
	InnerModelTransform *t = (InnerModelTransform *)innerModel->getNode(id);
	rx->setValue(t->backrX);
	ry->setValue(t->backrY);
	rz->setValue(t->backrZ);
}

void SpecificWorker::showJoint(QString id)
{
	InnerModelJoint *j = (InnerModelJoint *)innerModel->getNode(id);
	jointAngle->setValue(j->getAngle());
}

void SpecificWorker::showTranslation(QString id)
{
	InnerModelTransform *t = (InnerModelTransform *)innerModel->getNode(id);
	tx->setValue(t->backtX);
	ty->setValue(t->backtY);
	tz->setValue(t->backtZ);
}

void SpecificWorker::showMesh(QString id)
{
	InnerModelMesh *m = (InnerModelMesh *)innerModel->getNode(id);
	if (m->render == InnerModelMesh::NormalRendering)
		renderMode->setCurrentIndex(0);
	else if (m->render == InnerModelMesh::WireframeRendering)
		renderMode->setCurrentIndex(1);
	else
		qFatal("Internal error: only Normal and Wireframe rendering modes are supported (index %d not valid).", renderMode->currentIndex());

	rx->setValue(m->rx);
	ry->setValue(m->ry);
	rz->setValue(m->rz);
	tx->setValue(m->tx);
	ty->setValue(m->ty);
	tz->setValue(m->tz);
	scalex->setValue(m->scalex);
	scaley->setValue(m->scaley);
	scalez->setValue(m->scalez);
	osgFile->setText(m->meshPath);
}

void SpecificWorker::showPlane(QString id)
{
	InnerModelPlane *p = (InnerModelPlane *)innerModel->getNode(id);
	px->setValue(p->point(0));
	py->setValue(p->point(1));
	pz->setValue(p->point(2));
	pnx->setValue(p->normal(0));
	pny->setValue(p->normal(1));
	pnz->setValue(p->normal(2));
	texture->setText(p->texture);
	rectangleWidth->setValue(p->width);
	rectangleHeight->setValue(p->height);
	textureSize->setValue(p->repeat);

}

void SpecificWorker::showCamera(QString id)
{
	InnerModelCamera *c = (InnerModelCamera *)innerModel->getNode(id);
	focal->setValue(c->camera.getFocal());
	cwidth->setValue(c->camera.getWidth());
	cheight->setValue(c->camera.getHeight());
}


void SpecificWorker::saveButtonClicked()
{
	QString fileName = QFileDialog::getSaveFileName(this,
			tr("Save XML"), "",
			tr("XML file (*.xml)"));
	if (fileName.isEmpty())
		return;
	else {
		innerModel->save(fileName);
	}

}

void SpecificWorker::resetButtonClicked()
{
}

void SpecificWorker::cameraChanged()
{
	InnerModelCamera *c = (InnerModelCamera *)innerModel->getNode(currentNode.id);
	c->camera.setSize(cwidth->value(), cheight->value());
	c->camera.setFocal(focal->value());
}

void SpecificWorker::meshChanged()
{
	InnerModelMesh *m = (InnerModelMesh *)innerModel->getNode(currentNode.id);
	printf("d %p dd %s\n", m, m->id.toStdString().c_str());
	if (renderMode->currentIndex() == 0)
		m->render = InnerModelMesh::NormalRendering;
	else if (renderMode->currentIndex() == 1)
		m->render = InnerModelMesh::WireframeRendering;
	else
		qFatal("Error: only Normal and Wireframe modes are supported (index %d not valid).", renderMode->currentIndex());
	m->meshPath = osgFile->text();
	m->scalex = scalex->value();
	m->scaley = scaley->value();
	m->scalez = scalez->value();
	qDebug() << "File " << m->meshPath;
	// 	qDebug() << "Scale " << m->scale;
    imv->update();
	// imv->reloadMesh(m->id);
}

void SpecificWorker::planeChanged()
{
    InnerModelPlane *m = (InnerModelPlane *)innerModel->getNode(currentNode.id);
	m->normal = QVec::vec3(pnx->value(), pny->value(), pnz->value());
	m->point = QVec::vec3(px->value(), py->value(), pz->value());
	m->width = rectangleWidth->value();
	m->height = rectangleHeight->value();
	m->texture = texture->text();
    m->repeat = textureSize->value();
    imv->update();
}

void SpecificWorker::translationChanged()
{
	NodeType type = currentNode.type;

	// Treat 'root' special read-only special case
	if (type == IMTransform or type == IMTranslation)
	{
		innerModel->updateTranslationValues(currentNode.id, tx->value(), ty->value(), tz->value());
	}
	else if (type == IMMesh)
	{
		InnerModelMesh *m = (InnerModelMesh *)innerModel->getNode(currentNode.id);
		m->tx = tx->value();
		m->ty = ty->value();
		m->tz = tz->value();
	}
	else
		qFatal("Internal error worker.cpp:%d\n", __LINE__);
}

void SpecificWorker::rotationChanged()
{
	NodeType type = currentNode.type;

	// Treat 'root' special read-only special case
	if (type == IMTransform or type == IMTranslation)
	{
		innerModel->updateRotationValues(currentNode.id, rx->value(), ry->value(), rz->value());
	}
	else if (type == IMMesh)
	{
		InnerModelMesh *m = (InnerModelMesh *)innerModel->getNode(currentNode.id);
		m->rx = rx->value();
		m->ry = ry->value();
		m->rz = rz->value();
	}
	else
		qFatal("Internal error worker.cpp:%d\n", __LINE__);
}


void SpecificWorker::jointChanged()
{
	printf("joint changed\n");
	if (currentNode.type == IMJoint)
	{
		// 		InnerModelJoint *j = (InnerModelJoint *)innerModel->getNode(currentNode.id);
		innerModel->updateRotationValues(currentNode.id, 0, 0, jointAngle->value());
	}
	else
		qFatal("Internal error worker.cpp:%d\n", __LINE__);
}

void SpecificWorker::shownode()
{
	if(Typea->currentText()=="transform")
	{
		translationGroup_2->show();
		rotationGroup_2->show();
		meshGroup_2->hide();
		planeGroup_2->hide();
		cameraGroup_2->hide();
		jointGroup_2->hide();
		massBox->show();
		portBox->hide();
		noiseBox->hide();
		laserBox->hide();
		Ifconfiga->hide();
	}
	else if(Typea->currentText()== "mesh")
	{
		translationGroup_2->show();
		rotationGroup_2->show();
		meshGroup_2->show();
		planeGroup_2->hide();
		cameraGroup_2->hide();
		jointGroup_2->hide();
		massBox->hide();
		portBox->hide();
		noiseBox->hide();
		laserBox->hide();
		Ifconfiga->hide();
	}
	else if(Typea->currentText()== "plane")
	{
		planeGroup_2->show();
		translationGroup_2->hide();
		rotationGroup_2->hide();
		meshGroup_2->hide();
		cameraGroup_2->hide();
		jointGroup_2->hide();
		massBox->hide();
		portBox->hide();
		noiseBox->hide();
		laserBox->hide();
		Ifconfiga->hide();
	}
	else if(Typea->currentText()== "camera")
	{
		cameraGroup_2->show();
		translationGroup_2->hide();
		rotationGroup_2->hide();
		meshGroup_2->hide();
		planeGroup_2->hide();
		jointGroup_2->hide();
		massBox->hide();
		portBox->hide();
		noiseBox->hide();
		laserBox->hide();
		Ifconfiga->hide();
	}
	else if(Typea->currentText()== "imu")
	{
		cameraGroup_2->hide();
		translationGroup_2->hide();
		rotationGroup_2->hide();
		meshGroup_2->hide();
		planeGroup_2->hide();
		jointGroup_2->hide();
		massBox->hide();
		portBox->show();
		noiseBox->hide();
		laserBox->hide();
		Ifconfiga->hide();
	}
	else if(Typea->currentText()== "laser")
	{
		cameraGroup_2->hide();
		translationGroup_2->hide();
		rotationGroup_2->hide();
		meshGroup_2->hide();
		planeGroup_2->hide();
		jointGroup_2->hide();
		massBox->hide();
		portBox->show();
		noiseBox->hide();
		laserBox->show();
		Ifconfiga->show();
	}
	else if(Typea->currentText()== "rgbd")
	{
		cameraGroup_2->show();
		translationGroup_2->hide();
		rotationGroup_2->hide();
		meshGroup_2->hide();
		planeGroup_2->hide();
		jointGroup_2->hide();
		massBox->hide();
		portBox->show();
		noiseBox->hide();
		laserBox->hide();
		Ifconfiga->show();
	}
	else if(Typea->currentText()== "joint")
	{
		jointGroup_2->show();
		translationGroup_2->hide();
		rotationGroup_2->hide();
		meshGroup_2->hide();
		planeGroup_2->hide();
		cameraGroup_2->hide();
		massBox->hide();
		portBox->hide();
		noiseBox->hide();
		laserBox->hide();
		Ifconfiga->hide();
	}
	else
	{
		qDebug()<< "type valid node";
	}
}

void SpecificWorker::makenode()
{
	InnerModelNode *par= (InnerModelNode *)innerModel->getNode(parenta->text());
	if (par==NULL)
	{
		msgBox.setText("Enter valid Parent Id");
		msgBox.exec();
	}

	else
	{
		InnerModelNode *check= (InnerModelNode *)innerModel->getNode(newid->text());

		if(check==NULL)
		{
			if(Typea->currentText()=="transform")
			{
				InnerModelTransform *newnode = (InnerModelTransform *)innerModel->newTransform(newid->text(), "static", par, tx_2->value(), ty_2->value(), tz_2->value(), rx_2->value(), ry_2->value(), rz_2->value(), massa->value());
				par->addChild(newnode);
				flag=0;
			}
			else if(Typea->currentText()== "mesh")
			{
				if (renderMode_2->currentIndex() == 0)
					render1 = 0;
				else if (renderMode_2->currentIndex() == 1)
					render1 = 1;
				InnerModelMesh *newnode = (InnerModelMesh *)innerModel->newMesh(newid->text(), par, osgFile_2->text(), scalex_2->value(), scaley_2->value(), scalez_2->value(), render1, tx->value(), ty->value(), tz->value(), rx->value(), ry->value(), rz->value(), 0);
				par->addChild(newnode);
				flag=0;
			}
			else if(Typea->currentText()== "plane")
			{
				InnerModelPlane *newnode = (InnerModelPlane *)innerModel->newPlane(newid->text(), par, texture_2->text(), rectangleWidth_2->value(), rectangleHeight_2->value()
						, dep->value(), textureSize_2->value(), pnx_2->value(), pny_2->value(), pnz_2->value()
						, px_2->value(), py_2->value(), pz_2->value(), 0);
				par->addChild(newnode);
				flag=0;
			}
			else if(Typea->currentText()== "camera")
			{
				InnerModelCamera *newnode = (InnerModelCamera *)innerModel->newCamera(newid->text(), par, cwidth_2->value(), cheight_2->value(), focal_2->value());
				par->addChild(newnode);
				flag=0;
			}
			else if(Typea->currentText()== "imu")
			{
				InnerModelIMU *newnode = (InnerModelIMU *)innerModel->newIMU(newid->text(), par,porta->value());
				par->addChild(newnode);
				flag=0;
			}
			else if(Typea->currentText()== "laser")
			{
				InnerModelLaser *newnode = (InnerModelLaser *)innerModel->newLaser(newid->text(), par, porta->value(), Mina->value(), Maxa->value(), anglea->value(), measurea->value(), Ifconfiga->text());
				par->addChild(newnode);
				flag=0;
			}
			else if(Typea->currentText()== "rgbd")
			{
				InnerModelRGBD *newnode = (InnerModelRGBD *)innerModel->newRGBD(newid->text(), par, cwidth_2->value(), cheight_2->value(), focal_2->value(), noisea->value(), porta->value(), Ifconfiga->text());
				par->addChild(newnode);
				flag=0;
			}
			else if(Typea->currentText()== "joint")
			{
				qDebug()<< "later";
			}
			else
			{
				msgBox.setText("Enter valid type");
				msgBox.exec();
				flag=1;
			}
			if(flag==0)
			{
				QString fileName = QFileDialog::getSaveFileName(this,
						tr("Save XML"), "",
						tr("XML file (*.xml)"));
				if (fileName.isEmpty())
					return;
				else {
					innerModel->save(fileName);
				}
				qDebug()<< "create new node " << newid->text();

			}
		}
		else
		{
			msgBox.setText("Node you entered already exist");
			msgBox.exec();
		}

		disconnect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
		treeWidget->clear();
		connect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
		fillNodeMap(innerModel->getNode("root"), NULL);
        imv->update();
	}

}


void SpecificWorker::interfaceConnections(bool enable)
{
	if (enable)
	{
		// Save / Reset
		connect(resetButton, SIGNAL(clicked()), this, SLOT(resetButtonClicked()));
		connect(saveButton, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
		// Camera-related
		connect(cheight, SIGNAL(valueChanged(double)), this, SLOT(cameraChanged()));
		connect(cwidth, SIGNAL(valueChanged(double)), this, SLOT(cameraChanged()));
		connect(focal, SIGNAL(valueChanged(double)), this, SLOT(cameraChanged()));
		// Mesh-related
		connect(osgFile, SIGNAL(editingFinished()), this, SLOT(meshChanged()));
		connect(scalex, SIGNAL(valueChanged(double)), this, SLOT(meshChanged()));
		connect(scaley, SIGNAL(valueChanged(double)), this, SLOT(meshChanged()));
		connect(scalez, SIGNAL(valueChanged(double)), this, SLOT(meshChanged()));
		// Plane-related
		connect(texture, SIGNAL(editingFinished()), this, SLOT(planeChanged()));
		connect(px, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(py, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(pz, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(pnx, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(pny, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(pnz, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(rectangleWidth, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(rectangleHeight, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		connect(textureSize, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		// Rotation-related
		connect(rx, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
		connect(ry, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
		connect(rz, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
		// Translation-related
		connect(tx, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
		connect(ty, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
		connect(tz, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
		// Joint-related
		connect(jointAngle, SIGNAL(valueChanged(double)), this, SLOT(jointChanged()));
	}
	else
	{
		// Save / Reset
		disconnect(resetButton, SIGNAL(clicked()), this, SLOT(resetButtonClicked()));
		disconnect(saveButton, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
		// Camera-related
		disconnect(cheight, SIGNAL(valueChanged(double)), this, SLOT(cameraChanged()));
		disconnect(cwidth, SIGNAL(valueChanged(double)), this, SLOT(cameraChanged()));
		disconnect(focal, SIGNAL(valueChanged(double)), this, SLOT(cameraChanged()));
		// Mesh-related
		disconnect(osgFile, SIGNAL(editingFinished()), this, SLOT(meshChanged()));
		disconnect(renderMode, SIGNAL(currentIndexChanged(int)), this, SLOT(meshChanged()));
		disconnect(scalex, SIGNAL(valueChanged(double)), this, SLOT(meshChanged()));
		disconnect(scaley, SIGNAL(valueChanged(double)), this, SLOT(meshChanged()));
		disconnect(scalez, SIGNAL(valueChanged(double)), this, SLOT(meshChanged()));
		// Plane-related
		disconnect(texture, SIGNAL(editingFinished()), this, SLOT(planeChanged()));
		disconnect(px, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(py, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(pz, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(pnx, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(pny, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(pnz, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(rectangleWidth, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(rectangleHeight, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		disconnect(textureSize, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
		// Rotation-related
		disconnect(rx, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
		disconnect(ry, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
		disconnect(rz, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
		// Translation-related
		disconnect(tx, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
		disconnect(ty, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
		disconnect(tz, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
		// Joint-related
		disconnect(jointAngle, SIGNAL(valueChanged(double)), this, SLOT(jointChanged()));
	}
}

void SpecificWorker::create_new_node(bool bul)
{
	newnodeConnections(false);
	groupBox_2->show();
	newid->show();
	Typea->show();
	parenta->show();
	cameraGroup_2->hide();
	translationGroup_2->hide();
	rotationGroup_2->hide();
	meshGroup_2->hide();
	planeGroup_2->hide();
	jointGroup_2->hide();
	massBox->hide();
	portBox->hide();
	noiseBox->hide();
	laserBox->hide();
	Ifconfiga->hide();
	newnodeConnections(true);
}

void SpecificWorker::newnodeConnections(bool enable)
{
	if(enable)
	{
		connect(Typea,SIGNAL(currentIndexChanged(int)),this,SLOT(shownode()));
		connect(savepushButton,SIGNAL(clicked()),this,SLOT(makenode()));
	}

	else
	{
		disconnect(Typea,SIGNAL(currentIndexChanged(int)),this,SLOT(shownode()));
		disconnect(savepushButton,SIGNAL(clicked()),this,SLOT(makenode()));
	}
}

void SpecificWorker::reload_same()
{
	if(File_reload.isNull())
	{
		msgBox.setText("Nothing to Reload");
		msgBox.exec();
	}
	else
	{

		world3D = new OsgView(frame);
		disconnect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
		treeWidget->clear();
		connect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
		innerModel = new InnerModel(File_reload.toStdString());
		fillNodeMap(innerModel->getNode("root"), NULL);
        imv = new InnerModelViewer(innerModel, "root", world3D->getRootGroup(),false);
		timer.start(Period);
	}
}


   void SpecificWorker::remove_current_node()
   {
    interfaceConnections(false);
    current_node= nodeMapByItem[treeWidget->currentItem()];
    disconnect(&timer, SIGNAL(timeout()), this, SLOT(compute()));

    innerModel->removeNode(current_node.id);
    qDebug() << "Removed" << current_node.id;

    world3D = new OsgView(frame);
    imv = new InnerModelViewer(innerModel, "root", world3D->getRootGroup(),false);
    connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));

    disconnect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
    treeWidget->clear();
    connect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
    fillNodeMap(innerModel->getNode("root"), NULL);
    prevNode = NULL;

    interfaceConnections(true);
   }


void SpecificWorker::openFile()
{
	QString fileName = QFileDialog::getOpenFileName(this,
			tr("Open XML"), "",
			tr("XML file (*.xml);;All Files (*)"));
	if (fileName.isEmpty())
		return;
	else {
		File_reload=fileName;
        world3D = new OsgView(frame);
		disconnect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
		treeWidget->clear();
		connect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
		innerModel = new InnerModel(fileName.toStdString());
		fillNodeMap(innerModel->getNode("root"), NULL);
        imv = new InnerModelViewer(innerModel, "root", world3D->getRootGroup(),false);
		timer.start(Period);
	}
}
