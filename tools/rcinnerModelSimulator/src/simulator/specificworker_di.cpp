#include "specificworker.h"
void SpecificWorker::di_setImage(const std::string& item, const std::string& texture)
{
	QMutexLocker locker(mutex);
	QString m="RoboCompDisplay::setImage()";
	printf("SETIMAGE %s: %s\n", item.c_str(), texture.c_str());
	InnerModelDisplay *aux = dynamic_cast<InnerModelDisplay*>(getNode(QString::fromStdString(item), m));

	// qDebug()<<"aux->texture"<<aux->texture<<"qstring"<<QString::fromStdString(texture);

	aux->texture = QString::fromStdString(texture);

	osg::Image *image=NULL;
	image = osgDB::readImageFile(texture);
	if (not image)
	{
		qDebug() << "Couldn't load texture:" << texture.c_str();
		throw "Couldn't load texture.";
	}
	// qDebug()<<__FILE__<<" "<<__LINE__;
	// imv->planesHash[aux->id]->image=image;
	// qDebug()<<__FILE__<<" "<<__LINE__;
	imv->planesHash[aux->id]->setImage(image);
	// qDebug()<<__FILE__<<" "<<__LINE__;
	// qDebug()<<"change aux->texture"<<aux->texture;
}
