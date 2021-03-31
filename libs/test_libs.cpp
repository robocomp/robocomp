//
// Created by robolab on 30/3/21.
//

#include <innermodel/innermodel.h>
#include <osgviewer/osgview.h>
#include <qapplication.h>



int main( int argc, char **argv )
{
    QApplication a( argc, argv );

    InnerModel* innerModel = new InnerModel();
    delete innerModel;

    OsgView* osg_view = new OsgView();
    osg_view->show();
    int rvalue = a.exec();
    delete osg_view;
    return rvalue;
}