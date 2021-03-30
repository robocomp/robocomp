//
// Created by robolab on 30/3/21.
//

#include <innermodel/innermodel.h>
#include <osgviewer/osgview.h>

int main()
{
    InnerModel* innerModel = new InnerModel();
    delete innerModel;
    OsgView* osg_view = new OsgView();
    delete osg_view;
    return 0;
}
