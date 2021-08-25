import datetime
from string import Template

import robocompdsl.dsl_parsers.parsing_utils as p_utils
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils
from robocompdsl.templates.common.templatedict import TemplateDict


REGENERATE_INNERMODEL = """
void SpecificWorker::regenerateInnerModelViewer()
{
    if (innerModelViewer)
    {
        osgView->getRootGroup()->removeChild(innerModelViewer);
    }

    innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
}   
    
"""


class specificworker_cpp(TemplateDict):
    def __init__(self, component):
        super(specificworker_cpp, self).__init__()
        self.component = component
        self['innermodelviewer_code'] = self.innermodelviewer_code()
        self['innermodel_and_viewer_attribute_init'] = self.innermodel_and_viewer_attribute_init()

    def innermodelviewer_code(self):
        result = ""
        if self.component.innermodelviewer:
            result += "#ifdef USE_QTGUI\n"
            result += "<TABHERE>innerModelViewer = NULL;\n"
            result += "<TABHERE>osgView = new OsgView(this);\n"
            result += "<TABHERE>osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;\n"
            result += "<TABHERE>osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));\n"
            result += "<TABHERE>osg::Vec3d center(osg::Vec3(0.,0.,-0.));\n"
            result += "<TABHERE>osg::Vec3d up(osg::Vec3(0.,1.,0.));\n"
            result += "<TABHERE>tb->setHomePosition(eye, center, up, true);\n"
            result += "<TABHERE>tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));\n"
            result += "<TABHERE>osgView->setCameraManipulator(tb);\n"
            result += "#endif\n"
        return result


    def innermodel_and_viewer_attribute_init(self):
        result = ""
        if self.component.innermodelviewer:
            result += "#ifdef USE_QTGUI\n"
            result += "<TABHERE>innerModel = std::make_shared<InnerModel>(); //InnerModel creation example\n"
            result += "<TABHERE>innerModelViewer = new InnerModelViewer (innerModel, \"root\", osgView->getRootGroup(), true);\n"
            result += "#endif\n"
        return result





