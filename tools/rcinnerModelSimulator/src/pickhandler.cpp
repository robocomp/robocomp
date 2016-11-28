/*
 * Copyright 2015 <copyright holder> <email>
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

#include "pickhandler.h"
#include <Ice/Exception.h>

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
	if( (ea.getEventType()  != osgGA::GUIEventAdapter::PUSH) or
		(ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
		return false;
	osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
	if (view) 
		pick(view,ea);
	return false;
};

void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
   osgUtil::LineSegmentIntersector::Intersections intersections;
   std::string name;
    
    if (view->computeIntersections(ea,intersections))
    { 
		RoboCompRCISMousePicker::Pick pick;
		osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();	
     	if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
	            name = hitr->nodePath.back()->getName();		          // the geodes are identified by name.
        else if (hitr->drawable.valid())
        		name = hitr->drawable->className();

		 osg::Vec3d p = hitr->getWorldIntersectPoint();
         pick.x = p.x();
		 pick.y = p.y();
		 pick.z = -p.z();
		 qDebug() << __FUNCTION__ << "Picking " << QString::fromStdString(name) << pick.x << pick.y << pick.z;
		try
		{   if( rcis_mousepicker_proxy )
				rcis_mousepicker_proxy->setPick( pick );
		}
		catch(const Ice::Exception &ex)
		{ std::cout << ex << "in " << __FUNCTION__ << std::endl;}   
    }
}


// void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
// {
//     osgUtil::LineSegmentIntersector::Intersections intersections;
//     std::string gdlist="";
//     
//     if (view->computeIntersections(ea,intersections))
//     {  
//         for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
//         {
//             std::ostringstream os;
// 			if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
// 	        {
//                 // the geodes are identified by name.
//                 os<<"Object \""<<hitr->nodePath.back()->getName()<<"\""<<std::endl;
//             }
//             else if (hitr->drawable.valid())
//             {
// 				os<<"Object \""<<hitr->drawable->className()<<"\""<<std::endl;
//             }
//             os<<"        local coords vertex("<< hitr->getLocalIntersectPoint()<<")"<<"  normal("<<hitr->getLocalIntersectNormal()<<")"<<std::endl;
// 			os<<"        world coords vertex("<< hitr->getWorldIntersectPoint()<<")"<<"  normal("<<hitr->getWorldIntersectNormal()<<")"<<std::endl;
//            const osgUtil::LineSegmentIntersector::Intersection::IndexList& vil = hitr->indexList;
//            for(unsigned int i=0;i<vil.size();++i)
//            {
//                 os<<"        vertex indices ["<<i<<"] = "<<vil[i]<<std::endl;
//            }
//            gdlist += os.str();
//         }
//        std::cout << gdlist << std::endl;
// 	   try
// 	   {
// 		   RoboCompRCISMousepicker::Pick pick;
// 		   pick.x = 
// 		   rcis_mousepicker->
// 	   }
// 	   catch(const Ice::Exception &ex ex)
// 	   {}
// 		   
//     }
// }
