#include <graphModelViewer.h>

#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>

int main(int argc, char **argv)
{
	osg::ArgumentParser arguments(&argc, argv);

#if QT_VERSION >= 0x050000
	// Qt5 is currently crashing and reporting "Cannot make QOpenGLContext current in a different thread" when the viewer is run multi-threaded, this is regression from Qt4
	osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
#else
	osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::CullDrawThreadPerContext;
#endif

	while (arguments.read("--SingleThreaded")) threadingModel = osgViewer::ViewerBase::SingleThreaded;
	while (arguments.read("--CullDrawThreadPerContext")) threadingModel = osgViewer::ViewerBase::CullDrawThreadPerContext;
	while (arguments.read("--DrawThreadPerContext")) threadingModel = osgViewer::ViewerBase::DrawThreadPerContext;
	while (arguments.read("--CullThreadPerCameraDrawThreadPerContext")) threadingModel = osgViewer::ViewerBase::CullThreadPerCameraDrawThreadPerContext;

	QApplication app(argc, argv);

	GraphModelViewer *graphViewer = new GraphModelViewer(threadingModel, NULL, true);


	std::string path = std::string(argv[1]);
	xmlDocPtr doc;
	if ((doc = xmlParseFile(path.c_str())) == NULL)
	{
		fprintf(stderr,"Document not parsed successfully. \n");
		exit(1);
	}
	xmlNodePtr cur;
	if ((cur = xmlDocGetRootElement(doc)) == NULL)
	{
		fprintf(stderr,"empty document\n");
		xmlFreeDoc(doc);
		exit(1);
	}

	if (xmlStrcmp(cur->name, (const xmlChar *) "AGMModel"))
	{
		fprintf(stderr,"document of the wrong type, root node != AGMModel");
		xmlFreeDoc(doc);
		exit(1);
	}
	for (cur=cur->xmlChildrenNode; cur!=NULL; cur=cur->next)
	{
		if ((xmlStrcmp(cur->name, (const xmlChar *)"text")))
		{
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"symbol")))
			{
				xmlChar *stype = xmlGetProp(cur, (const xmlChar *)"type");
				xmlChar *sid = xmlGetProp(cur, (const xmlChar *)"id");
				graphViewer->addNode((char *)sid, (char *)stype);
/*
				static uint32_t xpos = 0;
				static uint32_t ypos = 0;
				static uint32_t zpos = 0;
				graphViewer->setNodePosition((char *)sid, osg::Vec3(xpos, ypos, zpos));
				xpos += 800;
				ypos += 0;
				zpos += 0;
*/
				graphViewer->setNodePosition((char *)sid, osg::Vec3(qrand()%int(50*RADIUS), qrand()%int(50*RADIUS), qrand()%int(50*RADIUS)));
				xmlFree(sid);
				xmlFree(stype);
			}
			else if ((!xmlStrcmp(cur->name, (const xmlChar *)"link")))
			{
			}
			else if ((!xmlStrcmp(cur->name, (const xmlChar *)"comment")))
			{
			}
			else
			{
				printf("??? %s\n", cur->name);
			}
		}
	}
	if ((cur = xmlDocGetRootElement(doc)) == NULL)
	{
		fprintf(stderr,"empty document\n");
		xmlFreeDoc(doc);
		exit(1);
	}
	for (cur=cur->xmlChildrenNode; cur!=NULL; cur=cur->next)
	{
		if ((xmlStrcmp(cur->name, (const xmlChar *)"text")))
		{
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"symbol")))
			{
			}
			else if ((!xmlStrcmp(cur->name, (const xmlChar *)"link")))
			{
				xmlChar *srcn = xmlGetProp(cur, (const xmlChar *)"src");
				if (srcn == NULL) { printf("Link %s lacks of attribute 'src'.\n", (char *)cur->name); exit(-1); }
				xmlChar *dstn = xmlGetProp(cur, (const xmlChar *)"dst");
				if (dstn == NULL) { printf("Link %s lacks of attribute 'dst'.\n", (char *)cur->name); exit(-1); }
				xmlChar *label = xmlGetProp(cur, (const xmlChar *)"label");
				if (label == NULL) { printf("Link %s lacks of attribute 'label'.\n", (char *)cur->name); exit(-1); }
				graphViewer->addEdge((char *)srcn, (char *)dstn, (char *)label);
				xmlFree(srcn);
				xmlFree(dstn);
			}
			else if ((!xmlStrcmp(cur->name, (const xmlChar *)"comment")))
			{
			}
			else
			{
				printf("??? %s\n", cur->name);
			}
		}
	}

	graphViewer->setGeometry(100, 100, 800, 600);
	graphViewer->show();

	return app.exec();

}


