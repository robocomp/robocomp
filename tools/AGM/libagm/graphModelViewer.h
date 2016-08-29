#include <QtGui>
#include <QtOpenGL/QGLWidget>

#include <osgQt/GraphicsWindowQt>
#ifndef Q_MOC_RUN
	#include <osgViewer/CompositeViewer>
	#include <osgViewer/ViewerEventHandlers>
	#include <osgGA/TrackballManipulator>

	#include <osg/Geode>
	#include <osg/ShapeDrawable>
	#include <osg/Material>
	#include <osg/Texture2D>
	#include <osgUtil/ShaderGen>
	#include <osgViewer/Viewer>
	#include <osgDB/ReadFile>
	#include <osgDB/WriteFile>
	#include <osg/Math>
	#include <osgText/Font3D>
	#include <osgText/Text>
	#include <osgText/Text3D>
#endif

#define RADIUS 100.
#define TIPSIZE 60.

#include "agm.h"

class GraphModelViewer;
class GraphModelEdge;

class SymbolNode : public osg::Group
{
friend class GraphModelEdge;
friend class GraphModelViewer;

public:
	SymbolNode(std::string _id, std::string _stype);
public:
	void setId(std::string str);
	void setType(std::string str);

	void setPosition(float x, float y, float z) { setPosition(osg::Vec3(x,y,z)); }
	void setPosition(osg::Vec3 np);

private:
	osg::Vec3 pos, vel;
	std::string stype;
	std::string id;

	osgText::Text *textId;
	osgText::Text *textType;
	osg::Billboard *billboard;
	osg::Drawable *sphere;

};


class GraphModelEdge : public osg::Group
{
friend class GraphModelViewer;
public:
	GraphModelEdge(std::string _src, std::string _dst, std::string _label, std::map<std::string, SymbolNode *> *_nodeMapId);

public:
	std::string src, dst;
	std::string label;
	std::map<std::string, SymbolNode *> *nodeMapId;

private:
	osg::Geode *geode;
	GraphModelViewer *viewer;
	osg::Quat quaternionFromInitFinalVector(const osg::Vec3 &initV, const osg::Vec3 &destV) const
	{
		osg::Vec3 vQuat = destV^initV;
		const double aQuat = acos(initV * destV);

		if (vQuat.length() > 0.00000001)
		{
			return osg::Quat(-aQuat, vQuat);
		}
		else
		{
			const double aQuat = atan2(destV[2], destV[0]);
			return osg::Quat(-aQuat+M_PIl/2., osg::Vec3(0, 1, 0));
		}
	}

	osg::Cylinder *line;
	osg::Cone *tip;
	osg::ShapeDrawable *lineDrawable;
	osg::ShapeDrawable *tipDrawable;
	osgText::Text *labelText;
	osg::Billboard *billboard;
	void relocate();
};



class GraphModelViewer : public QWidget, public osgViewer::CompositeViewer
{
Q_OBJECT
friend class GraphModelEdge;
public:
	GraphModelViewer(osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::CompositeViewer::SingleThreaded, QWidget *parent=NULL, bool autoUpdate=false);

	void addNode(std::string id, std::string stype);
	void removeNode(std::string id);
	void setNodePosition(std::string id, osg::Vec3 np);
	void addEdge(std::string src, std::string dst, std::string label);
	void removeEdge(std::string src, std::string dst, std::string label);

public slots:
	void animateStep();
	void update(const AGMModel::SPtr &w);

private:
	// Internal representation for nodes.
	struct GRNNodeInfo
	{
		std::string name;
		std::string type;
		int32_t identifier;
		std::vector<uint32_t> edges;
		std::vector<std::string> edgesNames;
	};
	std::vector<GRNNodeInfo> nodes;
	osg::Group *group;
	osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> gw;

	std::map<std::string, SymbolNode *> nodeMapId;
	std::vector<SymbolNode *> nodeVector;

	std::vector<GraphModelEdge *> edges;


	QWidget *addViewWidget(osgQt::GraphicsWindowQt *gw, osg::Node *scene);
	osgQt::GraphicsWindowQt *createGraphicsWindow(int x, int y, int w, int h, const std::string& name="", bool windowDecoration=false);
	virtual void paintEvent(QPaintEvent* event);

	void relocateEdges() { for (uint32_t i=0; i<edges.size(); i++) { edges[i]->relocate(); } }

	void updateStructure();

protected:
	AGMModel::SPtr model;
	QTimer timer;
};


