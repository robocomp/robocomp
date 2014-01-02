#include <osgviewer/getworldcoorofnode.h>

GetWorldCoorOfNode::GetWorldCoorOfNode()
 : osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false)
{
	 wcMatrix= new osg::Matrixd();
}

GetWorldCoorOfNode::~GetWorldCoorOfNode()
{
}

void GetWorldCoorOfNode::apply(osg::Node & node)
{
	if (!done)
	{
		if ( 0 == node.getNumParents() ) // no parents
		{
			wcMatrix->set( osg::computeLocalToWorld(this->getNodePath()) );
			done = true;
		}
		traverse(node);
	}
}

osg::Matrix* GetWorldCoorOfNode::getMat()
{
	return wcMatrix;
}


