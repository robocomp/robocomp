#include <osgviewer/findnamednode.h>

FindNamedNode::FindNamedNode(const std::string& name)
 : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ), _name( name ) 
{
}

FindNamedNode::~FindNamedNode(  )
{
}

void FindNamedNode::apply(osg::Node & node)
{
	if (node.getName() == _name) _node = &node;
	// Keep traversing the rest of the scene graph.
	traverse( node );
}

osg::Node * FindNamedNode::getNode()
{
	 return _node.get(); 
}


