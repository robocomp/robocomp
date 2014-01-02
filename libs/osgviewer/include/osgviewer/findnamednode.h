#ifndef FINDNAMEDNODE_H
#define FINDNAMEDNODE_H

#include <osg/NodeVisitor>
/**
	@author authorname <authormail>
*/
class FindNamedNode : public osg::NodeVisitor
{
public:
    FindNamedNode( const std::string& name );
    ~FindNamedNode();
	virtual void apply( osg::Node& node );
	osg::Node* getNode();

protected:
    std::string _name;
    osg::ref_ptr<osg::Node> _node;

};

#endif
