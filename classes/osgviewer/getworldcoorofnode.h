#ifndef GETWORLDCOOROFNODE_H
#define GETWORLDCOOROFNODE_H

#include <osg/NodeVisitor>
#include <osg/Transform>

/**
	@author authorname <authormail>
*/
class GetWorldCoorOfNode : public osg::NodeVisitor
{
public:
    GetWorldCoorOfNode();
    ~GetWorldCoorOfNode();
	virtual void apply( osg::Node &node);
	osg::Matrix* getMat();

private:
	bool done;
	osg::Matrix *wcMatrix;
};

#endif
