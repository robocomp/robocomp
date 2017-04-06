import sys, os, Ice

# Check that RoboComp has been correctly detected
ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"AGMWorldModel.ice")
import RoboCompAGMWorldModel

# AGM
sys.path.append('/usr/local/share/agm')
from AGGL import *

def fromInternalToIce(src):
	# Create new model
	dst = RoboCompAGMWorldModel.World(nodes=[], edges=[], version=0)
	dst.version = int(src.version)
	## Copy indices
	for nodeSrc in src.nodes.values():
		nodeDst = RoboCompAGMWorldModel.Node()
		nodeDst.nodeType = nodeSrc.sType
		try:
			nodeDst.nodeIdentifier = int(nodeSrc.name)
		except:
			nodeDst.nodeIdentifier = -1
		if nodeDst.nodeIdentifier == -1:
			print "Can't transform models containing nodes with invalid identifiers (type: " + nodeDst.nodeType + ").\n"
			sys.exit(-1)
		nodeDst.attributes = nodeSrc.attributes
		dst.nodes.append(nodeDst)

	## Copy links
	for srcLink in src.links:
		dstLink = RoboCompAGMWorldModel.Edge()
		dstLink.edgeType = srcLink.linkType
		try:
			dstLink.a = int(srcLink.a)
		except:
			dstLink.a = -1
		if dstLink.a == -1:
			print "Can't transform models containing edges linking invalid identifiers (type: "+dstLink.edgeType+").\n"
			sys.exit(-1)
		try:
			dstLink.b = int(srcLink.b)
		except:
			dstLink.b = -1
		if dstLink.b == -1:
			print "Can't transform models containing edges linking invalid identifiers (type: "+dstLink.edgeType+").\n"
			sys.exit(-1)

		dstLink.attributes = srcLink.attributes
		dst.edges.append(dstLink)

	return dst


def fromIceToInternal_model(src, ignoreInvalidEdges=False):
	dst = AGMGraph()
	knownNodes = dict()
	for srcNode in src.nodes:
		dst.addNode(0,0, str(srcNode.nodeIdentifier), srcNode.nodeType, srcNode.attributes)
		knownNodes[str(srcNode.nodeIdentifier)] = True
		#print srcNode.nodeIdentifier, str(srcNode.nodeIdentifier)
		if srcNode.nodeIdentifier == -1:
			raise Exception("Can't transform models containing nodes with invalid identifiers (type: "+src.nodes[i].nodeType+").\n")
			sys.exit(-1)

	for srcLink in src.edges:
		if srcLink.a == -1 or srcLink.b == -1:
			raise Exception("Can't transform models containing nodes with invalid identifiers (type: "+src.edges[i].edgeType+").\n")
			sys.exit(-1)
		edge = AGMLink(str(srcLink.a), str(srcLink.b), srcLink.edgeType,srcLink.attributes)
		if str(srcLink.a) in knownNodes and str(srcLink.b) in knownNodes:
			dst.links.append(edge)
		else:
			if not ignoreInvalidEdges:
				raise Exception('I was sent a model with an edge linkning a non-existing node.')
	dst.version = int(src.version)
	return dst

def fromIceToInternal_node(node):
	return AGMSymbol(str(node.nodeIdentifier), node.nodeType, [0,0], node.attributes)

def fromIceToInternal_edge(edge):
	return AGMLink(str(edge.a),str(edge.b),str(edge.edgeType), edge.attributes)


#bool AGMModelConverter::includeIceModificationInInternalModel(const RoboCompAGMWorldModel::Node &node, AGMModel::SPtr &world)
#{
	#for (uint32_t i=0; i<world->symbols.size(); ++i)
	#{
		#if (node.nodeType == world->symbols[i]->sType and node.nodeIdentifier == world->symbols[i]->identifier)
		#{
			#std::map<std::string, std::string>::const_iterator iter
			#for (iter = node.attributes.begin(); iter!=node.attributes.end(); iter++)
			#{
				#world->symbols[i]->attributes[iter->first] = iter->second
			#}
			#return true
		#}
	#}
	#return false
#}

#int32_t getIdFromString(char *sid, int32_t &lastVariableId, std::map <std::string, int32_t> &identifierMap)
#{
	#int32_t id
	#if ((sid[0] >= 'a' and sid[0] <= 'z') or (sid[0] >= 'A' and sid[0] <= 'Z'))
	#{
		#std::map <std::string, int32_t>::iterator iter = identifierMap.find(std::string(sid))
		#if (iter == identifierMap.end())
		#{
			#lastVariableId++
			#id = lastVariableId
			#identifierMap[std::string(sid)] = id
		#}
		#else
		#{
			#id = identifierMap[std::string(sid)]
		#}
	#}
	#else
	#{
		#id = atoi((char *)sid)
	#}
	#if (id<0)
	#{
		#fprintf(stderr, "AGMModels can't have negative identifiers (type: %s).\n", (char *)sid)
		#exit(-1)
	#}

	#return id
#}



