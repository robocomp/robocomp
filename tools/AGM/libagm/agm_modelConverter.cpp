#include <stdio.h>
#include <unistd.h>

#include <agm_modelConverter.h>
#include <agm_modelEdge.h>
#include <agm_modelPrinter.h>

#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>

#if ROBOCOMP_SUPPORT == 1
void AGMModelConverter::fromInternalToIce(const AGMModel::SPtr &src, RoboCompAGMWorldModel::World &dst)
{
	// remove dangling edges
	src->removeDanglingEdges();
	// copy version
	dst.version = src->version;
	// copy nodes
	dst.nodes.clear();
	for (uint32_t i=0; i<src->symbols.size(); ++i)
	{
		RoboCompAGMWorldModel::Node node;
		node.nodeType = src->symbols[i]->symbolType;
		node.nodeIdentifier = src->symbols[i]->identifier;
		if (node.nodeIdentifier == -1)
		{
			fprintf(stderr, "Can't transform models containing nodes with invalid identifiers (type: %s).\n", node.nodeType.c_str());
			AGMModelPrinter::printWorld(src);
			exit(-1);
		}
		node.attributes = src->symbols[i]->attributes;
		dst.nodes.push_back(node);
	}
	// copy edges
	dst.edges.clear();
	for (uint32_t i=0; i<src->edges.size(); ++i)
	{
		RoboCompAGMWorldModel::Edge edge;
		edge.edgeType = src->edges[i].linking;
		edge.a = src->edges[i].symbolPair.first;
		if (edge.a == -1)
		{
			fprintf(stderr, "Can't transform models containing edges linking invalid identifiers (type: %s).\n", edge.edgeType.c_str());
			AGMModelPrinter::printWorld(src);
			exit(-1);
		}
		edge.b = src->edges[i].symbolPair.second;
		if (edge.b == -1)
		{
			fprintf(stderr, "Can't transform models containing edges linking invalid identifiers (type: %s).\n", edge.edgeType.c_str());
			AGMModelPrinter::printWorld(src);
			exit(-1);
		}
		edge.attributes = src->edges[i]->attributes;
		dst.edges.push_back(edge);
	}
}


void AGMModelConverter::fromIceToInternal(const RoboCompAGMWorldModel::World &src, AGMModel::SPtr &dst)
{
	// nodes
	dst->symbols.clear();
	for (uint32_t i=0; i<src.nodes.size(); ++i)
	{
		dst->newSymbol(src.nodes[i].nodeIdentifier, src.nodes[i].nodeType, src.nodes[i].attributes);
		if (src.nodes[i].nodeIdentifier == -1)
		{
			fprintf(stderr, "Can't transform models containing nodes with invalid identifiers (type: %s).\n", src.nodes[i].nodeType.c_str());
			exit(-1);
		}
	}
	// edges
	dst->edges.clear();
	for (uint32_t i=0; i<src.edges.size(); ++i)
	{
		AGMModelEdge edge(src.edges[i].a, src.edges[i].b, src.edges[i].edgeType, src.edges[i].attributes);
		dst->edges.push_back(edge);
		if (src.edges[i].a == -1 or src.edges[i].b == -1)
		{
			fprintf(stderr, "Can't transform models containing nodes with invalid identifiers (type: %s).\n", src.edges[i].edgeType.c_str());
			exit(-1);
		}
	}
	// version
	dst->version = src.version;
	// last id
	dst->resetLastId();
}

void AGMModelConverter::fromInternalToIce(const AGMModelSymbol::SPtr &node, RoboCompAGMWorldModel::Node &dst)
{
	dst.nodeType = node->symbolType;
	dst.nodeIdentifier = node->identifier;
	dst.attributes = node->attributes;
}

// void AGMModelConverter::fromInternalToIce(const AGMModelEdge::SPtr &edge, RoboCompAGMWorldModel::Edge &dst)
// {
// 	dst.edgeType = edge->linking;
// 	dst.a = edge->symbolPair.first;
// 	dst.b = edge->symbolPair.second;
// 	dst.attributes = edge->attributes;
// }

void AGMModelConverter::fromInternalToIce(const AGMModelSymbol *node, RoboCompAGMWorldModel::Node &dst)
{
	dst.nodeType = node->symbolType;
	dst.nodeIdentifier = node->identifier;
	dst.attributes = node->attributes;
}

void AGMModelConverter::fromInternalToIce(const AGMModelEdge *edge, RoboCompAGMWorldModel::Edge &dst)
{
	dst.edgeType = edge->linking;
	dst.a = edge->symbolPair.first;
	dst.b =edge->symbolPair.second;
	dst.attributes = edge->attributes;
}

void AGMModelConverter::fromIceToInternal(const RoboCompAGMWorldModel::Node &node, AGMModelSymbol::SPtr &dst)
{
	dst->symbolType = node.nodeType;
	dst->attributes = node.attributes;
	dst->identifier = node.nodeIdentifier;
}

void AGMModelConverter::fromIceToInternal(const RoboCompAGMWorldModel::Edge &edge, AGMModelEdge &dst)
{
	dst->linking = edge.edgeType;
	dst->symbolPair.first = edge.a;
	dst->symbolPair.second =edge.b;
	dst->attributes = edge.attributes;
	
}


bool AGMModelConverter::includeIceModificationInInternalModel(const std::vector<RoboCompAGMWorldModel::Node> &nodes, AGMModel::SPtr &world)
{
	bool ret = true;
	for (auto n : nodes)
	{
		if (not includeIceModificationInInternalModel(n, world))
			ret = false;
	}
	return ret;
}

bool AGMModelConverter::includeIceModificationInInternalModel(const RoboCompAGMWorldModel::Node &node, AGMModel::SPtr &world)
{
	for (uint32_t i=0; i<world->symbols.size(); ++i)
	{
		if (node.nodeType == world->symbols[i]->symbolType and node.nodeIdentifier == world->symbols[i]->identifier)
		{
			std::map<std::string, std::string>::const_iterator iter;
			for (iter = node.attributes.begin(); iter!=node.attributes.end(); iter++)
			{
				world->symbols[i]->attributes[iter->first] = iter->second;
			}
			return true;
		}
	}
	return false;
}

bool AGMModelConverter::includeIceModificationInInternalModel(const std::vector<RoboCompAGMWorldModel::Edge> &edges, AGMModel::SPtr &world)
{
	bool ret = true;
	for (auto e : edges)
	{
		if (not includeIceModificationInInternalModel(e, world))
			ret = false;
	}
	return ret;
}

bool AGMModelConverter::includeIceModificationInInternalModel(const RoboCompAGMWorldModel::Edge &edge, AGMModel::SPtr &world)
{
	for (uint32_t i=0; i<world->edges.size(); ++i)
	{
		if (edge.edgeType == world->edges[i].linking  and edge.a == world->edges[i].symbolPair.first and edge.b == world->edges[i].symbolPair.second)
		{
			std::map<std::string, std::string>::const_iterator iter;
			for (iter = edge.attributes.begin(); iter!=edge.attributes.end(); iter++)
			{
				world->edges[i].attributes[iter->first] = iter->second;
			}
			return true;
		}
	}
	return false;
}

#endif

int32_t getIdFromString(char *sid, int32_t &lastVariableId, std::map <std::string, int32_t> &identifierMap)
{
	int32_t id;
	if ((sid[0] >= 'a' and sid[0] <= 'z') or (sid[0] >= 'A' and sid[0] <= 'Z'))
	{
		std::map <std::string, int32_t>::iterator iter = identifierMap.find(std::string(sid));
		if (iter == identifierMap.end())
		{
			lastVariableId++;
			id = lastVariableId;
			identifierMap[std::string(sid)] = id;
		}
		else
		{
			id = identifierMap[std::string(sid)];
		}
	}
	else
	{
		id = atoi((char *)sid);
	}
	if (id<0)
	{
		fprintf(stderr, "AGMModels can't have negative identifiers (type: %s).\n", (char *)sid);
		exit(-1);
	}

	return id;
}

void AGMModelConverter::fromXMLToInternal(const std::string path, AGMModel::SPtr &dst)
{
	// Empty the model
	dst->symbols.clear();
	dst->edges.clear();

	xmlDocPtr doc;
	if ((doc = xmlParseFile(path.c_str())) == NULL)
	{
		fprintf(stderr,"Document not parsed successfully. \n");
		return;
	}
	xmlNodePtr cur;
	if ((cur = xmlDocGetRootElement(doc)) == NULL)
	{
		fprintf(stderr,"empty document\n");
		xmlFreeDoc(doc);
		return;
	}
	if (xmlStrcmp(cur->name, (const xmlChar *) "AGMModel"))
	{
		fprintf(stderr,"document of the wrong type, root node != AGMModel");
		xmlFreeDoc(doc);
		return;
	}

	
	int32_t lastVariableId=1000;
	std::map <std::string, int32_t> identifierMap;

	for (cur=cur->xmlChildrenNode; cur!=NULL; cur=cur->next)
	{
		if ((xmlStrcmp(cur->name, (const xmlChar *)"text")))
		{
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"symbol")))
			{
				// Read ID and type
				xmlChar *stype = xmlGetProp(cur, (const xmlChar *)"type");
				xmlChar *sid = xmlGetProp(cur, (const xmlChar *)"id");
				int32_t id = getIdFromString((char *)sid, lastVariableId, identifierMap);

				// Read attributes
				std::map<std::string, std::string> attrMap;
				for (xmlNodePtr cur2=cur->xmlChildrenNode; cur2!=NULL; cur2=cur2->next)
				{
					if ((!xmlStrcmp(cur2->name, (const xmlChar *)"attribute")))
					{
						xmlChar *skey = xmlGetProp(cur2, (const xmlChar *)"key");
						if (skey == NULL) { printf("An atribute of %s lacks of attribute 'key'.\n", (char *)cur->name); exit(-1); }
						xmlChar *svalue = xmlGetProp(cur2, (const xmlChar *)"value");
						if (svalue == NULL) { printf("An atribute of %s lacks of attribute 'value'.\n", (char *)cur->name); exit(-1); }
						attrMap[std::string((char *)skey)] = std::string((char *)svalue);
						xmlFree(skey);
						xmlFree(svalue);
					}
					else if ((!xmlStrcmp(cur->name, (const xmlChar *)"text")))
					{
						printf("%s has invalid child %s\n", cur->name, cur2->name);
						exit(-1);
					}
				}

				dst->newSymbol(id, std::string((char*)stype), attrMap);

				xmlFree(sid);
				xmlFree(stype);
			}
			else if ((!xmlStrcmp(cur->name, (const xmlChar *)"link")))
			{
				xmlChar *srcn = xmlGetProp(cur, (const xmlChar *)"src");
				if (srcn == NULL) { printf("Link %s lacks of attribute 'src'.\n", (char *)cur->name); exit(-1); }
				xmlChar *dstn = xmlGetProp(cur, (const xmlChar *)"dst");
				if (dstn == NULL) { printf("Link %s lacks of attribute 'dst'.\n", (char *)cur->name); exit(-1); }
				xmlChar *label = xmlGetProp(cur, (const xmlChar *)"label");
				if (label == NULL) { printf("Link %s lacks of attribute 'label'.\n", (char *)cur->name); exit(-1); }
				
				int32_t srcN = getIdFromString((char *)srcn, lastVariableId, identifierMap);
				int32_t dstN = getIdFromString((char *)dstn, lastVariableId, identifierMap);
				
				// Read attributes
				std::map<std::string, std::string> attrMap;
				for (xmlNodePtr cur2=cur->xmlChildrenNode; cur2!=NULL; cur2=cur2->next)
				{
					if ((!xmlStrcmp(cur2->name, (const xmlChar *)"linkAttribute")))
					{
						xmlChar *skey = xmlGetProp(cur2, (const xmlChar *)"key");
						if (skey == NULL) { printf("An atribute of %s lacks of attribute 'key'.\n", (char *)cur->name); exit(-1); }
						xmlChar *svalue = xmlGetProp(cur2, (const xmlChar *)"value");
						if (svalue == NULL) { printf("An atribute of %s lacks of attribute 'value'.\n", (char *)cur->name); exit(-1); }
						attrMap[std::string((char *)skey)] = std::string((char *)svalue);
						xmlFree(skey);
						xmlFree(svalue);
					}
					else if ((!xmlStrcmp(cur->name, (const xmlChar *)"text")))
					{
						printf("%s has invalid child %s\n", cur->name, cur2->name);
						exit(-1);
					}
				}
				
				AGMModelEdge edge(srcN, dstN, (char *)label,attrMap);
				if (edge.symbolPair.first == -1 or edge.symbolPair.second == -1)
				{
					fprintf(stderr, "Can't create models with invalid identifiers (type: %s).\n", edge.linking.c_str());
					exit(-1);
				}
				
				dst->edges.push_back(edge);
				xmlFree(srcn);
				xmlFree(dstn);
			}
			else if ((!xmlStrcmp(cur->name, (const xmlChar *)"comment")))
			{
			}
			else
			{
				printf("??? %s (%d)\n", cur->name, __LINE__);
			}
		}
	}

	dst->version;
	dst->resetLastId();
}



