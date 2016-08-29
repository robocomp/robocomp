#include "agm_model.h"

#include "agm_modelEdge.h"
#include <sstream>

#include <algorithm>
#include <list>

AGMModel::AGMModel()
{
	lastId = 0;
	version = 0;
}

AGMModel::~AGMModel()
{
	lastId = 0;
	version = 0;
	symbols.clear();
	edges.clear();
}

AGMModel::AGMModel(const AGMModel::SPtr &src)
{
	setFrom(*src);
}

AGMModel::AGMModel(const AGMModel &src)
{
	//printf("AGMModel::AGMModel(const AGMModel &src)\n");
// 	printf("new model (&): (%p)\n", this);
	setFrom(src);
}


AGMModel::AGMModel(const std::string path)
{
	version = 0;

	// Open file and make initial checks
	xmlDocPtr doc;
	if ((doc = xmlParseFile(path.c_str())) == NULL)
	{
		fprintf(stderr,"Can't read XML file - probably a syntax error. \n");
		exit(1);
	}
	xmlNodePtr root;
	if ((root = xmlDocGetRootElement(doc)) == NULL)
	{
		fprintf(stderr,"Can't read XML file - empty document\n");
		xmlFreeDoc(doc);
		exit(1);
	}
	if (xmlStrcmp(root->name, (const xmlChar *) "AGMModel"))
	{
		fprintf(stderr,"Can't read XML file - root node != AGMModel");
		xmlFreeDoc(doc);
		exit(1);
	}

	
	// Read symbols (just symbols, then links in other loop)
	for (xmlNodePtr cur=root->xmlChildrenNode; cur!=NULL; cur=cur->next)
	{
		if (xmlStrcmp(cur->name, (const xmlChar *)"symbol") == 0)
		{
			xmlChar *stype = xmlGetProp(cur, (const xmlChar *)"type");
			xmlChar *sid = xmlGetProp(cur, (const xmlChar *)"id");
			AGMModelSymbol::SPtr s = newSymbol(atoi((char *)sid), (char *)stype);
			xmlFree(sid);
			xmlFree(stype);

			for (xmlNodePtr cur2=cur->xmlChildrenNode; cur2!=NULL; cur2=cur2->next)
			{
				if (xmlStrcmp(cur2->name, (const xmlChar *)"attribute") == 0)
				{
					xmlChar *attr_key   = xmlGetProp(cur2, (const xmlChar *)"key");
					xmlChar *attr_value = xmlGetProp(cur2, (const xmlChar *)"value");
					s->setAttribute(std::string((char *)attr_key), std::string((char *)attr_value));
					xmlFree(attr_key);
					xmlFree(attr_value);
				}
				else if (xmlStrcmp(cur2->name, (const xmlChar *)"comment") == 0) { }           // coments are always ignored
				else if (xmlStrcmp(cur2->name, (const xmlChar *)"text") == 0) { }     // we'll ignore 'text'
				else { printf("unexpected tag inside symbol: %s\n", cur2->name); exit(-1); } // unexpected tags make the program exit
			}
		}
		else if (xmlStrcmp(cur->name, (const xmlChar *)"link") == 0) { }     // we'll ignore links in this first loop
		else if (xmlStrcmp(cur->name, (const xmlChar *)"text") == 0) { }     // we'll ignore 'text'
		else if (xmlStrcmp(cur->name, (const xmlChar *)"comment") == 0) { }  // coments are always ignored
		else { printf("unexpected tag #1: %s\n", cur->name); exit(-1); }      // unexpected tags make the program exit
	}
	
	// Read links
	for (xmlNodePtr cur=root->xmlChildrenNode; cur!=NULL; cur=cur->next)
	{
		if (xmlStrcmp(cur->name, (const xmlChar *)"link") == 0)
		{
			xmlChar *srcn = xmlGetProp(cur, (const xmlChar *)"src");
			if (srcn == NULL) { printf("Link %s lacks of attribute 'src'.\n", (char *)cur->name); exit(-1); }
			int a = atoi((char *)srcn);
			xmlFree(srcn);

			xmlChar *dstn = xmlGetProp(cur, (const xmlChar *)"dst");
			if (dstn == NULL) { printf("Link %s lacks of attribute 'dst'.\n", (char *)cur->name); exit(-1); }
			int b = atoi((char *)dstn);
			xmlFree(dstn);

			
			xmlChar *label = xmlGetProp(cur, (const xmlChar *)"label");
			if (label == NULL) { printf("Link %s lacks of attribute 'label'.\n", (char *)cur->name); exit(-1); }
			std::string edgeName((char *)label);
			xmlFree(label);

			std::map<std::string, std::string> attrs;
			for (xmlNodePtr cur2=cur->xmlChildrenNode; cur2!=NULL; cur2=cur2->next)
			{
				if (xmlStrcmp(cur2->name, (const xmlChar *)"linkAttribute") == 0)
				{
					xmlChar *attr_key   = xmlGetProp(cur2, (const xmlChar *)"key");
					xmlChar *attr_value = xmlGetProp(cur2, (const xmlChar *)"value");
					attrs[std::string((char *)attr_key)] = std::string((char *)attr_value);
					xmlFree(attr_key);
					xmlFree(attr_value);
				}
				else if (xmlStrcmp(cur2->name, (const xmlChar *)"comment") == 0) { }           // coments are always ignored
				else if (xmlStrcmp(cur2->name, (const xmlChar *)"text") == 0) { }     // we'll ignore 'text'
				else { printf("unexpected tag inside symbol: %s ==> %s\n", cur2->name,xmlGetProp(cur2, (const xmlChar *)"id") ); exit(-1); } // unexpected tags make the program exit
			}
			
			addEdgeByIdentifiers(a, b, edgeName, attrs);
		}
		else if (xmlStrcmp(cur->name, (const xmlChar *)"symbol") == 0) { }   // symbols are now ignored
		else if (xmlStrcmp(cur->name, (const xmlChar *)"text") == 0) { }     // we'll ignore 'text'
		else if (xmlStrcmp(cur->name, (const xmlChar *)"comment") == 0) { }  // comments are always ignored
		else { printf("unexpected tag #2: %s\n", cur->name); exit(-1); }      // unexpected tags make the program exit
	}
}

AGMModel &AGMModel::operator=(const AGMModel &src)
{
	//printf("AGMModel& AGMModel::operator=(const AGMModel &src)\n");
	setFrom(src);
	return *this;
}

void AGMModel::setFrom(const AGMModel &src)
{
	lastId = src.lastId;
	version = src.version;

	symbols.clear();
	for (uint32_t i=0; i<src.symbols.size(); ++i)
	{
		newSymbol(src.symbols[i]->identifier, src.symbols[i]->typeString(), src.symbols[i]->attributes);
	}

	edges.clear();
	for (uint32_t i=0; i<src.edges.size(); ++i)
	{
		AGMModelEdge edge(src.edges[i].symbolPair.first, src.edges[i].symbolPair.second, src.edges[i].linking,src.edges[i].attributes);
		edges.push_back(edge);
	}

	resetLastId();
}

void AGMModel::resetLastId()
{
	int64_t maxId = 0;
	for (uint32_t i=0; i<symbols.size(); ++i)
	{
		if (symbols[i]->identifier >= maxId)
		{
			maxId = symbols[i]->identifier + 1;
		}
	}
	lastId = maxId;
}

void AGMModel::clear()
{
	lastId = 0;
	version = 0;
	symbols.clear();
	edges.clear();
}


int32_t AGMModel::numberOfEdges() const
{
	return edges.size();
}

int32_t AGMModel::numberOfSymbols() const
{
	return symbols.size();
}


int32_t AGMModel::getIdentifierByName(std::string name) const
{
	for (uint32_t i=0; i<symbols.size(); ++i)
	{
		if (symbols[i]->toString() == name)
		{
			return symbols[i]->identifier;
		}
	}
	return -1;
}


int32_t AGMModel::insertSymbol(AGMModelSymbol::SPtr s)
{
	symbols.push_back(s);
	return (int)symbols.size();
}


int32_t AGMModel::indexOfSymbol(const AGMModelSymbol::SPtr &value, int32_t from) const
{
	for (uint32_t i=from; i<symbols.size(); ++i)
	{
		if (symbols[i] == value)
		{
			return i;
		}
	}
	return -1;
}

int32_t AGMModel::indexOfFirstSymbolByValues(const AGMModelSymbol &value, int32_t from) const
{
	for (uint32_t i=from; i<symbols.size(); ++i)
	{
		AGMModelSymbol::SPtr p = symbols[i];
		AGMModelSymbol no_ptr = *(p.get());
		if (no_ptr == value)
		{
			return i;
		}
	}
// 	fprintf(stderr, "AGMModel::indexOfFirstSymbolByValues: no equal \"%s\"   %d\n", value.symbolType.c_str(), __LINE__);
// 	printf("%d\n", __LINE__);
	return -1;
}


int32_t AGMModel::indexOfFirstSymbolByType(const std::string &value, int32_t from) const
{
	for (uint32_t i=from; i<symbols.size(); ++i)
	{
		if (symbols[i]->symbolType == value)
		{
			return i;
		}
	}
// 	fprintf(stderr, "AGMModel::indexOfFirstSymbolByType: \"%s\", (%d)\n", value.c_str(), from);
// 	printf("%d\n", __LINE__);
	return -1;
}



#define SPLIT
#ifdef SPLIT
// Tenemos que usar el generatePDDLProblem con el tipado
#else
// Tenemos que usar el generatePDDLProblem sin el tipado
#endif

std::string AGMModel::generatePDDLProblem(const AGMModel::SPtr &target, int32_t unknowns, std::string domainName, std::string problemName) const
{
	for (uint32_t i=0; i<domainName.size(); ++i)
	{
		if (domainName[i] == '.')
		{
			domainName.resize(i);
			break;
		}
	}

	for (uint32_t i=0; i<problemName.size(); ++i)
	{
		if (problemName[i] == '.')
		{
			problemName.resize(i);
			break;
		}
	}

	std::ostringstream stringStream;
	if (target->symbols.size() == 0) return "";

	// H E A D E R
	stringStream << "(define (problem " << problemName << ")\n";
	stringStream << "\n";

	// D O M A I N
	stringStream << "	(:domain AGGL )\n";

	// D E C L A R E   O B J E C T S
	stringStream << "	(:objects\n";
	std::list <std::string> originalObjects;
	// Symbols that are present in the original model
	for (uint32_t s=0; s<symbols.size(); ++s)
	{
		originalObjects.push_back(symbols[s]->toString());
		stringStream << "		" << symbols[s]->toString() << " - tipo \n";
	}

	// Symbols that are only in the target model (not in the original model)
	std::list <std::string> targetObjects;
	for (uint32_t s=0; s<target->symbols.size(); ++s)
	{
		bool found = false;
		for (std::list<std::string>::iterator it=originalObjects.begin(); it!=originalObjects.end(); ++it)
		{
			if (*it == target->symbols[s]->toString())
			{
				found = true;
				break;
			}
		}
		if (not found)
		{
			targetObjects.push_back(target->symbols[s]->toString());
// 			unknowns++;
// 			stringStream << "		" << target->symbols[s]->toString() << "\n";
		}
	}

	// Unknown symbols that allow us to include new stuff in the model
	std::list <std::string> unknownObjectsVec;
	for (int32_t u=0; u<unknowns; ++u)
	{
		stringStream << "		unknown_" << u << " - tipo \n";
		std::ostringstream sstr;
		sstr << "unknown_" << u;
		unknownObjectsVec.push_back(sstr.str());
	}
	stringStream << "	)\n";
	stringStream << "\n";

	// I N I T I A L   W O R L D
	stringStream << "	(:init\n";
	// Initial cost
	//stringStream << "		(= (total-cost) 0)\n";
	// Unknown temporary objects we are going to artificially inject
	if (unknowns>0)
		stringStream << "		(firstunknown unknown_0)\n";
	for (int32_t u=1; u<unknowns; ++u)
	{
		stringStream << "		(unknownorder unknown_" << u-1 << " unknown_" << u << ")\n";
	}
	// Set not='s
	std::vector<std::string> allObjects;
	allObjects.insert(allObjects.end(),   originalObjects.begin(),   originalObjects.end());
	//allObjects.insert(allObjects.end(),     targetObjects.begin(),     targetObjects.end());
	allObjects.insert(allObjects.end(), unknownObjectsVec.begin(), unknownObjectsVec.end());
	bool useDiff = false;
	for (uint32_t ind1=0; ind1<allObjects.size() and useDiff; ind1++)
	{
		for (uint32_t ind2=0; ind2<allObjects.size(); ind2++)
		{
			if (ind1 != ind2)
			{
				stringStream << "		(diff " << allObjects[ind1] << " " << allObjects[ind2] << ")\n";
			}
		}
	}

	// Known symbols type for the objects in the initial world
	for (uint32_t s=0; s<symbols.size(); ++s)
	{
		stringStream << "		(IS" << symbols[s]->typeString() << " " << symbols[s]->toString() << ")\n";
	}
	// Introduce edges themselves
	for (uint32_t e=0; e<edges.size(); ++e)
	{
		stringStream << "		(" << edges[e].toString(this) << ")\n";
	}
	stringStream << "	)\n";

	// T A R G E T    W O R L D
	// T A R G E T    W O R L D
	stringStream << "	\n";
	stringStream << "	(:goal\n";
	if (targetObjects.size()>0)
	{
		stringStream << "		(exists (";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			stringStream << " ?" << *it;
		}
		stringStream << " )\n";
	}
	stringStream << "			(and\n";

	// Known symbols type for the objects in the target world
	for (uint32_t s=0; s<target->symbols.size(); ++s)
	{
		std::string kStr = " ";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			if (target->symbols[s]->toString() == *it)
				kStr = " ?";
		}
		stringStream << "				(IS" << target->symbols[s]->typeString() << kStr << target->symbols[s]->toString() << ")\n";
	}
	for (uint32_t e=0; e<target->edges.size(); ++e)
	{
		std::string label, a, b, kStr;
		target->edges[e].getStrings(target, label, a, b);
		stringStream << "				(" << label;

		kStr = " ";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			if (a == *it)
				kStr = " ?";
		}
		stringStream << kStr << a;

		kStr = " ";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			if (b == *it)
				kStr = " ?";
		}
		stringStream << kStr << b;

		stringStream << ")\n";
	}

	stringStream << "			)\n";
	if (targetObjects.size()>0)
		stringStream << "		)\n";
	stringStream << "	)\n";
	stringStream << "\n";

	// M E T R I C   D E F I N I T I O N
	//stringStream << "	(:metric minimize (total-cost))\n";
	stringStream << "\n";
	stringStream << "\n";
	stringStream << ")\n";

//   stringStream << symbolType << "_" << identifier;
  return stringStream.str();
}

/*
std::string AGMModel::generatePDDLProblem(const AGMModel::SPtr &target, int32_t unknowns, std::string domainName, std::string problemName) const
{
	for (uint32_t i=0; i<domainName.size(); ++i)
	{
		if (domainName[i] == '.')
		{
			domainName.resize(i);
			break;
		}
	}

	for (uint32_t i=0; i<problemName.size(); ++i)
	{
		if (problemName[i] == '.')
		{
			problemName.resize(i);
			break;
		}
	}

	std::ostringstream stringStream;
	if (target->symbols.size() == 0) return "";

	// H E A D E R
	stringStream << "(define (problem " << problemName << ")\n";
	stringStream << "\n";

	// D O M A I N
	stringStream << "	(:domain AGGL )\n";

	// D E C L A R E   O B J E C T S
	stringStream << "	(:objects\n";
	std::list <std::string> originalObjects;
	// Symbols that are present in the original model
	for (uint32_t s=0; s<symbols.size(); ++s)
	{
		originalObjects.push_back(symbols[s]->toString());
		stringStream << "		" << symbols[s]->toString() << "\n";
	}

	// Symbols that are only in the target model (not in the original model)
	std::list <std::string> targetObjects;
	for (uint32_t s=0; s<target->symbols.size(); ++s)
	{
		bool found = false;
		for (std::list<std::string>::iterator it=originalObjects.begin(); it!=originalObjects.end(); ++it)
		{
			if (*it == target->symbols[s]->toString())
			{
				found = true;
				break;
			}
		}
		if (not found)
		{
			targetObjects.push_back(target->symbols[s]->toString());
// 			unknowns++;
// 			stringStream << "		" << target->symbols[s]->toString() << "\n";
		}
	}

	// Unknown symbols that allow us to include new stuff in the model
	std::list <std::string> unknownObjectsVec;
	for (int32_t u=0; u<unknowns; ++u)
	{
		stringStream << "		unknown_" << u << "\n";
		std::ostringstream sstr;
		sstr << "unknown_" << u;
		unknownObjectsVec.push_back(sstr.str());
	}
	stringStream << "	)\n";
	stringStream << "\n";

	// I N I T I A L   W O R L D
	stringStream << "	(:init\n";
	// Initial cost
	stringStream << "		(= (total-cost) 0)\n";
	// Unknown temporary objects we are going to artificially inject
	if (unknowns>0)
		stringStream << "		(firstunknown unknown_0)\n";
	for (int32_t u=1; u<unknowns; ++u)
	{
		stringStream << "		(unknownorder unknown_" << u-1 << " unknown_" << u << ")\n";
	}
	// Set not='s
	std::vector<std::string> allObjects;
	allObjects.insert(allObjects.end(),   originalObjects.begin(),   originalObjects.end());
	//allObjects.insert(allObjects.end(),     targetObjects.begin(),     targetObjects.end());
	allObjects.insert(allObjects.end(), unknownObjectsVec.begin(), unknownObjectsVec.end());
	bool useDiff = false;
	for (uint32_t ind1=0; ind1<allObjects.size() and useDiff; ind1++)
	{
		for (uint32_t ind2=0; ind2<allObjects.size(); ind2++)
		{
			if (ind1 != ind2)
			{
				stringStream << "		(diff " << allObjects[ind1] << " " << allObjects[ind2] << ")\n";
			}
		}
	}

	// Known symbols type for the objects in the initial world
	for (uint32_t s=0; s<symbols.size(); ++s)
	{
		stringStream << "		(IS" << symbols[s]->typeString() << " " << symbols[s]->toString() << ")\n";
	}
	// Introduce edges themselves
	for (uint32_t e=0; e<edges.size(); ++e)
	{
		stringStream << "		(" << edges[e].toString(this) << ")\n";
	}
	stringStream << "	)\n";

	// T A R G E T    W O R L D
	// T A R G E T    W O R L D
	stringStream << "	\n";
	stringStream << "	(:goal\n";
	if (targetObjects.size()>0)
	{
		stringStream << "		(exists (";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			stringStream << " ?" << *it;
		}
		stringStream << " )\n";
	}
	stringStream << "			(and\n";

	// Known symbols type for the objects in the target world
	for (uint32_t s=0; s<target->symbols.size(); ++s)
	{
		std::string kStr = " ";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			if (target->symbols[s]->toString() == *it)
				kStr = " ?";
		}
		stringStream << "				(IS" << target->symbols[s]->typeString() << kStr << target->symbols[s]->toString() << ")\n";
	}
	for (uint32_t e=0; e<target->edges.size(); ++e)
	{
		std::string label, a, b, kStr;
		target->edges[e].getStrings(target, label, a, b);
		stringStream << "				(" << label;

		kStr = " ";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			if (a == *it)
				kStr = " ?";
		}
		stringStream << kStr << a;

		kStr = " ";
		for (std::list< std::string>::iterator it=targetObjects.begin(); it!=targetObjects.end(); ++it)
		{
			if (b == *it)
				kStr = " ?";
		}
		stringStream << kStr << b;

		stringStream << ")\n";
	}

	stringStream << "			)\n";
	if (targetObjects.size()>0)
		stringStream << "		)\n";
	stringStream << "	)\n";
	stringStream << "\n";

	// M E T R I C   D E F I N I T I O N
	stringStream << "	(:metric minimize (total-cost))\n";
	stringStream << "\n";
	stringStream << "\n";
	stringStream << ")\n";

//   stringStream << symbolType << "_" << identifier;
  return stringStream.str();
}
*/

std::vector<AGMModelSymbol::SPtr> AGMModel::getSymbols() const
{
	return symbols;
}

// std::vector<AGMModelEdge> AGMModel::getEdges() const
// {
// 	return edges;
// }

AGMModelSymbol::SPtr & AGMModel::symbol(uint32_t i)
{
	return symbols[i];
}


AGMModelEdge & AGMModel::edge(uint32_t i)
{
	return edges[i];
}

int32_t AGMModel::getIdentifierByType(std::string type, int32_t times) const
{
	int32_t ret = -1;
	uint32_t idx = 0;
	for (int32_t time=0; time<=times; ++time)
	{
		for ( ; idx<symbols.size(); ++idx)
		{
			if (symbols[idx]->symbolType == type)
			{
				if (time==times) { /*printf("indice será %d\n", (int32_t)idx);*/ ret = idx;  }
				break;
			}
		}
		if (idx>=symbols.size())
		{
			return -1;
		}
	}
	if (ret != -1)
	{
// 		printf("devolvemos [%d].id ---> _%d_\n", ret, symbols[ret]->identifier);
// 		printf("%s %d\n", __FILE__, __LINE__);
		return symbols[ret]->identifier;
	}
	return -1;
}


int32_t AGMModel::getLinkedID(int32_t id, std::string linkname, int32_t i) const
{
	int64_t ret = -1;
	uint32_t idx = 0;
	for (int32_t time=0; time<=i; ++time)
	{
		for ( ; idx<edges.size(); ++idx)
		{
			if (edges[idx].symbolPair.first == id and linkname == edges[idx].linking)
			{
				if (time==i) ret = idx;
				break;
			}
		}
		if (idx>=edges.size())
		{
			printf("Exception: %d, %s, %d\n", id, linkname.c_str(), i);
			return -1;
// 			AGMMODELEXCEPTION("Trying to get the identifier of a node linked to a given one");
		}
	}
	if (ret != -1)
		return edges[ret].symbolPair.second;
// 	printf("%d\n", __LINE__);
	return -1;
}

AGMModelSymbol::SPtr AGMModel::getParentByLink(int32_t id, std::string linkname, int32_t i) const
{
	uint32_t idx = 0;
	for (int32_t time=0; time<=i; ++time)
	{
		for ( ; idx<edges.size(); ++idx)
		{
			if (edges[idx].symbolPair.second == id and linkname == edges[idx].linking)
			{
				if (time == i)
					return getSymbolByIdentifier(edges[idx].symbolPair.first);
			}
		}
		if (idx>=edges.size())
		{
			AGMMODELEXCEPTION("Trying to get the identifier of a node linked to a given one");
		}
	}
	AGMMODELEXCEPTION("Trying to get the identifier of a node linked to a given one");
}




int32_t AGMModel::getIndexByIdentifier(int32_t targetId) const
{
	for (uint32_t idx=0; idx<symbols.size(); ++idx)
	{
		if (symbols[idx]->identifier == targetId)
		{
			return idx;
		}
	}

	return -1;
}

AGMModelSymbol::SPtr AGMModel::getSymbolByIdentifier(int32_t identif) const
{
	for (uint32_t i=0; i<symbols.size(); ++i)
	{
		if (symbols[i]->identifier == identif)
		{
			return symbols[i];
		}
	}
	std::ostringstream s;
	s << "Exception: " << identif;
	AGMMODELEXCEPTION(std::string("Exception: Trying to get a node with an unexistent index (")+s.str()+std::string(")."));
}

AGMModelSymbol::SPtr AGMModel::getSymbol(int32_t identif) const
{
	return getSymbolByIdentifier(identif);
}

AGMModelSymbol::SPtr AGMModel::getSymbolByName(const std::string &ss) const
{
	for (uint32_t i=0; i<symbols.size(); ++i)
	{
		if (symbols[i]->toString() == name)
		{
			return symbols[i];
		}
	}
	AGMMODELEXCEPTION(std::string("Exception: Trying to get a node with an unexistent name (")+ss+std::string(")."));
}


void AGMModel::removeSymbol(int32_t id)
{
	removeEdgesRelatedToSymbol(id);
	int32_t index = getIndexByIdentifier(id);
	if (index >= 0)
	{
		symbols.erase(symbols.begin() + index);
		removeEdgesRelatedToSymbol(id);
		return;
	}
	AGMMODELEXCEPTION(std::string("Exception: Trying to remove unexisting symbol"));
}

bool AGMModel::removeDanglingEdges()
{
	std::vector<AGMModelEdge>::iterator it = edges.begin();
	bool any = false;
	while (it!=edges.end())
	{
		bool found=false;
		const int32_t symbolsN = numberOfSymbols();
		for (int32_t symbolNumber=0; symbolNumber<symbolsN; symbolNumber++)
		{
			const int32_t id = symbols[symbolNumber]->identifier;
			if (it->symbolPair.first  == id or it->symbolPair.second == id)
			{
				found = true;
				break;
			}
		}
		if (not found)
		{
			it = edges.erase(it);
			any = true;
		}
		else
		{
			it++;
		}
	}
	return any;
}
// <AGMModel>
//
// 	<!-- R O O M -->
// 	<symbol id="2" type="room" />
//
// 	<!-- R O B O T -->
// 	<symbol id="1" type="robot">
// 	</symbol>
//
// 	<link src="1" dst="2" label="in" />
//
//
// 	<!-- non-Explored table -->
// 	<symbol id="5" type="object">
// 		<attribute key="tag" value="0" />
// 		<attribute key="polygon" value="(-842,1854);(775,1854);(775,751);(-842,751)" />
// 	</symbol>
// 	<link src="1" dst="5" label="know" />
// 	<link src="5" dst="2" label="in" />
//
// </AGMModel>


void AGMModel::save(std::string xmlFilePath)
{
	std::ofstream myfile;
	myfile.open (xmlFilePath);
	myfile <<  "<AGMModel>\n\n";

	for (uint32_t i=0; i<symbols.size(); ++i)
	{

		myfile <<"<symbol id=\""<<symbols[i]->identifier<<"\" type=\""<<symbols[i]->symbolType<<"\"";//>\n";
		if (symbols[i]->attributes.size()>0)
		{
			myfile <<">\n";
			std::map<std::string, std::string>::const_iterator itr = symbols[i]->attributes.begin();
			for(; itr!=symbols[i]->attributes.end(); ++itr)
			{
				myfile <<"\t<attribute key=\"" << itr->first <<"\" value=\""<<itr->second<<"\" />\n";
			}

			myfile <<"</symbol>\n";
		}
		else
			myfile <<"/>\n";

	}

	myfile <<"\n\n";
	for (uint32_t i=0; i<edges.size(); ++i)
	{

		myfile <<"<link src=\""<<edges[i].symbolPair.first<<"\" dst=\""<<edges[i].symbolPair.second<<"\" label=\""<<edges[i].linking<<"\"";//> \n";
		if (edges[i]->attributes.size()>0)
		{
			myfile <<">\n";
			std::map<std::string, std::string>::const_iterator itr = edges[i].attributes.begin();
			for(; itr!=edges[i].attributes.end(); ++itr)
			{
				myfile <<"\t<linkAttribute key=\"" << itr->first <<"\" value=\""<<itr->second<<"\" />\n";
			}
			myfile <<"</link>\n";
		}
		else
			myfile <<"/>\n";


	}
	myfile <<  "\n</AGMModel>\n";
	myfile.close();
}


int32_t AGMModel::replaceIdentifierInEdges(int32_t a, int32_t b)
{
	int32_t ret = 0;
	for (uint32_t edge=0; edge < edges.size(); ++edge)
	{
		if (edges[edge].symbolPair.first == a)
		{
			edges[edge].symbolPair.first = b;
			ret++;
		}
		if (edges[edge].symbolPair.second == a)
		{
			edges[edge].symbolPair.second = b;
			ret++;
		}
	}
	return ret;
}

bool AGMModel::removeEdgesRelatedToSymbol(int32_t id)
{
	bool any = false;
	for (int p=0; p<numberOfEdges(); p++)
	{
		if (edges[p].symbolPair.first == id or edges[p].symbolPair.second == id)
		{
			edges.erase(edges.begin() + p);
			any = true;
		}
	}
	return any;
}


void AGMModel::setSymbols(std::vector<AGMModelSymbol::SPtr> s)
{
	symbols = s;
}

void AGMModel::setEdges(std::vector<AGMModelEdge> e)
{
	edges = e;
}


int32_t AGMModel::getNewId()
{
	int32_t ret;
	ret = lastId;
	lastId++;
	return ret;
}


void AGMModel::addEdgeByIdentifiers(int32_t a, int32_t b, const std::string &edgeName, std::map<std::string, std::string> atr)
{
	// Nodes must exist.
	if (a < 0 or b < 0)
	{
		AGMMODELEXCEPTION("Trying to link invalid identifiers");;
	}

	// Check the edge doesn't already exists
	for (uint32_t i=0; i<edges.size(); i++)
	{
		if (edges[i].symbolPair.first == a)
		{
			if (edges[i].symbolPair.second == b)
			{
				if (edges[i].getLabel() == edgeName)
				{
					AGMMODELEXCEPTION(std::string("Exception: addEdgeByIdentifiers"));
				}
			}
		}
	}
	AGMModelEdge edge(a, b, edgeName, atr);
	edges.push_back(edge);
}


void AGMModel::removeEdgeByIdentifiers(int32_t a, int32_t b, const std::string &edgeName)
{
	// Nodes must exist.
	if (a < 0 or b < 0)
	{
		AGMMODELEXCEPTION("Trying to un-link using invalid identifiers");;
	}

	bool removed = false;

	for (uint32_t i=0; i<edges.size(); i++)
	{
		if (edges[i].symbolPair.first == a)
		{
			if (edges[i].symbolPair.second == b)
			{
				if (edges[i].getLabel() == edgeName)
				{
					edges.erase(edges.begin() + i);
					removed = true;
				}
			}
		}
	}

	if (not removed)
		AGMMODELEXCEPTION(std::string("Exception: removeEdgeByIdentifiers"));
}

bool AGMModel::renameEdgeByIdentifiers(int32_t a, int32_t b, const std::string &was, const std::string &will)
{
	// Nodes must exist.
	if (a < 0 or b < 0)
	{
		AGMMODELEXCEPTION("Trying to un-link using invalid identifiers");;
	}

	bool renamed = false;
	for (uint32_t i=0; i<edges.size(); i++)
	{
		if (edges[i].symbolPair.first == a)
		{
			if (edges[i].symbolPair.second == b)
			{
				if (edges[i].getLabel() == was)
				{
					edges[i].setLabel(will);
					renamed = true;
				}
			}
		}
	}
	return renamed;
}


AGMModelEdge & AGMModel::getEdgeByIdentifiers(int32_t a, int32_t b, const std::string &edgeName)
{
	// Nodes must exist.
	if (a < 0 or b < 0)
	{
		AGMMODELEXCEPTION("Trying to get an edge using invalid identifiers");;
	}

	for (uint32_t i=0; i<edges.size(); i++)
	{
		if (edges[i].symbolPair.first == a)
		{
			if (edges[i].symbolPair.second == b)
			{
				if (edges[i].getLabel() == edgeName)
				{
					return edges[i];
				}
			}
		}
	}
	std::ostringstream s;
	s << "Exception: " <<  a << " "<< b <<" "<<edgeName;
	AGMMODELEXCEPTION(std::string("Exception: AGMModel::getEdgeByIdentifiers EDGE  (")+s.str()+std::string(")."));

}


AGMModelEdge & AGMModel::getEdge(AGMModelSymbol::SPtr a, AGMModelSymbol::SPtr b, const std::string &edgeName)
{
	return getEdgeByIdentifiers(a->identifier, b->identifier, edgeName);
}



AGMModelSymbol::SPtr AGMModel::newSymbol(std::string typ, int32_t id)
{
	return newSymbol(id, typ);
}

AGMModelSymbol::SPtr AGMModel::newSymbol(int32_t identifier, std::string typ)
{
	AGMModelSymbol *s = new AGMModelSymbol(this, identifier, typ);
	return symbols[getIndexByIdentifier(s->identifier)];
}

AGMModelSymbol::SPtr AGMModel::newSymbol(int32_t identifier, std::string typ, std::map<std::string, std::string> atr)
{
	AGMModelSymbol *s = new AGMModelSymbol(this, identifier, typ, atr);
	return symbols[getIndexByIdentifier(s->identifier)];
}

AGMModelSymbol::SPtr AGMModel::newSymbol(std::string typ, std::map<std::string, std::string> atr)
{
	AGMModelSymbol *s = new AGMModelSymbol(this, -1, typ, atr);
	return symbols[getIndexByIdentifier(s->identifier)];
}



AGMModel::iterator::iterator()
{
	index = -1;
	modelRef = NULL;
}

AGMModel::iterator::iterator(iterator &iter)
{
	index = iter.index;
	modelRef = iter.modelRef;
}

AGMModel::iterator::iterator(const iterator &iter)
{
	index = iter.index;
	modelRef = iter.modelRef;
}

AGMModel::iterator AGMModel::iterator::begin(AGMModel *m)
{
	iterator iter;
	iter.modelRef = m;
	iter.index = -1;
	iter++;
	return iter;
}

AGMModel::iterator AGMModel::iterator::begin(boost::shared_ptr<AGMModel> m)
{
	return AGMModel::iterator::begin(m.get());
}

AGMModel::iterator AGMModel::iterator::end(AGMModel *m)
{
	iterator iter;
	iter.modelRef = m;
	iter.index = -10;
	return iter;
}

AGMModel::iterator AGMModel::iterator::end(boost::shared_ptr<AGMModel> m)
{
	return AGMModel::iterator::end(m.get());
}

bool AGMModel::iterator::operator==(const iterator &rhs)
{
	return index == rhs.index;
}

bool AGMModel::iterator::operator!=(const iterator &rhs)
{
	return index != rhs.index;
}

AGMModel::iterator AGMModel::iterator::operator++()
{
	if (modelRef == NULL) AGMMODELEXCEPTION(std::string("Attempting to use uninitialized iterator!"));

	// The end can't be incremented
	if (index == -10)
	{
		return *this;
	}

// 	printf("++\n");
	if (index+1 < (int32_t)modelRef->symbols.size())
	{
// 		printf("++ normal\n");
		index++;
	}
	else
	{
// 		printf("++ ends!!\n");
		index = -10;
	}
	return *this;
}

AGMModel::iterator AGMModel::iterator::operator++(int32_t times)
{
	AGMModel::iterator it = *this;
	operator++();
	return it;
}

AGMModelSymbol::SPtr AGMModel::iterator::operator*()
{
	if (index<0)
	{
		throw "invalid iterator";
	}

	if (modelRef == NULL) AGMMODELEXCEPTION(std::string("Attempting to use uninitialized iterator!"));
	return modelRef->symbols[index];
}

AGMModelSymbol::SPtr AGMModel::iterator::operator->()
{
	if (index<0)
	{
		throw "invalid iterator";
	}

	if (modelRef == NULL) AGMMODELEXCEPTION(std::string("Attempting to use uninitialized iterator!"));
	return modelRef->symbols[index];
}


#if ROBOCOMP_SUPPORT == 1

std::map<std::string, AGMModelSymbol::SPtr> AGMModel::getSymbolsMap(::RoboCompAGMCommonBehavior::ParameterMap params)
{
	std::map<std::string, AGMModelSymbol::SPtr> ret;
	for (auto v : params)
	{
		int r = str2int(params[v.first].value);
		if (r>0)
		{
			ret[v.first] = getSymbolByIdentifier(r);
		}
	}
	return ret;
}

#endif



