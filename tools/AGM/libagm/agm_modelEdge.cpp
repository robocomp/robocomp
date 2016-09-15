#include "agm_modelEdge.h"
#include "agm_modelPrinter.h"


AGMModelEdge::AGMModelEdge()
{
	symbolPair = std::pair<int32_t, int32_t>(0, 0);
	linking = "-";
	attributes.clear();
}

AGMModelEdge::~AGMModelEdge()
{
}

AGMModelEdge::AGMModelEdge(const AGMModelEdge &src)
{
	setFrom(src);
}


AGMModelEdge::AGMModelEdge(int32_t a, int32_t b, std::string linking_, std::map<std::string, std::string> atr)
{
	symbolPair = std::pair<int32_t, int32_t>(a, b);
	linking = linking_;
	attributes = atr;
}

AGMModelEdge& AGMModelEdge::operator=(const AGMModelEdge &src)
{
	this->setFrom(src);
	return *this;
}



void AGMModelEdge::setFrom(const AGMModelEdge &src)
{
	linking = src.linking;
	symbolPair = src.symbolPair;
	attributes = src.attributes;
}



std::string AGMModelEdge::toString(const AGMModel::SPtr &world, bool verbose) const
{
	return toString(world.get(),verbose);
}



std::string AGMModelEdge::toString(const AGMModel *world, bool verbose) const
{
	std::ostringstream stringStream;
	std::string stringA, stringB;

	try
	{
		stringA = world->getSymbol(symbolPair.first)->toString();
	}
	catch (...)
	{
		printf("AGMModel error: it probably lacks of a node with %d as identifier!!!\n", symbolPair.first);
		AGMModelPrinter::printWorld(world);
		exit(-1);
	}
	try
	{
		stringB = world->getSymbol(symbolPair.second)->toString();
	}
	catch (...)
	{
		printf("AGMModel error: it probably lacks of a node with %d as identifier!!!\n", symbolPair.second);
		AGMModelPrinter::printWorld(world);
		exit(-1);
	}
	
	stringStream << linking << " " << stringA << " " << stringB;
	
	if (verbose)
	{		
		std::map<std::string, std::string>::const_iterator itr = this->attributes.begin();
		for(; itr!=this->attributes.end(); ++itr)
		{
			//printf("\t<%s> --> <%s>\n", itr->first.c_str(), itr->second.c_str());
			stringStream <<"\n\t"<<itr->first<<" "<< itr->second;
		}
	}

	
	return stringStream.str();
}

void AGMModelEdge::setAttribute(std::string a, std::string v)
{
	attributes[a] = v;
}

std::string AGMModelEdge::getAttribute(std::string a)
{
	//return attributes[a];
	std::string s="";
	try
	{
		s= attributes.at(a);
	}
	catch (const std::out_of_range& oor)
	{
		std::cerr << "Out of Range error: " << oor.what() << '\n';
	}
	return s; 
}


void AGMModelEdge::setLabel(std::string l)
{
	linking = l;
}


void AGMModelEdge::setSymbolPair(std::pair<int32_t, int32_t> p)
{
	symbolPair = p;
}

void AGMModelEdge::getStrings(const AGMModel::SPtr &world, std::string &label, std::string &a, std::string &b) const
{
	getStrings(world.get(), label, a, b);
}

void AGMModelEdge::getStrings(const AGMModel *world, std::string &label, std::string &a, std::string &b) const
{
	try
	{
		a = world->getSymbol(symbolPair.first)->toString();
	}
	catch (...)
	{
		printf("AGMModel error: it probably lacks of a node with %d as identifier!!!\n", symbolPair.first);
		AGMModelPrinter::printWorld(world);
		exit(-1);
	}
	try
	{
		b = world->getSymbol(symbolPair.second)->toString();
	}
	catch (...)
	{
		printf("AGMModel error: it probably lacks of a node with %d as identifier!!!\n", symbolPair.second);
		AGMModelPrinter::printWorld(world);
		exit(-1);
	}

	label = linking;
}
