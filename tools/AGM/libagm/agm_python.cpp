// http://www.boost.org/doc/libs/1_53_0/libs/python/doc/tutorial/doc/html/python/exposing.html

#ifndef Q_MOC_RUN
	#include <boost/python.hpp>
	#include "agm_modelEdge.h"
	#include "agm_modelSymbols.h"
	#include "agm.h"


using namespace boost::python;

// 	 .def("", &::)

BOOST_PYTHON_MODULE(libagm)
{
	class_<AGM>("AGM", init<std::string, std::string>())
	 .def("print", &AGM::print)
	;


	class_<AGMModelEdge>("AGMModelEdge", init<>()) 
	 .def(init<int, int, std::string>())
	 .def(init<const AGMModelEdge&>())
	 .def("getLabel", &AGMModelEdge::getLabel)
	 .def("getSymbolPair", &AGMModelEdge::getSymbolPair)
	 .def("setLabel", &AGMModelEdge::setLabel)
	 .def("setSymbolPair", &AGMModelEdge::setSymbolPair)
	 .def("toString", &AGMModelEdge::toString)
	;
	
	
	
	
	
	class_<AGMModelSymbol>("AGMModelSymbol", init<>()) 
	 .def(init<std::string>())
	 .def(init<int32_t, std::string>())
	 .def(init<int32_t, std::string, std::map<std::string, std::string> >())
	 .def(init<const AGMModelSymbol&>())
	;

		
	class_<AGMModel>("AGMModel", init<>()) 
	 // Constructors
	 .def(init<boost::shared_ptr<AGMModel> >())
	 .def(init<AGMModel &>())
	 .def("string2float", &AGMModel::string2float)
	 // (S/G)ets
	 .def("clear", &AGMModel::clear)
	 .def("insertSymbol", &AGMModel::insertSymbol)
	 .def("resetLastId", &AGMModel::resetLastId)
	 .def("numberOfEdges", &AGMModel::numberOfEdges)
	 .def("numberOfSymbols", &AGMModel::numberOfSymbols)
	 .def("indexOfSymbol", &AGMModel::indexOfSymbol)
	 .def("indexOfFirstSymbolByType", &AGMModel::indexOfFirstSymbolByType)
	 .def("indexOfFirstSymbolByValues", &AGMModel::indexOfFirstSymbolByValues)
	 .def("getSymbols", &AGMModel::getSymbols)
	 .def("getEdges", &AGMModel::getEdges)
	 .def("setSymbols", &AGMModel::setSymbols)
	 .def("setEdges", &AGMModel::setEdges)
	 .def("getIdentifierByName", &AGMModel::getIdentifierByName)
	 .def("getIdentifierByType", &AGMModel::getIdentifierByType)
	 .def("getLinkedID", &AGMModel::getLinkedID)
	 .def("getIndexByIdentifier", &AGMModel::getIndexByIdentifier)
	 .def("generatePDDLProblem", &AGMModel::generatePDDLProblem)
	;

	
}


#endif

