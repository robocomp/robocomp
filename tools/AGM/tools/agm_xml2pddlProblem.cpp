#include "agm.h"
#include <fstream>
#include <ostream>
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
	if (argc < 4)
	{
		std::cout << "Usage:\n\t" << argv[0] << " initialModel.xml targetPattern.xml outputPDDLProblem.pddl [newMemorySize]" << std::endl;
		return -1;
	}

	int unknownSymbols = 5;
	if (argc>4)
	{
		unknownSymbols = atoi(argv[4]);
	}

	AGMModel::SPtr xml1(new AGMModel());
	AGMModelConverter::fromXMLToInternal(argv[1], xml1);
	AGMModel::SPtr xml2(new AGMModel());
	AGMModelConverter::fromXMLToInternal(argv[2], xml2);

	printf("Creating problem with %d potentially new symbols\n", unknownSymbols);
	std::string ret = xml1->generatePDDLProblem(xml2, unknownSymbols, "active.pddl", argv[3]);

	std::ofstream out(argv[3]);
	out << ret;
	out.close();

	return 0;
}

