#include "agm.h"

int main(int argc, char **argv)
{
	AGMModel::SPtr xml(new AGMModel());

	AGMModelConverter::fromXMLToInternal("/home/robocomp/AGM/examples/basic/initialModel.xml", xml);
	AGMModelPrinter::printWorld(xml);
}

