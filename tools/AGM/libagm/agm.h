#pragma once

#ifndef Q_MOC_RUN
	#include <agm_misc_functions.h>
	#include <agm_model.h>
	#include <agm_modelSymbols.h>
	#include <agm_modelPrinter.h>
	#include <agm_modelEdge.h>
	#include <agm_modelConverter.h>
	#include <agm_plan.h>
	#include <agm_inner.h>
#endif

/** @defgroup CPPAPI AGM's C++ API

\section agmmodelell Working with AGM models

\subsection creating Creating a model

We can create empty models:
\code{.cpp}
	AGMModel::SPtr newModel(new AGMModel());
\endcode
Create copies of existing models:
\code{.cpp}
	AGMModel::SPtr newModel(new AGMModel(worldModel));
\endcode

Or load models from an XML file (see \ref xmlmodels):
\code{.cpp}
	AGMModel::SPtr newModel;
	AGMModelConverter::fromXMLToInternal("/path/to the XML file", newModel);
\endcode


\subsection addSymbol Adding a symbol to a model
The following code snippet creates two symbols of types "object" and "objectSt" (the latter with a particular ID).
\code{.cpp}
	AGMModelSymbol::SPtr newMug = newModel->newSymbol("object");
	AGMModelSymbol::SPtr newMugStatus = newModel->newSymbol("objectSt", 999);
\endcode


\subsection removeSymbol Remove a symbol from a model
The following code snippet removes two symbols in two different ways: a) using its pointer b) using its identifier:
\code{.cpp}
	newModel->removeSymbolnewSymbol(newMug->identifier);
	newModel->removeSymbolnewSymbol(newMugStatus);
\endcode


\subsection accessFromParameters Accessing the symbols of a rule execution given by the AGM executive
The following greates a map providing pointers to the symbols in the rule (a <em>std::map&lt;string, AGMModelSymbol::Ptr&gt;</em> map).
We can ask the library to generate pointers for some of the parameters:
\code{.cpp}
			auto symbols = newModel->getSymbolsMap(params, "robot", "container");
\endcode
or all of them (if no particular parameter is specified):
\code{.cpp}
			auto symbols = newModel->getSymbolsMap(params);
\endcode



\subsection addLink Adding an edge to a model
Links can be added using pointers to the symbols involved or their identifiers
\code{.cpp}
	newModel->addEdge(newMilk, newMilkStatus, "reachable");
	newModel->addEdge(newMilk->identifier, newMilkStatus->identifier, "see");
\endcode


\subsection update Updating the attributes of a symbol
Updating the attributes of a symbol is straightforward:
\code{.cpp}
// Modify the attributes in the current model
worldModel->symbols[ballIndex]->attributes["x"] = xPos;
worldModel->symbols[ballIndex]->attributes["y"] = yPos;
worldModel->symbols[ballIndex]->attributes["z"] = zPos;
\endcode


\subsection print Printing a model
Regardless of whether we have a AGMModel::SPtr or its RoboComp/Ice version we will use the same function:
\code{.cpp}
AGMModelPrinter::printWorld(model);
\endcode



\subsection agmexec Sending model modifications and symbol updates to the AGM executive

\subsubsection agmexec1 Model modifications
When an AGM agent detects a new object or relationship it is necessary to propose a full-model modification proposal. This publication will trigger model proposal verification in the AGM executive, so this kind of proposal should only be used when necessary (when making structural changes in the model).

As a result of the following code a modification proposal would be published:
\code{.cpp}
// Create a modification event with the previous and proposed model
RoboCompAGMWorldModel::Event e;
e.sender = "balltracker";
e.why = RoboCompAGMWorldModel::BehaviorBasedModification;
AGMModelConverter::fromInternalToIce(worldModel, e.backModel);
AGMModelConverter::fromInternalToIce(newModel, e.newModel);
try { agmagenttopic->modificationProposal(e);}
catch(const Ice::Exception &e) { std::cout << e << std::endl;};
\endcode

\subsubsection agmexec2 Symbol updates
When an AGM agent needs to update the attributes of a symbol and no structural modifications are made, the agent can choose to publish only a symbol update. It is faster than a full-model modification proposal, since no model-verification is involved.

As a result of the following code a symbol update would be published:
\code{.cpp}
// Fill a RoboCompAGMWorldModel::Node structure with the new data.
RoboCompAGMWorldModel::Node node;
AGMModelConverter::fromInternalToIce(worldModel->symbols[ballIndex], node);
// Publish the proposal
try { agmagenttopic->update(node); }
catch(const Ice::Exception &e) { cout << e << endl; }
\endcode




**/



/**
\page xmlmodels AGM XML files
The following is an example of an AGM model XML file and its visual representation:
<table border="0">
<tr>
<td>
\code{.xml}
<AGMModel>
	<symbol id="1" type="OBJ" x="0" y="-150">
		<attribute key="alias" value="package1" />
	</symbol>

	<symbol id="2" type="TRCK" x="200" y="-150">
		<attribute key="alias" value="PGH-TRUCK" />
	</symbol>

	<symbol id="6" type="LCTN" x="200" y="0">
		<attribute key="alias" value="PGH-PO" />
	</symbol>

	<symbol id="8" type="LCTN" x="80" y="0">
		<attribute key="alias" value="PGH-AIRPORT" />
	</symbol>

	
	<symbol id="10" type="CITY" x="170" y="150">
		<attribute key="alias" value="PGH" />
	</symbol>

	<link src="1" dst="8" label="at" />
	<link src="6" dst="10" label="inCity" />
	<link src="8" dst="10" label="inCity" />
	<link src="2" dst="6" label="at" />

</AGMModel>
\endcode
</td>
<td>
<img src="init0.png" width="600px">
</td>
</tr>
</table>


\section tags AGMModel Tags

- &lt;%AGMModel&gt;: The root tag containing the whole model.
- &lt;symbol&gt;: Represents a symbol.
- &lt;link&gt;: Represents a link.
- &lt;attribute&gt;: Used to add attributes to symbols and edges.

\subsection symbol symbol
The &lt;symbol&gt; tag is used to include symbols in a model. They have the following attributes:
- id: the number identifying the symbol
- type: the type of the symbol
- x (optional): x coordinate, used for rendering
- y (optional): y coordinate, used for rendering

\subsection link link
The &lt;link&gt; tag is used to include links between already-existing symbols in a model. They have the following attributes:
- src: the number identifying the source symbol
- dst: the number identifying the destination symbol
- label: the label of the link

\subsection attribute attribute
The &lt;attribute&gt; tag is used to include attributes in symbols and links. The attribute tag has the following attributes:
- key: the name of the attribute
- value: the content of the attribute


**/



