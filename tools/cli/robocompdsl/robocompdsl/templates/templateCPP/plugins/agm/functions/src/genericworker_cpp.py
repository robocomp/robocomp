import datetime
from string import Template
from robocompdsl.templates.common.templatedict import TemplateDict



AGM_CREATEACTION_STR = """
RoboCompPlanning::Action GenericWorker::createAction(std::string s)
{
	// Remove useless characters
	char chars[]="()";
		for (unsigned int i=0; i<strlen(chars); ++i)
	{
		s.erase(std::remove(s.begin(), s.end(), chars[i]), s.end());
	}

		// Initialize string parsing
	RoboCompPlanning::Action ret;
	istringstream iss(s);

	// Get action (first segment)
	if (not iss)
	{
		printf("agent %s: received invalid action (%s) -> (%d)\\n", PROGRAM_NAME, __FILE__, __LINE__);
		exit(-1);
	}
	else
	{
		iss >> ret.name;
	}

	do
	{
		std::string ss;
		iss >> ss;
		ret.symbols.push_back(ss);
	} while (iss);

	return ret;
}
"""

AGM_ACTIVATE = """
bool GenericWorker::activate(const BehaviorParameters &prs)
{
	printf("Worker::activate\\n");
	mutex->lock();
	p = prs;
	active = true;
	iter = 0;
	mutex->unlock();
	return active;
}
"""

AGM_DEACTIVATE = """
bool GenericWorker::deactivate()
{
	printf("Worker::deactivate\\n");
	mutex->lock();
	active = false;
	iter = 0;
	mutex->unlock();
	return active;
}
"""

AGM_SETPARAMETERSANDPOSSIBLEACTIVATION_STR = """
bool GenericWorker::setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated)
{
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	for (RoboCompAGMCommonBehavior::ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		// Action
		p.action = createAction(params["action"].value);

		// Fill received plan
		p.plan.clear();
		QStringList actionList = QString::fromStdString(params["plan"].value).split(QRegExp("[()]+"), QString::SkipEmptyParts);
		for (int32_t actionString=0; actionString<actionList.size(); actionString++)
		{
			std::vector<string> elementsVec;
			QStringList elements = actionList[actionString].remove(QChar('\\n')).split(QRegExp("\\\\s+"), QString::SkipEmptyParts);
			for (int32_t elem=0; elem<elements.size(); elem++)
			{
				elementsVec.push_back(elements[elem].toStdString());
			}
			p.plan.push_back(elementsVec);
		}
	}
	catch (...)
	{
		return false;
	}

	// Check if we should reactivate the component
	if (isActive())
	{
		activate(p);
		reactivated = true;
	}

	return true;
}
"""

CPP_TYPES = ['int', 'float', 'bool', 'void']

class genericworker_cpp(TemplateDict):
    def __init__(self, component):
        super(genericworker_cpp, self).__init__()
        self.component = component
        self['agm_methods'] = self.agm_methods()



    def gui_setup(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_SETUP_STR
        return result

    def agm_methods(self):
        result = ""
        # TODO: move to component method (self.component.is_agm())
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += AGM_CREATEACTION_STR
            result += AGM_ACTIVATE
            result += AGM_DEACTIVATE
            result += AGM_SETPARAMETERSANDPOSSIBLEACTIVATION_STR
        return result

