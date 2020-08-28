import datetime
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number
from templates.templateCPP.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict

INCLUDE_STR = '#include <${iface_name}${suffix}.h>\n'


PROXY_PTR_STR = """${prx_type}Prx${ptr} ${lower}${num}_${prefix}proxy;\n"""


TOPIC_MANAGER_STR = """
IceStorm::TopicManagerPrx${ptr} topicManager;
try
{
	topicManager = ${type}(communicator()->propertyToProxy("TopicManager.Proxy"));
}
catch (const Ice::Exception &ex)
{
	cout << "[" << PROGRAM_NAME << "]: Exception: \'rcnode\' not running: " << ex << endl;
	return EXIT_FAILURE;
}
"""


PUBLISHES_STR = """
while (!<LOWER>_topic)
{
	try
	{
		<LOWER>_topic = topicManager->retrieve("<NORMAL>");
	}
	catch (const IceStorm::NoSuchTopic&)
	{
		cout << "[" << PROGRAM_NAME << "]: ERROR retrieving <NORMAL> topic. \\n";
		try
		{
			<LOWER>_topic = topicManager->create("<NORMAL>");
		}
		catch (const IceStorm::TopicExists&){
			// Another client created the topic.
			cout << "[" << PROGRAM_NAME << "]: ERROR publishing the <NORMAL> topic. It's possible that other component have created\\n";
		}
	}
	catch(const IceUtil::NullHandleException&)
	{
		cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\\n"<<
		"\\t\\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\\n";
		return EXIT_FAILURE;
	}
}

"""

SUBSCRIBESTO_STR = """
// Server adapter creation and publication
<CHANGE1> <LOWER>_topic;
<CHANGE2> <PROXYNAME>;
try
{
	if (not GenericMonitor::configGetString(communicator(), prefix, "<NORMAL>Topic.Endpoints", tmp, ""))
	{
		cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy <NORMAL>Proxy";
	}
	Ice::ObjectAdapterPtr <NORMAL>_adapter = communicator()->createObjectAdapterWithEndpoints("<LOWER>", tmp);
	<PTR_TYPE>Ptr <LOWER>I_ = <CHANGE3>(worker);
	<CHANGE4> <PROXYNAME> = <NORMAL>_adapter->addWithUUID(<LOWER>I_)->ice_oneway();
	if(!<LOWER>_topic)
	{
		try {
			<LOWER>_topic = topicManager->create("<NORMAL>");
		}
		catch (const IceStorm::TopicExists&) {
			//Another client created the topic
			try{
				cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\\n";
				<LOWER>_topic = topicManager->retrieve("<NORMAL>");
			}
			catch(const IceStorm::NoSuchTopic&)
			{
				cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\\n";
				//Error. Topic does not exist
			}
		}
		catch(const IceUtil::NullHandleException&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\\n"<<
			"\\t\\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\\n";
			return EXIT_FAILURE;
		}
		IceStorm::QoS qos;
		<LOWER>_topic->subscribeAndGetPublisher(qos, <PROXYNAME>);
	}
	<NORMAL>_adapter->activate();
}
catch(const IceStorm::NoSuchTopic&)
{
	cout << "[" << PROGRAM_NAME << "]: Error creating <NORMAL> topic.\\n";
	//Error. Topic does not exist
}

"""

IMPLEMENTS_STR = """
try
{
	// Server adapter creation and publication
	if (not GenericMonitor::configGetString(communicator(), prefix, "<NORMAL>.Endpoints", tmp, ""))
	{
		cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy <NORMAL>";
	}
	Ice::ObjectAdapterPtr adapter<NORMAL> = communicator()->createObjectAdapterWithEndpoints("<NORMAL>", tmp);
	<C++_VERSION>
	adapter<NORMAL>->add(<LOWER>, Ice::stringToIdentity("<LOWER>"));
	adapter<NORMAL>->activate();
	cout << "[" << PROGRAM_NAME << "]: <NORMAL> adapter created in port " << tmp << endl;
}
catch (const IceStorm::TopicExists&){
	cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for <NORMAL>\\n";
}

"""


REQUIRE_STR = """
try
{
	if (not GenericMonitor::configGetString(communicator(), prefix, "<NORMAL><PROXYNUMBER>Proxy", proxy, ""))
	{
		cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy <NORMAL>Proxy\\n";
	}
	<C++_VERSION>
}
catch(const Ice::Exception& ex)
{
	cout << "[" << PROGRAM_NAME << "]: Exception creating proxy <NORMAL><PROXYNUMBER>: " << ex;
	return EXIT_FAILURE;
}
rInfo("<NORMAL>Proxy<PROXYNUMBER> initialized Ok!");

"""

UNSUBSCRIBE_STR = """
try
{
	std::cout << \"Unsubscribing topic: ${name} \" <<std::endl;
	${name}_topic->unsubscribe( ${name} );
}
catch(const Ice::Exception& ex)
{
	std::cout << \"ERROR Unsubscribing topic: $name \" << ex.what()<<std::endl;
}
"""


class src_main_cpp(TemplateDict):
    def __init__(self, component):
        super(src_main_cpp, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['component_name'] = component.name
        self['implements_interface_includes'] = self.interface_includes(self.component.implements, 'I', True)
        self['subscribes_interface_includes'] = self.interface_includes(self.component.subscribesTo, 'I', True)
        self['imports_interface_includes'] = self.interface_includes(self.component.recursiveImports)
        self['interface_includes'] = self.interface_includes(self.component.recursiveImports)
        self['proxies_map_creation'] = self.proxies_map_creation()
        self['publishes_proxy_ptr'] = self.proxy_ptr(self.component.publishes, 'pub')
        self['requires'] = self.requires()
        self['requires_proxy_ptr'] = self.proxy_ptr(self.component.requires)
        self['topic_manager_creation'] = self.topic_manager_creation()
        self['publish'] = self.publish()
        self['specificworker_creation'] = self.specificworker_creation()
        self['commonbehaviorI_creation'] = self.commonbehaviorI_creation()
        self['implements'] = self.implements()
        self['subscribes_to'] = self.subscribes_to()
        self['unsubscribe_code'] = self.unsubscribe_code()

    @staticmethod
    def interface_includes(interfaces, suffix='', lower=False):
        result = ""
        for interface in sorted(interfaces):
            if communication_is_ice(interface):
                name = interface if isinstance(interface, str) else interface.name
                name = name.split('/')[-1].split('.')[0]
                if lower:
                    name = name.lower()
                result += Template(INCLUDE_STR).substitute(iface_name=name, suffix=suffix)
        return result

    def proxy_ptr(self, interfaces, prefix=''):
        result = ""
        for interface, num in get_name_number(interfaces):
            if communication_is_ice(interface):
                ptr = ""
                if self.component.language.lower() != "cpp":
                    ptr = "Ptr"
                name = interface.name
                module = self.component.idsl_pool.module_providing_interface(name)
                proxy_type = utils.get_type_string(name, module['name'])
                result += Template(PROXY_PTR_STR).substitute(prx_type=proxy_type, ptr=ptr, lower=name.lower(), num=num,
                                                             prefix=prefix)
        return result

    def topic_manager_creation(self):
        result = ""
        need_topic = False
        for pub in self.component.publishes:
            if communication_is_ice(pub):
                need_topic = True
        for pub in self.component.subscribesTo:
            if communication_is_ice(pub):
                need_topic = True
        if need_topic:
            if self.component.language.lower() == "cpp":
                ptr = ""
                manager_type = "IceStorm::TopicManagerPrx::checkedCast"
            else:
                ptr = "Ptr"
                manager_type = "topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>"
            result += Template(TOPIC_MANAGER_STR).substitute(ptr=ptr, type=manager_type)
        return result

    def publish(self):
        result = ""
        for pba in self.component.publishes:
            if type(pba) == str:
                pb = pba
            else:
                pb = pba[0]
            if communication_is_ice(pba):
                if self.component.language.lower() == "cpp":
                    result += "IceStorm::TopicPrx " + pb.lower() + "_topic;\n"
                else:
                    result += "std::shared_ptr<IceStorm::TopicPrx> " + pb.lower() + "_topic;\n"
                result += PUBLISHES_STR.replace("<NORMAL>", pb).replace("<LOWER>", pb.lower())
                module = self.component.idsl_pool.module_providing_interface(pb)
                proxy_type = utils.get_type_string(pb, module['name'])
                if self.component.language.lower() == "cpp":
                    result += "Ice::ObjectPrx " + pb.lower() + "_pub = " + pb.lower() + "_topic->getPublisher()->ice_oneway();\n"
                    result += "" + pb.lower() + "_pubproxy = " + proxy_type + "Prx::uncheckedCast(" + pb.lower() + "_pub);\n"
                    result += "mprx[\"" + pb + "Pub\"] = (::IceProxy::Ice::Object*)(&" + pb.lower() + "_pubproxy);\n"
                else:
                    result += "auto " + pb.lower() + "_pub = " + pb.lower() + "_topic->getPublisher()->ice_oneway();\n"
                    result += "" + pb.lower() + "_pubproxy = Ice::uncheckedCast<" + pb + "Prx>(" + pb.lower() + "_pub);\n"
        return result

    def subscribes_to(self):
        result = ""
        for interface, num in get_name_number(self.component.subscribesTo):
            name = interface.name
            if communication_is_ice(interface):
                if self.component.language.lower() == "cpp":
                    change1 = "IceStorm::TopicPrx"
                    change2 = "Ice::ObjectPrx"
                    change3 = " new <NORMAL>I"
                    change4 = "Ice::ObjectPrx"
                else:
                    change1 = "std::shared_ptr<IceStorm::TopicPrx>"
                    change2 = "Ice::ObjectPrxPtr"
                    change3 = " std::make_shared <<NORMAL>I>"
                    change4 = "auto"

                module = self.component.idsl_pool.module_providing_interface(name)
                proxy_type = utils.get_type_string(name, module['name'])
                result += SUBSCRIBESTO_STR.replace("<CHANGE1>", change1).replace("<CHANGE2>", change2).replace(
                    "<CHANGE3>",
                    change3).replace(
                    "<CHANGE4>", change4).replace("<NORMAL>", name).replace("<LOWER>", name.lower()).replace(
                    "<PROXYNAME>", name.lower() + num).replace("<PROXYNUMBER>", num).replace("<PTR_TYPE>", proxy_type)
        return result

    def implements(self):
        result = ""
        for ima in self.component.implements:
            if type(ima) == str:
                im = ima
            else:
                im = ima[0]
            if communication_is_ice(ima):
                if self.component.language.lower() == "cpp":
                    cpp = "<NORMAL>I *<LOWER> = new <NORMAL>I(worker);"
                else:
                    cpp = "auto <LOWER> = std::make_shared<<NORMAL>I>(worker);"
                result += IMPLEMENTS_STR.replace("<C++_VERSION>", cpp).replace("<NORMAL>", im).replace("<LOWER>",
                                                                                                       im.lower())

        return result

    def requires(self):
        result = ""
        for interface, num in get_name_number(self.component.requires):
            name = interface.name
            if communication_is_ice(interface):
                module = self.component.idsl_pool.module_providing_interface(name)
                proxy_type = utils.get_type_string(name, module['name'])
                if self.component.language.lower() == "cpp":
                    cpp = "<PROXYNAME>_proxy = <PROXY_TYPE>Prx::uncheckedCast( communicator()->stringToProxy( proxy ) );"
                else:
                    cpp = "<PROXYNAME>_proxy = Ice::uncheckedCast<<PROXY_TYPE>Prx>( communicator()->stringToProxy( proxy ) );"
                result += REQUIRE_STR.replace("<C++_VERSION>", cpp).replace("<NORMAL>", name).replace("<LOWER>",
                                                                                                      name.lower()).replace(
                    "<PROXYNAME>", name.lower() + num).replace("<PROXYNUMBER>", num).replace('<PROXY_TYPE>', proxy_type)
            if self.component.language.lower() == "cpp":
                result += "mprx[\"" + name + "Proxy" + num + "\"] = (::IceProxy::Ice::Object*)(&" + name.lower() + num + "_proxy);//Remote server proxy creation example\n"
        return result

    def specificworker_creation(self):
        result = ""
        if self.component.language.lower() == "cpp":
            var_name = 'm'
        else:
            var_name = 't'
            proxy_list = [interface.name.lower() + num + "_proxy" for interface, num in get_name_number(self.component.requires)]
            proxy_list += [interface.name.lower() + "_pubproxy" for interface in self.component.publishes]
            if proxy_list:
                result += "tprx = std::make_tuple(" + ",".join(proxy_list) + ");\n"
            else:
                result += "tprx = std::tuple<>();\n"
        result += "SpecificWorker *worker = new SpecificWorker({}prx, startup_check_flag);\n".format(var_name)
        return result

    def unsubscribe_code(self):
        result = "\n"
        for interface in self.component.subscribesTo:
            if communication_is_ice(interface):
                result = Template(UNSUBSCRIBE_STR).substitute(name=interface.name.lower())
        return result

    def proxies_map_creation(self):
        result = ""
        if self.component.language.lower() == 'cpp':
            result += "MapPrx mprx;\n"
        else:
            result += "TuplePrx tprx;\n"
        return result

    def commonbehaviorI_creation(self):
        result = ""
        if self.component.language.lower() == "cpp":
            result += "CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor);\n"
        else:
            result += "auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);\n"
        return result
