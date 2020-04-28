from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number

INCLUDE_STR = '#include <${iface_name}${suffix}.h>\n'

def interface_includes(interfaces, suffix='', lower=False):
    result = ""
    for iface in sorted(interfaces):
        if communication_is_ice(iface):
            iface_name = iface
            while not isinstance(iface_name, str):
                iface_name = iface_name[0]
            iface_name = iface_name.split('/')[-1].split('.')[0]
            if lower:
                iface_name = iface_name.lower()
            result += Template(INCLUDE_STR).substitute(iface_name=iface_name, suffix=suffix)
    return result

PROXY_PTR_STR = """${name}Prx${ptr} ${lower}${num}_${prefix}proxy;\n"""

def proxy_ptr(interfaces,language, prefix=''):
    result = ""
    for iface, num in get_name_number(interfaces):
        if communication_is_ice(iface):
            ptr = ""
            if language.lower() != "cpp":
                ptr = "Ptr"
            name = iface[0]
            result += Template(PROXY_PTR_STR).substitute(name=name, ptr=ptr, lower=name.lower(), num=num, prefix=prefix)
    return result

TOPIC_MANAGER_STR="""
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

def topic_manager_creation(component):
    result = ""
    need_topic = False
    for pub in component.publishes:
        if communication_is_ice(pub):
            need_topic = True
    for pub in component.subscribesTo:
        if communication_is_ice(pub):
            need_topic = True
    if need_topic:
        if component.language.lower() == "cpp":
            ptr = ""
            type = "IceStorm::TopicManagerPrx::checkedCast"
        else:
            ptr = "Ptr"
            type = "topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>"
        result += Template(TOPIC_MANAGER_STR).substitute(ptr=ptr, type=type)
    return result



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

def publish(component):
    result = ""
    proxy_list = []
    for pba in component.publishes:
        if type(pba) == str:
            pb = pba
        else:
            pb = pba[0]
        if communication_is_ice(pba):
            if component.language.lower() == "cpp":
                result += "IceStorm::TopicPrx " + pb.lower() + "_topic;\n"
            else:
                result += "std::shared_ptr<IceStorm::TopicPrx> " + pb.lower() + "_topic;\n"
            result += PUBLISHES_STR.replace("<NORMAL>", pb).replace("<LOWER>", pb.lower())
            if component.language.lower() == "cpp":
                result += "Ice::ObjectPrx " + pb.lower() + "_pub = " + pb.lower() + "_topic->getPublisher()->ice_oneway();\n"
                result += "" + pb.lower() + "_pubproxy = " + pb + "Prx::uncheckedCast(" + pb.lower() + "_pub);\n"
                result += "mprx[\"" + pb + "Pub\"] = (::IceProxy::Ice::Object*)(&" + pb.lower() + "_pubproxy);\n"
            else:
                result += "auto " + pb.lower() + "_pub = " + pb.lower() + "_topic->getPublisher()->ice_oneway();\n"
                result += "" + pb.lower() + "_pubproxy = Ice::uncheckedCast<" + pb + "Prx>(" + pb.lower() + "_pub);\n"
                proxy_list.append(pb.lower() + "_pubproxy")
    return result, proxy_list

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
	<NORMAL>Ptr <LOWER>I_ = <CHANGE3>(worker);
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

def subscribes_to(component):
    result = ""
    for name, num in get_name_number(component.subscribesTo):
        nname = name
        while type(nname) != type(''):
            nname = name[0]
        if communication_is_ice(name):
            if component.language.lower() == "cpp":
                change1 = "IceStorm::TopicPrx"
                change2 = "Ice::ObjectPrx"
                change3 = " new <NORMAL>I"
                change4 = "Ice::ObjectPrx"
            else:
                change1 = "std::shared_ptr<IceStorm::TopicPrx>"
                change2 = "Ice::ObjectPrxPtr"
                change3 = " std::make_shared <<NORMAL>I>"
                change4 = "auto"

            result += SUBSCRIBESTO_STR.replace("<CHANGE1>", change1).replace("<CHANGE2>", change2).replace("<CHANGE3>",
                                                                                                          change3).replace(
                "<CHANGE4>", change4).replace("<NORMAL>", nname).replace("<LOWER>", nname.lower()).replace(
                "<PROXYNAME>", nname.lower() + num).replace("<PROXYNUMBER>", num)
    return result


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

def implements(component):
    result = ""
    for ima in component.implements:
        if type(ima) == str:
            im = ima
        else:
            im = ima[0]
        if communication_is_ice(ima):
            if component.language.lower() == "cpp":
                cpp = "<NORMAL>I *<LOWER> = new <NORMAL>I(worker);"
            else:
                cpp = "auto <LOWER> = std::make_shared<<NORMAL>I>(worker);"
            result += IMPLEMENTS_STR.replace("<C++_VERSION>", cpp).replace("<NORMAL>", im).replace("<LOWER>", im.lower())

    return result

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

def requires(component):
    result = ""
    proxy_list = []
    for iface, num in get_name_number(component.requires):
        if communication_is_ice(iface):
            if component.language.lower() == "cpp":
                cpp = "<PROXYNAME>_proxy = <NORMAL>Prx::uncheckedCast( communicator()->stringToProxy( proxy ) );"
            else:
                cpp = "<PROXYNAME>_proxy = Ice::uncheckedCast<<NORMAL>Prx>( communicator()->stringToProxy( proxy ) );"
            name = iface[0]
            result += REQUIRE_STR.replace("<C++_VERSION>", cpp).replace("<NORMAL>", name).replace("<LOWER>",
                                                                                                 name.lower()).replace(
                "<PROXYNAME>", name.lower() + num).replace("<PROXYNUMBER>", num)
        if component.language.lower() == "cpp":
            result += "mprx[\"" + name + "Proxy" + num + "\"] = (::IceProxy::Ice::Object*)(&" + name.lower() + num + "_proxy);//Remote server proxy creation example\n"
        else:
            proxy_list.append(name.lower() + num + "_proxy")
    return result, proxy_list


def specificworker_creation(language, proxy_list):
    result = "\n"
    if language.lower() == "cpp":
        var_name = 'm'
    else:
        var_name = 't'
        if proxy_list:
            result += "tprx = std::make_tuple(" + ",".join(proxy_list) + ");\n"
        else:
            result += "tprx = std::tuple<>();\n"
    result += "SpecificWorker *worker = new SpecificWorker({}prx);\n".format(var_name)
    return result

UNSUBSCRIBE_STR = """
try
{
	std::cout << \"Unsubscribing topic: ${name} \" <<std::endl;
	${name}_topic->unsubscribe( ${name} );
}
catch(const Ice::Exception& ex)
{
	std::cout << \"ERROR Unsubscribing topic: $name \" <<std::endl;
}
"""

def unsubscribe_code(component):
    result = "\n"
    for iface in component.subscribesTo:
        iface_name = iface
        while type(iface_name) != type(''):
            iface_name = iface[0]
        if communication_is_ice(iface):
            result = Template(UNSUBSCRIBE_STR).substitute(name=iface_name.lower())
    return result
