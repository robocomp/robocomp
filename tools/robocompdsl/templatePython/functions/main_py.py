from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number

SUBSCRIBESTO_STR = """
${iface_name}_adapter = ic.createObjectAdapter("${iface_name}Topic")
${iface_name_lower}I_ = ${iface_name}I(worker)
${iface_name_lower}_proxy = ${iface_name}_adapter.addWithUUID(${iface_name_lower}I_).ice_oneway()

subscribeDone = False
while not subscribeDone:
    try:
        ${iface_name_lower}_topic = topicManager.retrieve("${iface_name}")
        subscribeDone = True
    except Ice.Exception as e:
        print("Error. Topic does not exist (creating)")
        time.sleep(1)
        try:
            ${iface_name_lower}_topic = topicManager.create("${iface_name}")
            subscribeDone = True
        except:
            print("Error. Topic could not be created. Exiting")
            status = 0
qos = {}
${iface_name_lower}_topic.subscribeAndGetPublisher(qos, ${iface_name_lower}_proxy)
${iface_name}_adapter.activate()

"""


TOPIC_MANAGER_STR ="""
# Topic Manager
proxy = ic.getProperties().getProperty("TopicManager.Proxy")
obj = ic.stringToProxy(proxy)
try:
    topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
except Ice.ConnectionRefusedException as e:
    print('Cannot connect to IceStorm! ('+proxy+')')
    status = 1
"""

def storm_topic_manager_creation(component):
    result = ""
    try:
        need_storm = False
        for pub in component.publishes:
            if communication_is_ice(pub):
                need_storm = True
        for sub in component.subscribesTo:
            if communication_is_ice(sub):
                need_storm = True
        if need_storm:
            result += TOPIC_MANAGER_STR
    except:
        pass
    return result

REQUIRE_STR = """
# Remote object connection for ${iface_name}
try:
    proxyString = ic.getProperties().getProperty('${iface_name}${num}Proxy')
    try:
        basePrx = ic.stringToProxy(proxyString)
        ${iface_name_lower}${num}_proxy = ${iface_name}Prx.uncheckedCast(basePrx)
        mprx["${iface_name}Proxy${num}"] = ${iface_name_lower}${num}_proxy
    except Ice.Exception:
        print('Cannot connect to the remote object (${iface_name})', proxyString)
        #traceback.print_exc()
        status = 1
except Ice.Exception as e:
    print(e)
    print('Cannot get ${iface_name}Proxy property.')
    status = 1

"""

def require_proxy_creation(component):
    result = ""
    for iface, num in get_name_number(component.requires):
        if communication_is_ice(iface):
            name = iface[0]
            result += Template(REQUIRE_STR).substitute(iface_name=name, iface_name_lower=name.lower(), num=num)
    return result

PUBLISHES_STR = """
# Create a proxy to publish a ${iface_name} topic
topic = False
try:
    topic = topicManager.retrieve("${iface_name}")
except:
    pass
while not topic:
    try:
        topic = topicManager.retrieve("${iface_name}")
    except IceStorm.NoSuchTopic:
        try:
            topic = topicManager.create("${iface_name}")
        except:
            print('Another client created the ${iface_name} topic? ...')
pub = topic.getPublisher().ice_oneway()
${iface_name_lower}Topic = ${iface_name}Prx.uncheckedCast(pub)
mprx["${iface_name}Pub"] = ${iface_name_lower}Topic

"""

def publish_proxy_creation(component):
    result = ""
    for iface, num in get_name_number(component.publishes):
        if communication_is_ice(iface):
            name = iface[0]
            result += Template(PUBLISHES_STR).substitute(iface_name=name, iface_name_lower=name.lower())
    return result

IMPLEMENTS_STR = """
adapter = ic.createObjectAdapter('${iface_name}')
adapter.add(${iface_name}I(worker), ic.stringToIdentity('${iface_name_lower}'))
adapter.activate()
"""

def implements_adapters_creation(component):
    result = ""
    for iface in component.implements:
        if communication_is_ice(iface):
            name = iface[0]
            result += Template(IMPLEMENTS_STR).substitute(iface_name=name, iface_name_lower=name.lower())
    return result
