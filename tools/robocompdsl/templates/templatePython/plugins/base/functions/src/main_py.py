import datetime
import sys
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number, IDSLPool
from templates.common.templatedict import TemplateDict

SUBSCRIBESTO_STR = """
${iface_name}_adapter = ic.createObjectAdapter("${iface_name}Topic")
${iface_name_lower}I_ = ${iface_name_lower}I.${iface_name}I(worker)
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


TOPIC_MANAGER_STR = """\
# Topic Manager
proxy = ic.getProperties().getProperty("TopicManager.Proxy")
obj = ic.stringToProxy(proxy)
try:
    topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
except Ice.ConnectionRefusedException as e:
    print(colored('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
    exit(1)
"""


REQUIRE_STR = """
# Remote object connection for ${iface_name}
try:
    proxyString = ic.getProperties().getProperty('${iface_name}${num}Proxy')
    try:
        basePrx = ic.stringToProxy(proxyString)
        ${iface_name_lower}${num}_proxy = ${module_name}.${iface_name}Prx.uncheckedCast(basePrx)
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
${iface_name_lower}Topic = ${module_name}.${iface_name}Prx.uncheckedCast(pub)
mprx["${iface_name}Pub"] = ${iface_name_lower}Topic

"""


IMPLEMENTS_STR = """\
adapter = ic.createObjectAdapter('${iface_name}')
adapter.add(${iface_name_lower}I.${iface_name}I(worker), ic.stringToIdentity('${iface_name_lower}'))
adapter.activate()

"""


class src_main_py(TemplateDict):
    def __init__(self, component):
        super(src_main_py, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['component_name'] = self.component.name
        self['storm_topic_manager_creation'] = self.storm_topic_manager_creation()
        self['require_proxy_creation'] = self.require_proxy_creation()
        self['publish_proxy_creation'] = self.publish_proxy_creation()
        self['implements_adapters_creation'] = self.implements_adapters_creation()
        self['subscribes_adapters_creation'] = self.subscribes_adapters_creation()

    def storm_topic_manager_creation(self):
        result = ""
        need_storm = False
        for pub in self.component.publishes:
            if communication_is_ice(pub):
                need_storm = True
        for sub in self.component.subscribesTo:
            if communication_is_ice(sub):
                need_storm = True
        if need_storm:
            result += TOPIC_MANAGER_STR
        return result

    def require_proxy_creation(self):
        result = ""
        for iface, num in get_name_number(self.component.requires):
            if communication_is_ice(iface):
                name = iface.name
                module = self.component.idsl_pool.module_providing_interface(iface.name)
                result += Template(REQUIRE_STR).substitute(iface_name=name, module_name=module['name'], iface_name_lower=name.lower(), num=num)
        return result

    def publish_proxy_creation(self):
        result = ""
        for iface, num in get_name_number(self.component.publishes):
            if communication_is_ice(iface):
                name = iface.name
                module = self.component.idsl_pool.module_providing_interface(iface.name)
                result += Template(PUBLISHES_STR).substitute(iface_name=name,
                                                             iface_name_lower=name.lower(),
                                                             module_name=module['name'])
        return result

    def implements_adapters_creation(self):
        result = ""
        for iface in self.component.implements:
            if communication_is_ice(iface):
                name = iface[0]
                result += Template(IMPLEMENTS_STR).substitute(iface_name=name, iface_name_lower=name.lower())
        return result

    def subscribes_adapters_creation(self):
        result = ""
        for sut in self.component.subscribesTo:
            if communication_is_ice(sut):
                name = sut[0]
                result += Template(SUBSCRIBESTO_STR).substitute(iface_name=name, iface_name_lower=name.lower())
        return result

