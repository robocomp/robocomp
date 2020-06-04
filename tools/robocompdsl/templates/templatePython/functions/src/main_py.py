import datetime
import sys
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number, IDSLPool

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
adapter.add(${iface_name}I(worker), ic.stringToIdentity('${iface_name_lower}'))
adapter.activate()

"""


class TemplateDict(dict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['import_qtwidgets'] = self.import_qtwidgets()
        self['component_name'] = self.component.name
        self['app_creation'] = self.app_creation()
        self['storm_topic_manager_creation'] = self.storm_topic_manager_creation()
        self['require_proxy_creation'] = self.require_proxy_creation()
        self['publish_proxy_creation'] = self.publish_proxy_creation()
        self['implements_adapters_creation'] = self.implements_adapters_creation()
        self['subscribes_adapters_creation'] = self.subscribes_adapters_creation()
        self['ros_service_and_subscribe_creation'] = self.ros_service_and_subscribe_creation()

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

    # TODO: refactor. Check ros type conversions in cpp template
    def ros_service_and_subscribe_creation(self):
        pool = self.component.idsl_pool
        result = ""
        if self.component.usingROS:
            result += "<TABHERE>rospy.init_node(\"" + self.component.name + "\", anonymous=True)\n"
        for sub in self.component.subscribesTo:
            nname = sub.name
            module = pool.module_providing_interface(nname)
            if module is None:
                raise ValueError('\nCan\'t find module providing %s\n' % nname)
            if not communication_is_ice(sub):
                for interface in module['interfaces']:
                    if interface['name'] == nname:
                        for mname in interface['methods']:
                            method = interface['methods'][mname]
                            for p in method['params']:
                                s = "\"" + mname + "\""
                                if p['type'] in ('float', 'int'):
                                    result += "<TABHERE>rospy.Subscriber(" + s + ", " + p['type'].capitalize() + "32, worker.ROS" + method['name'] + ")\n"
                                elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                                    result += "<TABHERE>rospy.Subscriber(" + s + ", UInt" + p['type'].split('t')[1] + ", worker.ROS" + method['name'] + ")\n"
                                elif p['type'] in IDSLPool.getRosTypes():
                                    result += "<TABHERE>rospy.Subscriber(" + s + ", " + p['type'].capitalize() + ", worker.ROS" + method['name'] + ")\n"
                                elif '::' in p['type']:
                                    result += "<TABHERE>rospy.Subscriber(" + s + ", " + p['type'].split('::')[1] + ", worker.ROS" + method['name'] + ")\n"
                                else:
                                    result += "<TABHERE>rospy.Subscriber(" + s + ", " + p['type'] + ", worker.ROS" + method['name'] + ")\n"

        for imp in self.component.implements:
            nname = imp.name
            module = pool.module_providing_interface(nname)
            if module is None:
                print('\nCan\'t find module providing', nname, '\n')
                sys.exit(-1)
            if not communication_is_ice(imp):
                for interface in module['interfaces']:
                    if interface['name'] == nname:
                        for mname in interface['methods']:
                            method = interface['methods'][mname]
                            s = "\"" + mname + "\""
                            result += "<TABHERE>rospy.Service(" + s + ", " + mname + ", worker.ROS" + method['name'] + ")\n"
        return result

    def import_qtwidgets(self):
        result = ""
        if self.component.gui is not None:
            result += 'from PySide2 import QtWidgets\n'
        return result

    def app_creation(self):
        result = ""
        if self.component.gui is not None:
            result += 'app = QtWidgets.QApplication(sys.argv)\n'
        else:
            result += 'app = QtCore.QCoreApplication(sys.argv)\n'
        return result
