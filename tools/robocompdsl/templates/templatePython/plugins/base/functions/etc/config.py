from dsl_parsers.parsing_utils import communication_is_ice, get_name_number
from rcportchecker import RCPortChecker
from templates.common.templatedict import TemplateDict

STORM_TOPIC_MANAGER_STR = """\
# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
"""


def get_existing_port(interface_name):
    port = 0
    ports = RCPortChecker().search_interface_ports_by_name(interface_name)
    if ports is not None:
        ports = [i for i in ports.keys() if i > 0]
        if ports is not None and len(ports):
            port = min(ports)
    return port

class etc_config(TemplateDict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['config_implements_endpoints'] = self.config_implements_endpoints()
        self['config_subscribes_endpoints'] = self.config_subscribes_endpoints()
        self['config_requires_proxies'] = self.config_requires_proxies()
        self['storm_topic_manager'] = self.storm_topic_manager()

    def config_requires_proxies(self):
        result = ""
        for interface, num in get_name_number(self.component.requires):
            if communication_is_ice(interface):
                port = 0
                if interface.name == 'DifferentialRobot': port = 10004
                elif interface.name == 'Laser': port = 10003
                else:
                    port = get_existing_port(interface.name)
                result += interface.name + num + "Proxy = " + interface.name.lower() + ":tcp -h localhost -p " + str(
                    port) + "\n"
        if result != "":
            result = '# Proxies for required interfaces\n' + result + '\n\n'
        return result

    def storm_topic_manager(self):
        result = ""
        if len(self.component.publishes + self.component.subscribesTo) > 0:
            result += STORM_TOPIC_MANAGER_STR
        return result

    def config_implements_endpoints(self):
        result = ""
        for interface in self.component.implements:
            if communication_is_ice(interface):
                port = get_existing_port(interface.name)
                result += interface.name + f".Endpoints=tcp -p {port}\n"
        if result != "":
            result = '# Endpoints for implements interfaces\n' + result + '\n\n'
        return result

    def config_subscribes_endpoints(self):
        result = ""
        for interface in self.component.subscribesTo:
            if communication_is_ice(interface):
                port = get_existing_port(interface.name)
                result += interface.name + f"Topic.Endpoints=tcp -p {port}\n"
        if result != "":
            result = '# Endpoints for subscriptions interfaces\n' + result + '\n\n'
        return result
