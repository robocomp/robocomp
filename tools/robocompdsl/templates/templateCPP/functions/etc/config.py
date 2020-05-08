from dsl_parsers.parsing_utils import communication_is_ice, get_name_number


def config_implements_endpoints(component):
    result = ""
    for iface in component.implements:
        if communication_is_ice(iface):
            result += iface.name + ".Endpoints=tcp -p 0\n"
    if result != "":
        result = '# Endpoints for implements interfaces\n'+result+'\n\n'
    return result


def config_subscribes_endpoints(component):
    result = ""
    for iface in component.subscribesTo:
        if communication_is_ice(iface):
            result += iface.name + "Topic.Endpoints=tcp -p 0\n"
    if result != "":
        result = '# Endpoints for subscriptions interfaces\n' + result + '\n\n'
    return result

def config_requires_proxies(component):
    result = ""
    for iface, num in get_name_number(component.requires):
        if communication_is_ice(iface):
            port = 0
            if iface.name == 'DifferentialRobot': port = 10004
            if iface.name == 'Laser': port = 10003
            result += iface.name + num + "Proxy = " + iface.name.lower() + ":tcp -h localhost -p " + str(port)+"\n"
    if result != "":
        result = '# Proxies for required interfaces\n' + result + '\n\n'
    return result

STORM_TOPIC_MANAGER_STR = """\
# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
"""

def storm_topic_manager(component):
    result = ""
    if len(component.publishes + component.subscribesTo) > 0:
        result += STORM_TOPIC_MANAGER_STR
    return result

def get_template_dict(component):
    return {
        'config_implements_endpoints': config_implements_endpoints(component),
        'config_subscribes_endpoints': config_subscribes_endpoints(component),
        'config_requires_proxies': config_requires_proxies(component),
        'storm_topic_manager': storm_topic_manager(component)
    }