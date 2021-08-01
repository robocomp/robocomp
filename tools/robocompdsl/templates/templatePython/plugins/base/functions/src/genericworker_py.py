import datetime
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from templates.common.templatedict import TemplateDict





class src_genericworker_py(TemplateDict):
    def __init__(self, component):
        super(src_genericworker_py, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['requires_proxies'] = self.requires_proxies()
        self['publishes_proxies'] = self.publishes_proxies()


    # TODO: Refactor this and publishes with a zip?
    def requires_proxies(self):
        result = ""
        for req, num in get_name_number(self.component.requires):
            if isinstance(req, str):
                rq = req
            else:
                rq = req[0]
            if communication_is_ice(req):
                result += "self." + rq.lower() + num + "_proxy = mprx[\"" + rq + "Proxy" + num + "\"]\n"
            else:
                result += "self." + rq.lower() + "_proxy = ServiceClient" + rq + "()\n"
        return result

    def publishes_proxies(self):
        result = ""
        for pb, num in get_name_number(self.component.publishes):
            if isinstance(pb, str):
                pub = pb
            else:
                pub = pb[0]
            if communication_is_ice(pb):
                result += "self." + pub.lower() + num + "_proxy = mprx[\"" + pub + num + "\"]\n"
            else:
                result += "self." + pub.lower() + "_proxy = Publisher" + pub + "()\n"
        return result
