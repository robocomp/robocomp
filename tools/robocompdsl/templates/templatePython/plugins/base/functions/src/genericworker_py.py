import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from templates.common.templatedict import TemplateDict
from templates.templatePython.plugins.base.functions import function_utils as utils

SLICE_LOAD_STR = """\
Ice.loadSlice("-I ./src/ --all ./src/${interface_name}.ice")
"""

LIST_CLASSES_STR = """\
class ${list_type}(list):
    def __init__(self, iterable=list()):
        super(${list_type}, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, ${item_type})
        super(${list_type}, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, ${item_type})
        super(${list_type}, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, ${item_type})
        super(${list_type}, self).insert(index, item)

setattr(${module_name}, "${list_type}", ${list_type})

"""


class src_genericworker_py(TemplateDict):
    def __init__(self, component):
        super(src_genericworker_py, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['load_slice_and_create_imports'] = self.load_slice_and_create_imports()
        self['implements_and_subscribes_imports'] = self.implements_and_subscribes_imports()
        self['requires_proxies'] = self.requires_proxies()
        self['publishes_proxies'] = self.publishes_proxies()
        self['create_lists_classes'] = self.create_lists_classes()

    def implements_and_subscribes_imports(self):
        result = ""
        for im in self.component.implements + self.component.subscribesTo:
            if communication_is_ice(im):
                result += 'import ' + im.name.lower() + 'I\n'
        return result

    # TODO: Check if can be merged with SERVANT_PY.py slice_loading function
    def load_slice_and_create_imports(self, includeDirectories=None):
        result = ""
        import os
        for imp in sorted(set(self.component.recursiveImports + self.component.imports)):
            file_name = os.path.basename(imp)
            name = os.path.splitext(file_name)[0]
            result += Template(SLICE_LOAD_STR).substitute(interface_name=name)
            module = DSLFactory().from_file(file_name, includeDirectories=includeDirectories)
            result += f"import {module['name']}\n"
        return result



    def create_lists_classes(self):
        result = ""
        for idsl in sorted(set(self.component.recursiveImports + self.component.imports)):
            try:
                module = self.component.idsl_pool.module_providing_interface(idsl.split('.')[0])
            except Exception as e:
                print(e.message)
                exit(-1)

            if module is not None:  # For modules without interface
                for sequence in module['sequences']:
                    item_type = utils.get_type_string(sequence['typeSequence'], module['name'])
                    if item_type == 'bytes': continue
                    result += Template(LIST_CLASSES_STR).substitute(list_type=sequence['name'].split('/')[1],
                                                                    item_type=item_type,
                                                                    module_name=module['name'])
        return result


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
                result += "self." + pub.lower() + num + "_proxy = mprx[\"" + pub + "Pub" + num + "\"]\n"
            else:
                result += "self." + pub.lower() + "_proxy = Publisher" + pub + "()\n"
        return result
