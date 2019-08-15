import os

from parseIDSL import IDSLParsing
from parseCDSL import CDSLParsing


class LoadInterfaces:
    @staticmethod
    def get_interfaces_name(file):
        names = []
        idsl_content = IDSLParsing.fromFileIDSL(file)
        for content in idsl_content['module']['contents']:
            if content[0] == "interface":
                names.append(content[1])
        return names

    @staticmethod
    def load_all_interfaces(self, path):
        interfaces = {}
        for r, d, f in os.walk(path):
            for file in f:
                if '.idsl' in file:
                    names = self.get_interfaces_name(os.path.join(r, file))
                    for name in names:
                        if name in interfaces:
                            interfaces[name].append(file)
                        else:
                            interfaces[name] = [file]
        return interfaces

    def get_files_from_interface(inputInterface):
        files = []
        for i in dictionary[inputInterface]:
            files.append(i)
        return files

    def get_interfaces_from_file(inputFile):
        interfaces = []
        for interf, files_list in dictionary.items():
            for file in files_list:
                if file == inputFile:
                    interfaces.append(interf)
        return interfaces


dictionary = LoadInterfaces.load_all_interfaces(LoadInterfaces, "/opt/robocomp/interfaces/IDSLs")


class FileChecker:
    def check_imported_interfaces(self, cdsl_dictionary):
        imported_interfaces_list = cdsl_dictionary['imports']
        communication_interfaces_list = cdsl_dictionary['requires'] + cdsl_dictionary['implements'] + cdsl_dictionary[
            'subscribesTo'] + cdsl_dictionary['publishes']
        files_to_import = []  # required idsl files to be imported

        for com in communication_interfaces_list:
            files = LoadInterfaces.get_files_from_interface(com)
            files_to_import.extend(files)

        for file_required in files_to_import:
            if file_required not in (imported_interfaces_list):
                return file_required
        return None

    def check_text(self, file_dict, errors):
        innermodel = "innerModelViewer"
        if not errors:
            interface_missing = self.check_imported_interfaces(file_dict)
            if interface_missing is not None:
                if isinstance(interface_missing, list):  # check if list
                    for i in interface_missing:
                        msg = "Interface " + i + " must be imported"
                        errors.append((0, msg))
                else:
                    msg = "Interface " + interface_missing + " must be imported"
                    errors.append((0, msg))
            if innermodel in file_dict['options'] and file_dict['language'] != 'cpp':
                msg = "Incompatible innermodelViewer configuration"
                errors.append((0, msg))
        return errors
