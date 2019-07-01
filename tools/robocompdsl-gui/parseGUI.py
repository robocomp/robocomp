

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

dictionary = LoadInterfaces.load_all_interfaces(LoadInterfaces, "/opt/robocomp/interfaces/IDSLs")
#print( "Todas las posibilidades\n", dictionary)

#obtener el fichero para un interfaz específico
#print("Fichero para DifferentialRobot")
#print(dictionary["DifferentialRobot"])

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

def check_imported_interfaces(cdsl_dictionary):
    imported_interfaces_list = cdsl_dictionary['imports']
    communication_interfaces_list = cdsl_dictionary['requires'] + cdsl_dictionary['implements'] + cdsl_dictionary[
        'subscribesTo'] + cdsl_dictionary['publishes']
    files_to_import = [] #required idsl files to be imported

    for com in communication_interfaces_list:
        files = get_files_from_interface(com)
        files_to_import.extend(files)

    for file_required in files_to_import:
        if file_required not in (imported_interfaces_list):
            return file_required
    return None

def check_file(cdslFile):
    #comprobar fichero y como resultado devolver lista con < nº_linea, texto, error > mostrar por pantalla
    if cdslFile.endswith(".cdsl"):

        file_errors = []
        inputText = open(cdslFile, 'r').read()
        file_dict = CDSLParsing.fromString(inputText)

        interface_missing = check_imported_interfaces(file_dict)
        if interface_missing != None:#comprobar si es lista
            if isinstance(interface_missing, list):
                for i in interface_missing:
                    print ("Interface " + i + " must be imported")
            else:
                print ("Interface "+ interface_missing + " must be imported")

        if file_dict['innermodelviewer'] == True and file_dict['language'] != 'cpp' and file_dict['gui'][0] != 'Qt' :
            print("Incompatible innermodelViewer configuration")

    else:
        print("Invalid file extension")


#get_interfaces_from_file("CGR.idsl")
#get_files_from_interface("MSKBodyEvent")
#check_file("cdslFiles/eleComp.cdsl")
#check_file("cdslFiles/Comp1.cdsl")
#check_file("cdslFiles/Comp2.cdsl")
#check_file("cdslFiles/Comp3.cdsl")
#check_file("cdslFiles/Comp4.cdsl")
#check_file("cdslFiles/Comp5.cdsl")
#check_file("cdslFiles/Comp6.cdsl")
#check_file("cdslFiles/Comp7.cdsl")
