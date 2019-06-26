

import os
from parseIDSL import IDSLParsing
from parseCDSL import CDSLParsing
from parseCDSL import LoadInterfaces

dictionary = LoadInterfaces.load_all_interfaces(LoadInterfaces, "/opt/robocomp/interfaces/IDSLs")
#print( "Todas las posibilidades\n", dictionary)

#obtener el fichero para un interfaz específico

#print("Fichero para DifferentialRobot")
#print(dictionary["DifferentialRobot"])


def check_imported_interfaces(cdsl_dictionary):

    imported_interfaces_list = cdsl_dictionary['imports']
    communication_interfaces_list = cdsl_dictionary['requires'] + cdsl_dictionary['implements'] + cdsl_dictionary[
        'subscribesTo'] + cdsl_dictionary['publishes']
    interfacesToImp = [] #required interfaces to be imported

    #Search which interfaces must be imported
    for r in communication_interfaces_list:#for each interface in communication
        for d in dictionary[r]:#for each file to be imported from each interface
            interfacesToImp.append(d)

    for interface_required in interfacesToImp:
        interface_required = interface_required
        if interface_required not in (imported_interfaces_list):
            return interface_required

    return None

def get_interfaces_from_file(inputFile):
    interfaces = []
    for interf, files_list in dictionary.items():
        for file in files_list:
            if file == inputFile:
                interfaces.append(interf)
    return interfaces

def get_files_from_interface(inputInterface):
    files = []
    for i in dictionary[inputInterface]:
        files.append(i)
    return files


def check_file(cdslFile):
    #comprobar fichero y como resultado devolver lista con < nº_linea, texto, error > mostrar por pantalla
    if cdslFile.endswith(".cdsl"):

        file_errors = []
        inputText = open(cdslFile, 'r').read()
        file_dict = CDSLParsing.fromString(inputText)

        interface_missing = check_imported_interfaces(file_dict)
        if interface_missing != None:
            print ("Interface " + interface_missing + " must be imported")

        if file_dict['innermodelviewer'] == True and file_dict['language'] != 'cpp' and file_dict['gui'][0] != 'Qt' :
            print("Incompatible innermodelViewer configuration")

    else:
        print("Invalid file extension")


get_interfaces_from_file("CGR.idsl")
get_files_from_interface("MSKBodyEvent")
#check_file("cdslFiles/eleComp.cdsl")
#check_file("cdslFiles/Comp1.cdsl")
#check_file("cdslFiles/Comp2.cdsl")
#check_file("cdslFiles/Comp3.cdsl")
#check_file("cdslFiles/Comp4.cdsl")
#check_file("cdslFiles/Comp5.cdsl")
#check_file("cdslFiles/Comp6.cdsl")
#check_file("cdslFiles/Comp7.cdsl")
