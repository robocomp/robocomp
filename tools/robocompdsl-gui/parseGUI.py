

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

def check_file(cdslFile):
    if cdslFile.endswith(".cdsl"):
        inputText = open(cdslFile, 'r').read()
        file_dict = CDSLParsing.fromString(inputText)  # solve, it already checks if interface exists

        interface_missing = check_imported_interfaces(file_dict)
        if interface_missing != None:
            print ("Interface " + interface_missing + " must be imported")

        if file_dict['innermodelviewer'] == True and file_dict['language'] != 'cpp' and file_dict['gui'][0] != 'Qt' :
            print("Incompatible innermodelViewer configuration")

    else:
        print("Invalid file extension")

check_file("eleComp.cdsl")
