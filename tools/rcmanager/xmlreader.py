
import xmltodict, pprint, json, os
from xml.etree import ElementTree

def read_from_file(filename, printOnScreen=False):
    if printOnScreen:
        print "Opening file:", filename
    try:
        with open(filename) as f:
            data = f.read()
            if '.xml' in filename:
                return read_from_text(data, 'xml')
            elif '.json' in filename:
                js = json.load(filename)
                if printOnScreen:
                    print "JSON file:", json.dumps(js, indent=4, sort_keys=True)
                return js
    except:
        print "Filename ", filename, "does not exist"

def read_from_text(data, type, printOnScreen=False):
    if type == 'xml':
        xml = xmltodict.parse(data)
        if printOnScreen:
            print "XML file:", json.dumps(xml, indent=4, sort_keys=True)
        return xml

def validate_xml(xml):
    try:
        ElementTree.fromstring(xml)
    except Exception, e:
        return False
    return True

def get_text_from_file(filename):
    try:
        file = open(filename, 'r')
        data = file.read()
    except Exception, e:
        raise e
    return data
