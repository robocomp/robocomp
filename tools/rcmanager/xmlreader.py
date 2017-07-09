import xmltodict, pprint, json, os


def xml_reader(filename, printOnScreen=False):
	if printOnScreen:
		print "Opening file:", filename
	try:
		with open(filename) as f:
			data = f.read()
			if '.xml' in filename:
				xml = xmltodict.parse(data)
				if printOnScreen:
					print "XML file:", json.dumps(xml, indent=4, sort_keys=True)
				return xml
			elif '.json' in filename:
				js = json.load(filename)
				if printOnScreen:
					print "JSON file:", json.dumps(js, indent=4, sort_keys=True)
				return js
	except:
		print "Filename ", filename,  "does not exist"
