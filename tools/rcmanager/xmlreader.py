import xmltodict, pprint, json, os


def xml_reader(filename):
	print filename
	try:
		with open(filename) as f:
			data = f.read()
			if '.xml' in filename:
				xml = xmltodict.parse(data)
				print json.dumps(xml, indent=4, sort_keys=True)
				return xml
			elif '.json' in filename:
				js = json.load(filename)
				print json.dumps(js, indent=4, sort_keys=True)
				return js
	except:
		print "Filename ", filename,  "does not exist"
