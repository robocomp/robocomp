import xmllib, string, sys

sys.path.append('/usr/local/share/agm/')
from AGGL import *

class AGMWorldModelParser(xmllib.XMLParser):
	def __init__(self, data):
		xmllib.XMLParser.__init__(self)
		self.world = False
		self.currentSymbol = None

		self.nodes = dict()
		self.links = list()

		self.feed(data)

	def handle_data(self, data):
		pass

	def start_AGMModel(self, attrs):
		if not self.world:
			self.world = True
		else:
			print 'errorrr'
			sys.exit(-1)

	def end_AGMModel(self):
		if self.world:
			self.world = False
		else:
			print 'errorrr'
			sys.exit(-1)

	def start_symbol(self, attrs):
		ids = str(attrs['id'])
		self.currentSymbol = ids
		x = ''
		if 'x' in attrs: x = attrs['x']
		y = ''
		if 'y' in attrs: y = attrs['y']
		self.nodes[ids] = AGMSymbol(ids, attrs['type'], [x, y])

	def end_symbol(self):
		self.currentSymbol = None

	def start_link(self, attrs):
		src = attrs['src']
		dst = attrs['dst']
		if not src in self.nodes:
			print 'No src node', src
			sys.exit(-1)
		if not dst in self.nodes:
			print 'No dst node', dst
			sys.exit(-1)
		enabled = False

		enabled = True
		try:
			if attrs['enabled'].lower() in ["false", "f", "fa", "fal", "fals", "0", "n", "no"]:
				enabled = False
		except:
			pass

		self.links.append(AGMLink(src, dst, attrs['label'], enabled=enabled))


	def end_link(self):
		pass

	def start_attribute(self, attrs):
		self.nodes[self.currentSymbol].attributes[attrs['key']] = attrs['value']
	def start_linkAttribute(self, attrs):
		#-1 means currentLink (last link)
		self.links[-1].attributes[attrs['key']] = attrs['value']

	def end_attribute(self):
		pass
	def end_linkAttribute(self):
		pass

## Makes a graph with the information contained in a XML file
def graphFromXMLFile(path):
	with open(path, 'r') as filehandle:
		data = filehandle.read()
		return graphFromXMLText(data)

def graphFromXMLText(text):
	#print 'graphFromXMLText'
	#print text
	parser = AGMWorldModelParser(str(text))
	parser.close()
	return AGMGraph(parser.nodes, parser.links)



