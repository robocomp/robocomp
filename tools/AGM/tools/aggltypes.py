#!/usr/bin/env python

import sys, re, traceback


def setToSortedList(s):
	l = list(s)
	l.sort()
	return l

def extractTypesLinksFromDomain(content):
	typeSet = set()
	for match in re.compile('\:[a-zA-Z][a-zA-Z_0-9]*\(').finditer(content):
		typeSet.add(content[match.span()[0]+1:match.span()[1]-1])
	linkSet = set()
	for match in re.compile('\([a-zA-Z][a-zA-Z_0-9]*\)').finditer(content):
		linkSet.add(content[match.span()[0]+1:match.span()[1]-1])
	return typeSet, linkSet



def extractTypesLinksFromGraph(content):
	typeSet = set()
	for match in re.compile('type=\"[a-zA-Z][a-zA-Z_0-9]*\"').finditer(content):
		typeSet.add(content[match.span()[0]+6:match.span()[1]-1])
	linkSet = set()
	for match in re.compile('label=\"[a-zA-Z][a-zA-Z_0-9]*\"').finditer(content):
		linkSet.add(content[match.span()[0]+7:match.span()[1]-1])
	return typeSet, linkSet


def wrongCall():
	print 'usage:  ', sys.argv[0], '<inputfile.aggl>  [<graphdescription.xml>]'
	sys.exit(1)


try:
	path = sys.argv[1]
except: # We don't have the domain... error
	wrongCall()

# At least we have the domain file
try:
	content = open(path, 'r').read()
except: # The domain file is wrong
	print 'Can\'t read', path
	sys.exit(1)

try:
	domainTypes, domainLinks = extractTypesLinksFromDomain(content)
except:
	traceback.print_exc()

try:
	path2 = sys.argv[2]
	# We have a second file...
	try:
		content2 = open(path2, 'r').read()
	except:
		print 'Can\'t read', path
		sys.exit(1)

	print 'b)'
	try:
		graphTypes, graphLinks = extractTypesLinksFromGraph(content2)
		print 'TYPES:', setToSortedList(graphTypes)
		print 'LINKS:', setToSortedList(graphLinks)
		print '--------'
		print '--------'
		#print 'TYPES:', setToSortedList(domainTypes-graphTypes)
		#print 'LINKS:', setToSortedList(domainLinks-graphLinks)
		print 'TYPES:', setToSortedList(graphTypes-domainTypes)
		print 'LINKS:', setToSortedList(graphLinks-domainLinks)

	except:
		traceback.print_exc()

except:
	print 'a)'
	print 'TYPES:', setToSortedList(domainTypes)
	print 'LINKS:', setToSortedList(domainLinks)







