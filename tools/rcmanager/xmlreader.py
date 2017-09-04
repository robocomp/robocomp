#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------
#  -----  rcmanager  -----
#  -----------------------
#  An ICE component manager.
#
#    Copyright (C) 2009-2015 by RoboLab - University of Extremadura
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#

#
# CODE BEGINS
#

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
