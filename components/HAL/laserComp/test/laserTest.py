#!/usr/bin/env python2.4

#    Copyright (C) 2010 by RoboLab
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
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, traceback, Ice, time

Ice.loadSlice('../../../Interfaces/Laser.ice')

import RoboCompLaser

ic = None

class Client (Ice.Application):
	def run (self, argv):
		global ic
		status = 0
		self.shutdownOnInterrupt()

		ic = self.communicator()

		print 'Get connection config'
		try:
			proxyString = ic.getProperties().getProperty('LaserProxy')
			print '  Proxy string: ', proxyString
		except:
			print '  Cannot get LaserProxy property.'
			return

		print 'Connect as a generic proxy'
		self.laserPrx = None
		basePrx = None
		try:
			basePrx = self.communicator().stringToProxy(proxyString)
			print '  LaserPrx: ', type(basePrx)
		except:
			traceback.print_exc()
			print '  stringToProxy :-('
			status = 1

		print 'Cast the proxy to a LASER proxy'
		try:
			self.laserPrx = RoboCompLaser.LaserPrx.checkedCast(basePrx)
		except:
			traceback.print_exc()
			print '  checkedCast :-('
			status = 1

		if status != 1:
			conf = self.laserPrx.getLaserConfData()
			print 'maxMeasures = ' + str(conf.maxMeasures)
			print 'maxDegrees = ' + str(conf.maxDegrees)
			print 'maxRange = ' + str(conf.maxRange)
			print 'minRange = ' + str(conf.minRange)
			print 'iniRange = ' + str(conf.iniRange)
			print 'endRange = ' + str(conf.endRange)
			print 'cluster = ' + str(conf.cluster)
			print 'sampleRate = ' + str(conf.sampleRate)


			for x in range(10):
				data = self.laserPrx.getLaserData()
				print len(data)
				for indx in range(10):
					print '[' + str(data[indx].angle) + ', ' + str(data[indx].dist) + '], '
				print ''
				time.sleep(1)
		else:
			print ':\'('

	def stop (self):
		if self.communicator():
			try:
				self.communicator().destroy()
			except:
				traceback.print_exc()
				status = 1

c = Client()
c.main(sys.argv)




