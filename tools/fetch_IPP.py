#!/usr/bin/env python
#-*- coding: utf-8 -*-
#fetch IPP from external server
import sys,os,urllib

url = "--output-document IPP.tar.gz 'https://robocloud.unex.es/public.php?service=files&t=ac16d5368704df20a691f8b496236378&download'"

def selectVersion():
  version = os.popen("uname -a")
  if "x86_64" in version:
    return "64"
  else:
    return "32" 

def download(version):
  os.system('cd /'
  ret = os.system('sudo wget -c ' + url)
  if ret == 0:
    ret = os.system('sudo tar -xzf IPP.tar.gz')
    os.system('cd opt/intel/ipp')  
    if version == "32":
      os.system('sudo echo "/opt/intel/ipp/6.1.1.042/emt64/sharedlibs/" >> /etc/ld.so.conf'
      os.system("sudo rm -r opt/intel/ipp/6.1.1.042"
    if version == "64":
      os.system('sudo echo "/opt/intel/ipp/6.1.1.042/emt64/sharedlibs/" >> /etc/ld.so.conf'
      os.system("sudo rm -r opt/intel/ipp/6."
    #os.system('rm /IPP.tar.gz')
    os.system('sudo ldconfig')
  else:
    print 'Error downloading, please try again'
    sys.exit(0)


if __name__ == '__main__':
  checkExistence()
  version = selectVersion()
  download(version)

