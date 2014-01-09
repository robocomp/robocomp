#-*- coding: utf-8 -*-
#fetch DSLEditor from external server
import sys,os,urllib

url32_Ice = "--output-document DSLEditor.tar.gz 'http://robocloud.unex.es/public.php?service=files&t=e1e5f67d74870f1c471cb6cb9f8d5b21&download'"
url64_Ice = "--output-document DSLEditor.tar.gz 'http://robocloud.unex.es/public.php?service=files&t=5f841a6e47856e678a353a697b1f3902&download'"
url32_Pattern = "--output-document DSLEditor.tar.gz  'http://robocloud.unex.es/public.php?service=files&t=28c2613206f8395e697c77e049202c0f&download'"
url64_Pattern = "--output-document DSLEditor.tar.gz 'http://robocloud.unex.es/public.php?service=files&t=9f4c2ebeea988aa23dcd0f8e4f7a48a3&download'"
url32_Agents = "--output-document DSLEditor.tar.gz 'http://robocloud.unex.es/public.php?service=files&t=8dada5855bac445ce063faf63f822909&download'"
url64_Agents = "--output-document DSLEditor.tar.gz 'http://robocloud.unex.es/public.php?service=files&t=2760ca258df1e12428d6cd2992446790&download'"

def yesNoInput(message):
  answer = ''
  while answer != 'yes' and answer != 'no':
    if message[-1] != ' ': message = message + ' '
    answer = raw_input(message)
  return answer



def selectVersion():
  print 'Which version do you want to download?'
  print ' Ice:'
  print '   1. 32 bits'
  print '   2. 64 bits'
  print ' Pattern:'
  print '   3. 32 bits'
  print '   4. 64 bits'
  print ' Agents:'
  print '   5. 32 bits'
  print '   6. 64 bits'
  version = raw_input('Version: ')
  if version == '1':
    url = url32_Ice
    name = 'DSLEditor_32'
  elif version == '2':
    url = url64_Ice
    name = 'DSLEditor_64'
  elif version == '3':
    url = url32_Pattern
    name = 'DSLEditor_Patterns_32'
  elif version == '4':
    url = url64_Pattern
    name = 'DSLEditor_Patterns_64'
  elif version == '5':
    url = url32_Agents
    name = 'DSLEditor_Agents_32'
  elif version == '6':
    url = url64_Agents
    name = 'DSLEditor_Agents_64'
  else:
    print 'No available version'
    sys.exit(0)
  return url,name

def checkExistence():
  path = os.environ['ROBOCOMP'] + '/Tools/RoboCompDSL/'
  print path
  if os.path.exists(path):
    if yesNoInput('You already have a RCDslEditor version. Delete it? (yes/no)') == 'yes':
      os.system('rm -r '+path)
    else:
      if yesNoInput('Try to download anyway? (yes/no)') == 'no':
        sys.exit(0)

def download(url,name):
  ret = os.system('wget '+url)
  if ret == 0:
    ret = os.system('tar -xzf DSLEditor.tar.gz')
    
    if (name !='DSLEditor_Agents_32' and name !='DSLEditor_Agents_64'):
		os.system('mv '+name+'/RoboCompDSL RoboCompDSL')
    os.system('rm DSLEditor.tar.gz')
    if (name !='DSLEditor_Agents_32' and name !='DSLEditor_Agents_64'):
		os.system('rm -r '+name)
  else:
    print 'Error downloading, please try again'
    sys.exit(0)


if __name__ == '__main__':
  checkExistence()
  url,name = selectVersion()
  download(url,name)
