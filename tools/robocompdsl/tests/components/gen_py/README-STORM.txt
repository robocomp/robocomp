1) To run IceStorm, start the icebox service: 
  
$ icebox --Ice.Config=config.icebox

2) config.icebox contents:

IceBox.Service.IceStorm=IceStormService,34:createIceStorm --Ice.Config=config.icestorm

3) config.icestorm contents:

IceStorm.TopicManager.Endpoints=tcp -p 9999
IceStorm.Publish.Endpoints=tcp -p 10000
IceStorm.Transient=1
Freeze.DbEnv.IceStorm.DbHome=
