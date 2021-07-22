Components publishing and subscribing needs the rcnode application to be running.

rcnode internally will launch the icebox application with some predefined configuration.

$ icebox --Ice.Config=config.icebox

- config.icebox contents:

IceBox.Service.IceStorm=IceStormService,34:createIceStorm --Ice.Config=config.icestorm

- config.icestorm contents:

IceStorm.TopicManager.Endpoints=tcp -p 9999
IceStorm.Publish.Endpoints=tcp -p 10000
IceStorm.Transient=1
Freeze.DbEnv.IceStorm.DbHome=