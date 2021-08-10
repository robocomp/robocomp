import docker

client = docker.from_env()

print(client.containers.run("robocomp/robocomp:focal_dsr", "pwd"))
