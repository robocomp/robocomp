# !/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
import json
from pathlib import Path

import docker
import typer
from docker import APIClient
import datetime
import os
import apt
from termcolor import cprint, colored
from robocomp import is_interactive
from rcconfig.rcconfig import RC_CONFIG
from rcworkspace.workspace import Workspace

app = typer.Typer(help=typer.style("Docker wrapper for Robocomp images and commands.", fg=typer.colors.GREEN))

docker_client = docker.from_env()

DEBUG = True


CURRENT_FILE_PATH = os.path.dirname(os.path.abspath(__file__))
INSTALLATION_PATH = "/opt/robocomp/share/rcbuildvalidator/"
ROBOCOMP_SRC_PATH = "/home/robolab/robocomp/"

DEBUG = False
pkg_name = "docker-ce"

branches = ['development', 'stable']


def ubuntu_images_completer(prefix, parsed_args, **kwargs):
    import requests
    response = requests.get('https://registry.hub.docker.com//v1/repositories/ubuntu/tags')
    result = []
    for tag in response.json():
        if '-' not in tag['name'] and prefix in tag['name']:
            result.append(tag['name'])
    return result


@app.command(name="install-docker")
def docker_install():
    print("Docker is not installed in your system, we need to install it now.")
    return cli.execute_command(f"sh {INSTALLATION_PATH}resources/docker_install.sh")


def stop_and_remove_container(container_name):
    try:
        container = docker_client.containers.get(container_name)
    except docker.errors.NotFound:
        pass
    else:
        container.stop()
        container.remove()


@app.command(name="check-robocomp-install")
def robocomp_installation_check(
        ubuntu_version: str = typer.Argument("20.04"),
        branch: str = typer.Argument("development"),
        install_script: Path = typer.Option(None),
        force_build: bool = typer.Option(False)
):
    options = ""
    if DEBUG:
        options = "-l -x"
    if install_script is None:
        install_script = f'{INSTALLATION_PATH}/resources/robocomp_install.sh'
    command = f"bash {options} {install_script} {branch} 1>&2"
    run_on_image(f"robocomp/robocomp:ubuntu-{ubuntu_version}", command)


@app.command(name="interactive")
def start_interactive_docker_container(
        ubuntu_version: str = typer.Argument("20.04"),
        force_build: bool = typer.Option(False, "--force-build", "-f"),
        mount_paths=None
):
    if force_build or not check_robocomp_image_exists(ubuntu_version):
        image_name_and_tag = build_robocomp_dependencies_image(ubuntu_version)
    if mount_paths is None:
        mount_paths = []
    mount_options = []
    for mount_path in mount_paths:
        if mount_path:
            mount_path = mount_path.rstrip(os.path.sep)
            mount_options.append(f"-v {os.path.expanduser(mount_path)}:/home/robolab/{mount_path.split('/')[-1]}")
    mount_options = " ".join(mount_options)
    stop_and_remove_container("robocomp_test")
    cprint(f"Starting {image_name_and_tag} container.", 'green')
    cprint(f"You can find the robocomp installation script in /home/robolab/robocomp_install.sh", 'green')
    cprint(f"To end this session execute <exit> or use <Ctrl>+d in the current bash terminal.", 'green')
    command = f"docker run --name robocomp_test -it -w /home/robolab/  \
    --user=robolab:robolab \
    --env='DISPLAY' \
    --volume='/etc/group:/etc/group:ro' \
    --volume='/etc/passwd:/etc/passwd:ro' \
    --volume='/etc/shadow:/etc/shadow:ro' \
    --volume='/etc/sudoers.d:/etc/sudoers.d:ro' \
    --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \
    --volume='{INSTALLATION_PATH}/resources/robocomp_install.sh:/home/robolab/robocomp_install.sh' \
    {image_name_and_tag} bash"
    return cli.execute_command(command)

def build_robocomp_dependencies_image(version: str = typer.Argument("20.04")):
    return build_image(base_name="ubuntu",
                       base_tag=version,
                       branch="tools_refactoring",
                       robocomp_version="dependencies",
                       push=True)

def string_to_docker_image(input_str: str ):
    import re
    # regex = r"^(?P<month>\w+)\s(?P<day>\d+)\,?\s(?P<year>\d+)"
    regex = r"(?:(?P<repo>.+)\/)?(?P<image>[^:]+)(?::(?P<tag>.+))?"
    matches = re.search(regex, input_str)
    return matches.groupdict().values()

@app.command("build")
def build_image(
        base_name="ubuntu",
        base_tag="20.04",
        image_tag=None,
        branch="tools_refactoring",
        robocomp_version="dsr-components",
        push=False):
    cli = APIClient(base_url='unix:///var/run/docker.sock')
    if image_tag is None:
        image_tag = f"{base_name}-{base_tag}"
    new_tag = f"robocomp/{robocomp_version}:{image_tag}"
    for line in cli.build(
            path=str(Path(RC_CONFIG["ROBOCOMP_SRC"]) / "docker"),
            dockerfile="Dockerfile",
            rm=True,
            tag=new_tag,
            buildargs={
                "base_image": base_name,
                "base_tag": base_tag,
                "branch": branch,
                "robocomp_version": f"{robocomp_version}"
            }):
        decoded = json.loads(line.decode("utf-8"))
        if "stream" in decoded:
            print(decoded["stream"])
        else:
            print(decoded)
    return new_tag


def check_robocomp_dependencies_image_exists(version):
    check_robocomp_image_exists(f"robocomp/dependencies")


def check_robocomp_image_exists(image_name, tag=None, version="20.04"):
    if tag is None:
        tag = f"robocomp-ubuntu{version}"
    images = docker_client.images.list(image_name)
    for image in images:
        if f'{image_name}:{tag}' in image.tags:
            return True
    return False


def save_log(container_name="robocomp_test", output_path=None):
    container = docker_client.containers.get(container_name)
    logs = container.logs()
    if logs:
        date = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        if output_path is None:
            output_file = Path(f"/tmp/{container}_{date}.txt")
        else:
            output_file = output_path / f"{container}_{date}.txt"
        if DEBUG:
            print(f"Writtig log to {colored(output_file, 'green')} ...")
        with open(output_file, 'w') as output_file:
            output_file.write(logs.decode('utf-8'))
        return output_file
    else:
        return ""

def run_on_image(image_str, command, volumes_=None, force_build=False):
    repo, image, tag = string_to_docker_image(image_str)
    base_name, base_tag = tag.split("-")
    volumes = {}
    if volumes_ is not None:
        volumes.update(volumes_)
    if force_build or not check_robocomp_image_exists(repo+"/"+image, tag):
        image_name_and_tag = build_image(base_name=base_name,
                                         base_tag=base_tag,
                                         branch="tools_refactoring",
                                         robocomp_version=image,
                                         push=True)
    else:
        image_name_and_tag = image_str
    stop_and_remove_container("robocomp_test")

    container = docker_client.containers.run(
        image_name_and_tag,
        command,
        name="robocomp_test",
        working_dir="/home/robolab/",
        user="robolab:robolab",
        environment=["DISPLAY"],
        volumes=volumes,
        detach=True,
        stderr=True
    )
    for line in container.logs(stream=True):
        print("\t"+line.strip().decode("utf-8"))

    exit_code = container.wait()
    last_log_file = save_log()
    if exit_code['StatusCode'] == 0:
        cprint("Run done OK", 'green')
    else:
        print(f"Run FAILED with return status {exit_code}. Check the {colored(last_log_file, 'red')} file.")
    return container

@app.command("build-comp")
def build_component_in_container(component_name, component_path=None, robocomp_version="robocomp/dsr:ubuntu-20.04"):
    if component_path is None:
        if component_name:
            if component_path := Workspace().find_component_path(component_name, is_interactive()):
                path_in_container = component_path
                try:
                    path_in_container = path_in_container.relative_to(Path.home())
                except ValueError:
                    pass
                volumes = {
                    component_path: {'bind': str(Path("/home/robolab/") / path_in_container), 'mode': 'rw'},
                }
                command = f"sh -x -c \"cd {path_in_container} && mkdir -p build && cd build && cmake .. && make\""
                run_on_image(robocomp_version, command, volumes)
        else:
            raise Exception()


# @app.command()
# def orensbruli():
#     image_name = "robocomp/robocomp"
#     tag = "focal_fcl_dev"
#     branch = "development"
#     if check_robocomp_image_exists(image_name, tag):
#         options = ""
#         if DEBUG:
#             options = "-l -x"
#         volumes = {
#             '/etc/group': {'bind': '/etc/group', 'mode': 'ro'},
#             '/etc/passwd': {'bind': '/etc/passwd', 'mode': 'ro'},
#             '/etc/shadow': {'bind': '/etc/shadow', 'mode': 'ro'},
#             '/etc/sudoers.d': {'bind': '/etc/sudoers.d', 'mode': 'ro'},
#             '/tmp/.X11-unix': {'bind': '/tmp/.X11-unix', 'mode': 'rw'}
#         }
#         print(colored(
#             f"Checking build of __ for Robocomp ({branch}) in {image_name}:",
#             "green"))
#         status_code = 0
#         retries = 0
#         while retries < 3:
#             try:
#                 container = docker_client.containers.run(
#                     f"{image_name}:{tag}",
#                     f"robocomp",
#                     name="robocomp_robocomp",
#                     working_dir="/home/robolab/",
#                     user="robolab:robolab",
#                     environment=["DISPLAY"],
#                     volumes=volumes,
#                     detach=True,
#                     stderr=True
#                 )
#                 for line in container.logs(stream=True):
#                     print(line.strip().decode("utf-8"))
#
#                 exit_dode = container.wait()
#             except docker.errors.APIError as e:
#                 if e.status_code == 409:
#                     print(f"{e}\nContainer exists . Trying to stop...")
#                 else:
#                     print(e)
#                 stop_and_remove_container("/robocomp_robocomp")
#                 retries += 1


@app.callback()
def _callback():
    try:
        cache = apt.cache.Cache()
        pkg = cache[pkg_name]
        if pkg.is_installed:
            cprint(f"{pkg_name} already installed", 'green')
        else:
            output = docker_install()
    except KeyError:
        output = docker_install()


if __name__ == '__main__':
    app()
