#!/usr/bin/python3
# PYTHON_ARGCOMPLETE_OK
import datetime
import os
import shlex
import subprocess
import argcomplete
import apt
from termcolor import cprint, colored


CURRENT_FILE_PATH = os.path.dirname(os.path.abspath(__file__))
INSTALLATION_PATH = "/opt/robocomp/share/rcbuildtest/"

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

def execute_command(command, ignore_errors=False, ignore_output=False):
    if DEBUG:
        print(f"> {command}")
    try:
        if ignore_output and not DEBUG:
            return subprocess.check_call(shlex.split(command), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        else:
            return subprocess.check_call(shlex.split(command))
    except subprocess.CalledProcessError as e:
        if not ignore_errors or DEBUG:
            print(e)
        return e.returncode


def execute_commands_list(commands_list, ignore_errors=False,  ignore_output=False):
    for command in commands_list:
        execute_command(command, ignore_errors,  ignore_output)

def get_command_output(command, ignore_errors=False):
    try:
        return subprocess.check_output(shlex.split(command))
    except subprocess.CalledProcessError as e:
        if not ignore_errors:
            print(e)
        return "ERROR"

def docker_install():
    print("Docker is not installed in your system, we need to install it now.")
    return execute_command(f"sh {INSTALLATION_PATH}resources/docker_install.sh")

def remove_container(container_name):
    return execute_command(f"docker container rm {container_name}", ignore_errors=True, ignore_output=True)

def stop_and_remove_container(container_name):
    commands = [
        f"docker stop {container_name}",
        f"docker kill {container_name}",
    ]
    execute_commands_list(commands, ignore_errors=True, ignore_output=True)
    remove_container(container_name)

def start_docker_container(ubuntu_version, branch):
    stop_and_remove_container("robocomp_test")
    options = ""
    if DEBUG:
        options = "-l -x"
    command = f"docker run --name robocomp_test -it -w /home/robolab/ --user robolab:robolab -v {INSTALLATION_PATH}/resources/robocomp_install.sh:/home/robolab/robocomp_install.sh robocomp/clean-testing:robocomp-ubuntu{ubuntu_version} bash {options} robocomp_install.sh {branch}"
    return execute_command(command)

def start_interactive_docker_container(ubuntu_version):
    stop_and_remove_container("robocomp_test")
    cprint(f"Starting Ubuntu {ubuntu_version} container.", 'green')
    cprint(f"You can find the robocomp installation script in /home/robolab/robocomp_install.sh", 'green')
    cprint(f"To end this session execute <exit> or use <Ctrl>+d in the current bash terminal.", 'green')
    command = f"docker run --name robocomp_test -it -w /home/robolab/ --user robolab:robolab -v {INSTALLATION_PATH}/resources/robocomp_install.sh:/home/robolab/robocomp_install.sh robocomp/clean-testing:robocomp-ubuntu{ubuntu_version} bash"
    return execute_command(command)


def download_and_initialize_image(version):
    stop_and_remove_container("ubuntu_init")
    options = ""
    if DEBUG:
        options = "-l -x"
    commands = [
        f"docker pull ubuntu:{version}",
        f"docker run --name ubuntu_init -it -v {INSTALLATION_PATH}/resources/robocomp_ubuntu_init.sh:/root/robocomp_ubuntu_init.sh ubuntu:{version} bash {options} /root/robocomp_ubuntu_init.sh",
        f"docker commit ubuntu_init robocomp/clean-testing:robocomp-ubuntu{version}"
    ]
    execute_commands_list(commands)
    stop_and_remove_container("ubuntu_init")

def docker_check_image_exists(version):
    command = "docker images robocomp/clean-testing"
    output = get_command_output(command)
    if f"robocomp/clean-testing   robocomp-ubuntu{version}" in output.decode('utf-8'):
        return True
    else:
        return False


def save_docker_log():
    command = "docker logs robocomp_test"
    command_output = get_command_output(command)
    if command_output != "ERROR":
        date = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        filename = f"/tmp/rcbuildtesting_{date}.txt"
        if DEBUG:
            print(f"Writtig log to {colored(filename, 'green')} ...")
        with open(filename, 'w') as output_file:
            output_file.write(command_output.decode('utf-8'))
            return filename
    else:
        return ""

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='rcbuildtesting')
    parser.add_argument('-b', '--branch', type=str, default='development')
    parser.add_argument('-v', '--version', type=str, default='18.04').completer = ubuntu_images_completer
    parser.add_argument('--manual-mode', action='store_true')
    parser.add_argument('-d', '--debug', action='store_true')
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    if args.debug:
        DEBUG = True
    try:
        cache = apt.cache.Cache()
        pkg = cache[pkg_name]
        if pkg.is_installed:
            cprint(f"{pkg_name} already installed", 'green')
        else:
            output = docker_install()
    except KeyError:
        output = docker_install()

    print(f"UBUNTU VERSION  = {args.version}")
    print(f"ROBOCOMP BRANCH = {args.branch}")

    if not docker_check_image_exists(args.version):
        download_and_initialize_image(args.version)

    if args.manual_mode:
        docker_exit_code = start_interactive_docker_container(args.version)
        last_log_file = save_docker_log()
    else:
        docker_exit_code = start_docker_container(args.version, args.branch)
        last_log_file = save_docker_log()
        if docker_exit_code == 0:
            cprint("BUILT DONE OK", 'green')
        else:
            print(f"Built FAILED with return status {docker_exit_code}. Check the {colored(last_log_file, 'red')} file.")

    remove_container("robocomp_test")


