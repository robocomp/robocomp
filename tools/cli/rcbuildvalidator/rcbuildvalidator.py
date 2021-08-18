import apt
import argcomplete
from termcolor import cprint, colored

import cli

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='rcbuildvalidator makes easy to test the installation and build of the Robocomp core in many different Ubuntu versions.')
    parser.add_argument('-b', '--branch', type=str, default='development', help="Set the Robocomp branch to be tested")
    parser.add_argument('-v', '--version', type=str, default='18.04', help="Set the ubuntu version to be used").completer = ubuntu_images_completer
    parser.add_argument('--manual-mode', action='store_true', help="Open an interactive terminal of ubuntu with the version set by -v")
    parser.add_argument('-m', '--mount', nargs='*', type=str, action='append', help="Accept a list of paths to be mountes in the home directory of the launched machine").completer = DirectoriesCompleter()
    parser.add_argument('-d', '--debug', action='store_true', help="Shows some debugging messages")

    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    if args.debug:
        DEBUG = True

    print(f"UBUNTU VERSION  = {args.version}")
    print(f"ROBOCOMP BRANCH = {args.branch}")

    if not cli.docker.check_robocomp_image_exists(args.version):
        cli.docker.build_robocomp_dependencies_image(args.version)

    if args.manual_mode:
        mount_paths = []
        if args.mount:
            mount_paths = [y for x in args.mount for y in x] # flatten list
        docker_exit_code = cli.docker.start_interactive_docker_container(args.version, mount_paths)
        last_log_file = cli.docker.save_robocomp_build_log()
    else:
        docker_exit_code = cli.docker.robocomp_installation_check(args.version, args.branch)
        last_log_file = cli.docker.save_robocomp_build_log()
        if docker_exit_code == 0:
            cprint("BUILT DONE OK", 'green')
        else:
            print(f"Built FAILED with return status {docker_exit_code}. Check the {colored(last_log_file, 'red')} file.")

    cli.docker.remove_container("robocomp_test")


