#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import os
import subprocess
import sys
import traceback
from shutil import rmtree

from termcolor import cprint, colored


class MyParser(argparse.ArgumentParser):
    """
    Convenience class for the ArgParse parser.
    """

    def error(self, message):
        cprint('error: %s' % message, 'red')
        self.print_help()
        sys.exit(2)


class ComponentGenerationChecker:
    """
    Class containing methods used to generate code and compile a bunch of components to be tested.
    """

    def __init__(self):
        self.results = {}
        self.valid = 0
        self.generated = 0
        self.compared = 0
        self.comp_failed = 0
        self.gen_failed = 0
        self.dry_run = False

    def generate_code(self, idsl_file, output_dir, dry_run=True):
        """
        Execution of the robocompdsl script to generate the component code for a gigen .idsl file.
        :param idsl_file: filename of the .idsl of the component to be generated
        :param log_file: name of the log file to be written with the result of the command execution.
        :param dry_run: Force to just show messages of what would be done but no command is executed.
        :return: command execution return code
        """
        ice_file = os.path.join(output_dir,os.path.splitext(os.path.basename(idsl_file))[0]+".ice")
        log_file = os.path.join(output_dir,os.path.splitext(os.path.basename(idsl_file))[0]+"_generation.log")
        if dry_run:
            print('robocompdsl %s %s > %s 2>&1' % (idsl_file, ice_file, log_file))
            return 0
        else:
            with open(log_file, "wb") as log:
                command_output = subprocess.Popen(
                    "python " + os.path.expanduser("~/robocomp/tools/robocompdsl/robocompdsl.py") + " %s %s" % (idsl_file, ice_file),
                    stdout=log,
                    stderr=log,
                    shell=True)
                stdout, stderr = command_output.communicate()
                # print(stdout)
                # print(stderr)
                return command_output.returncode, ice_file
            return -1, None

    def compare_ice(self, first_file, second_file, output_dir):
        """
        Execute cmake on the curren directory . Output and error is saved to make_output.log.
        :return: command return code
        """
        second_file_name = os.path.splitext(os.path.basename(second_file))[0]
        log_file = os.path.join(output_dir, second_file_name+"_comparison.log")
        with open(log_file, "wb") as log:
            command_output = subprocess.Popen("grep -Fxv -f %s %s" % (first_file, second_file),
                                              stdout=log,
                                              stderr=log,
                                              shell=True)
            stdout, stderr = command_output.communicate()
            # print(stdout)
            # print(stderr)
        log_lines = -1
        with open(log_file, "r") as log:
            for log_lines, l in enumerate(log):
                pass
        return command_output.returncode, log_lines+1


    def remove_genetared_files(self, current_dir, dry_run=True):
        """
        Remove all files but .smdsl, .cdsl and .log from the given dir.
        :param current_dir: Directory to look for files to remove.
        :param dry_run: Just show the output. No rm is really executed.
        :return: None
        """
        if os.path.isdir(current_dir):
            component_files = os.listdir(current_dir)
            # Remove not useful files
            remove_dir = True
            for next_file in component_files:
                if next_file.endswith(".ice") or next_file.endswith(".log"):
                    if dry_run:
                        print("rm -r %s" % next_file)
                    else:
                        full_path_ice = os.path.join(current_dir, next_file)
                        if os.path.isfile(full_path_ice):
                            os.remove(full_path_ice)
                        else:
                            rmtree(full_path_ice)
                else:
                    remove_dir = False
            if remove_dir:
                if dry_run:
                    print("rm -r %s" % current_dir)
                else:
                    rmtree(current_dir)

    def check_components_generation(self, idsls_dir, dry_run, dirty,  output_dir=".", generate_only=False, filter="",visual=False):
        """
        Main method of the class. Generate needed code, compile and show the results
        :param idsls_dir: alternative dir for the idsl files
        :param dry_run: just show what would be done. No file is removed.
        :param dirty: Leave all the generated files. No clean is done at the end of the script.
        :param generate_only: No compilation is executed for the components.
        :param filter: A string can be given to filter the directory of the components to be generated/compiled.
        :param clean_only: Just clean the generated files.
        :return: None
        """
        output_dir = os.path.join(output_dir, "auto_generated_ice_files")
        if not os.path.isdir(output_dir):
            os.mkdir(output_dir)
        self.dry_run = dry_run
        previous_dir = os.getcwd()
        list_dir = os.listdir(idsls_dir)
        for file in list_dir:
            if file.endswith(".idsl") and filter in file:
                idsl_file = file
                idsl_path = os.path.join(idsls_dir,idsl_file)
                self.valid += 1
                self.results[idsl_path] = {'generation': False, 'comparation': False }
                code, ice_path = self.generate_code(idsl_path, output_dir, self.dry_run)
                if code == 0:
                    cprint("%s generation OK" % ice_path, 'green')
                    self.results[idsl_path]['new_ice_path'] = ice_path
                    self.results[idsl_path]['generation'] = True
                    self.generated += 1
                    if generate_only:
                        self.results[idsl_path]['comparation'] = False
                        print("%s not compared (-g option)" % file)
                        self.comp_failed +=1
                    else:
                        print("Executing comparation for %s ... WAIT!" % (ice_path))
                        original_ice_file = os.path.join(idsls_dir, "..", os.path.basename(ice_path))
                        self.results[idsl_path]['original_ice_path'] = original_ice_file
                        comparison_result, diff_lines = self.compare_ice(original_ice_file, ice_path, output_dir)
                        self.results[idsl_path]['diff_lines'] = diff_lines
                        if (comparison_result == 0 and diff_lines ==0) or self.dry_run:
                            self.results[idsl_path]['comparation'] = True
                            self.compared += 1
                            cprint("%s comparation OK" % idsl_path, 'green')
                        else:
                            self.results[idsl_path]['comparation'] = False
                            self.comp_failed += 1
                            cprint("%s comparation FAILED" % idsl_path, 'red')


                else:
                    cprint("%s generation FAILED"%idsl_path, 'red')
                    self.gen_failed += 1
                    self.results[idsl_path]['generation'] = False

        # Command final output
        print("%d idsl files found" % self.valid)
        print("%d ice files generated OK (%d failed)" % (self.generated, self.gen_failed))
        print("%d are identical (%d are different)" % (self.compared, self.comp_failed))

        for file, result in self.results.items():
            cname = colored(file, 'magenta')

            # Printing results for generation
            if result['generation']:
                gen_result = colored("TRUE", 'green')
            else:
                gen_result = colored("FALSE", 'red')

            # Printing results for compilation
            if result['comparation']:
                comp_result = colored("TRUE", 'green')
            else:
                comp_result = colored("FALSE", 'red')

            if generate_only:
                print("\t%s have been generated? %s" % (cname, gen_result))
            else:
                print("\t%s have been generated? %s Are equal? %s %s different lines" % (cname, gen_result, comp_result, result['diff_lines']))
            if visual and not result['comparation'] and result['generation']:
                with open("meld.log", "wb") as log:
                    command_output = subprocess.Popen("meld %s %s"%(result['original_ice_path'], result['new_ice_path']),
                                                      stdout=log,
                                                      stderr=log,
                                                      shell=True)

                    stdout, stderr = command_output.communicate()

        if not dirty:
            self.remove_genetared_files(output_dir, self.dry_run)
        print("")
        os.chdir(previous_dir)


if __name__ == '__main__':
    parser = MyParser(description=colored(
        'This application generate components from cdsl files to check component generation/compilation\n', 'magenta'),
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('indir', type=str, help='Input dir for idsl files')
    parser.add_argument("-g", "--generate-only", action="store_true",
                        help="Only the generation with robocompdsl is checked. No cmake or make is tested")
    parser.add_argument("-d", "--dirty",
                        help="No cleaning is done after execution. All source files and temp will be left on their dirs.",
                        action="store_true")
    # parser.add_argument("-c", "--clean",
    #                     help="Just clean all source files and temp will be left on their dirs. .cdsl, .smdsl and .logs are kept on their dirs.",
    #                     action="store_true")
    parser.add_argument("-n", "--dry-run",
                        help="Executing dry run. No remove and make will be executed in dry run mode. CMake is executed and some files generated.",
                        action="store_true")
    parser.add_argument("-i", "--installation", type=str,
                        help="Installation directory where robocompdsl.py can be found.",
                        default="~/robocomp/tools/robocompdsl/component_generation_test")

    parser.add_argument("-f", "--filter", type=str,
                        help="Execute the check only for directories containing this string.", default="")
    parser.add_argument("-v", "--visual",
                        help="open meld application con visually compare old and new ice files",
                        action="store_true")
    args = parser.parse_args()

    try:
        checker = ComponentGenerationChecker()
        checker.check_components_generation(args.indir, args.dry_run, args.dirty, ".", args.generate_only,
                                            args.filter, args.visual)
    except (KeyboardInterrupt, SystemExit):
        cprint("\nExiting in the middle of the execution.", 'red')
        cprint("Some files will be left on the directories.", 'yellow')
        cprint("Use -c option to clean all the generated files.", 'yellow')
        sys.exit()
    except Exception as e:
        cprint("Unexpected exception: %s"%e.message, 'red')
        traceback.print_exc()

