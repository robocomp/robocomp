import os
import sys
from distutils import spawn
from cogapp import Cog

sys.path.append("/opt/robocomp/python")

DIFF_TOOLS = ["meld", "kdiff3", "diff"]


def get_diff_tool(prefered=None):
    if prefered in DIFF_TOOLS:
        tool_path = spawn.find_executable(prefered)
        if tool_path is not "":
            return prefered, tool_path
    for tool in DIFF_TOOLS:
        tool_path = spawn.find_executable(tool)
        if tool_path is not "":
            return tool, tool_path
    return None, None


def replaceTagsInFile(path):
    i = open(path, 'r', encoding='utf-8', errors='ignore')
    text = i.read()
    reps = []
    reps.append(["\n<@@<" ,""])
    reps.append([">@@>\n" ,""])
    reps.append(["<TABHERE>", '\t'])
    reps.append(["<S1>", ' '])
    reps.append(["<S2>", '  '])
    reps.append(["<S4>", '    '])
    for r in reps:
        text = text.replace(r[0], r[1])
    i.close()
    w = open(path, 'w')
    w.write(text)
    w.close()


def generate_cog_command(params, template, output_file):
    params_strings = ["-D %s=%s" % (key, value) for key, value in params.items()]
    return "cog -z -d %s -o %s %s" % (" ".join(params_strings), output_file, template)


def run_cog_and_replace_tags(cog_command, ofile):
    run = cog_command.split(' ')
    ret = Cog().main(run)
    if ret != 0:
        raise RuntimeError('ERROR (%d) executing cog %s' % (ret, cog_command))
    replaceTagsInFile(ofile)


def create_directory(directory):
    try:
        print('Creating', directory,)
        os.mkdir(directory)
        print('')
    except:
        if os.path.isdir(directory):
            print('(already existed)')
            pass
        else:
            raise RuntimeError('\nCOULDN\'T CREATE %s' % directory)