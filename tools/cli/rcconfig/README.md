rcconfig
==

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This tools is used to define and store "global" variables available for all the python components and other tools in Robocomp.  

Currently it's only used during installation process to set the `ROBOCOMP_SRC` variable defined in the `RC_CONFIG` dictionary when you import rcconfig package.
This variable point to the path of the source code of robocomp.

```shell
$ rcconfig --help                                                                                   
Usage: rcconfig [OPTIONS] COMMAND [ARGS]...

  Config robocomp variables system wide.

Options:
  --install-completion [bash|zsh|fish|powershell|pwsh]
                                  Install completion for the specified shell.
  --show-completion [bash|zsh|fish|powershell|pwsh]
                                  Show completion for the specified shell, to
                                  copy it or customize the installation.

  --help                          Show this message and exit.

Commands:
  location
  print
  set-src-dir

```

### Command examples
Change the value for the current `ROBOCOMP_SRC` entry in `RC_CONFIG`:
```shell
rcconfig set-src-dir <path>
```

You can check the content of the current `RC_CONFIG` dictionary with
```bash
rcconfig print
```

Or get the value of an specific entry with
```bash
rcconfig print ROBOCOMP_SRC
```