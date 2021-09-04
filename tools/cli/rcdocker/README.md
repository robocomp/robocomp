rcdocker
==

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This is a docker wrapper for robocomp. It's intended to be an easier way to generate Robocomp images and to be able to use them
for testing porposes. <link to docker page of robocomp>

```shell
$ rcdocker --help                                                                                   
Usage: rcdocker [OPTIONS] COMMAND [ARGS]...

  Docker wrapper for Robocomp images and commands.

Options:
  --install-completion [bash|zsh|fish|powershell|pwsh]
                                  Install completion for the specified shell.
  --show-completion [bash|zsh|fish|powershell|pwsh]
                                  Show completion for the specified shell, to
                                  copy it or customize the installation.

  --help                          Show this message and exit.

Commands:
  build
  build-comp
  check-robocomp-install
  install-docker
  interactive
```