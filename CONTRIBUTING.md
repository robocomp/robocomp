# Robocomp Community Contributing Guide 1.0

**First things first:** if you're unsure of _anything_, just ask or submit the issue or pull request anyways. Nobody will be angry with you. The worst that can happen is that you'll be politely asked to change something. We appreciate any sort of contributions, and don't want a wall of rules to get in the way of that.

However, for those individuals who want a bit more guidance on the best way to contribute to the project, read on. This document will cover what you are looking for. By addressing all these points, it raises the chances we can quickly address your contributions.

__Important__: The default branch of Robocomp is `development`. This means that new features or bug reports 
for this branch are our priority.

## Issues

### Reporting an Issue

- Make sure you test against the latest version (of the highlyunstable branch). It is possible we 
already fixed the bug you're experiencing.

- Provide a reproducible test case, explaining your bug as thoroughly as possible. For example, screenshots of  your
problem or the output your error provided.
If a contributor can't reproduce an issue, then it dramatically 
lowers the chances it'll get fixed. And in some cases, the issue will eventually be closed.

- Respond promptly to any questions made to your issue. Stale issues will be closed.

### Opening a Pull Request

Thank you for contributing! When you are ready to open a pull request, you will
need to [fork
Robocomp](https://github.com/robocomp/robocomp#fork-destination-box), push your 
changes to your fork, and then open a pull request. 

*Tell us* what your commit does, if the title is not self-explanatory. 
For example, "Fixed a typo" is ok, but "Added Docker for testing" might need a short summary explaining how to use it, and why.

If you need to sync commits from `robocomp/robocomp` to push your own changes, follow
[this](https://gist.github.com/CristinaSolana/1885435) tutorial.

## Tips for working on Robocomp

- If you make a change to the dependencies necessary for using Robocomp, 
be sure to update the `Readme.md` installation tutorial to include your changes.

- If you plan to make big changes, ensure you are not breaking Robocomp's core functionalities.
Even through Travis tests the build, you could break an internal function. 
Please ensure you test new features locally at first.

### Code of Conduct

All contributors tacitly agree to abide by both the letter and spirit of the [Code of Conduct](CODE_OF_CONDUCT.md). 
Failure, or unwillingness, to do so will result in contributions being respectfully declined.

