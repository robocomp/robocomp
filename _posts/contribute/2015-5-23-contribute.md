---
layout: post
title: How to contribute to robocomp github repository
categories: [tutorial]
tags: [Github]
description: Step by step guide to contribute to robocomp github repository
---
#How to contribute to robocomp github repository

In brief the steps involved will be:
* Fork the robocomp repository
* Clone the repository
* Create a new branch
* Switch to the created branch
* Make suitable changes
* Test the changes
* Add the files you want to commit
* Commit the files
* Push to repository
* Compare changes
* Pull a request for the commit made
* Start a discussion over the commits made

In detail explanation: 

1) Forking the repository:

Open the robocomp github repository : [https://github.com/robocomp/robocomp](https://github.com/robocomp/robocomp)

Click on the fork option as shown below:

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/fork.jpg)


The indication that the repo is forked can be observed as follows:

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/forked.jpg)
 

2) Cloning the the repository:

Open a terminal and move to home directory and type: 
	
	git clone <link to the forked repository>

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/cloned.jpg)


3) Creating a branch:

Open a terminal and move the “pwd” to the place where the forked repository is present.

*  To see a list of all branches present and to see the current working branch type and execute:

	`git branch`

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/branch.jpg)


* Create a new branch to work on. There is no restriction wrt the naming of the branch:

	`git branch <branch_name>`

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/branched.jpg)


* To switch to the branch that you created to work on execute:

	`git checkout <branch_name>`

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/switched.jpg)


4) Making changes:

Demo : Lets make changes to *Readme.md*

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/changes.jpg)


5) Commit the changes made

* To get a list of all the files to which changes have been made type:

	`git status`

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/status.jpg)


* Next add only those files which we want to commit by typing:

	`git add <filename>`

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/add.jpg)


*  Now commit the changes by using command:

	`git commit -m 'commit message'`

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/commit.jpg)


6) Push the commit to repository:

	git push origin <current_working_branch_name>

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/push.jpg)


7) Compare and pull request

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/pull.jpg)


8) Discuss the commits you have made with the rest of the contributing team.
