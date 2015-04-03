########## How to contribute to robocomp github repository #############

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
* Start a discussion regarding you commit

Now each and every step will be explained in detail:

1) Forking the repository:

Open the robocomp github repository : [https://github.com/robocomp/robocomp](https://github.com/robocomp/robocomp)

Click on the fork option as shown below:

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/fork.jpg)

The indication that the repo is forked can be observed as follows:

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/forked.jpg)

 
2) Cloning the the repository:

Open a terminal and move to home directory and type: git clone <link to the forked repository>

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/cloned.jpg)


3) Creating a branch:

Open a terminal and move the “pwd” to the place where the forked repository is present.

* First command is “git branch -a”, it provides a list of all branches present and indicates the current working branch

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/branch.jpg)

* To Create a new branch type “git branch <branch_name>”. There is no restriction to the <branch_name>, but it is desirable to be related to the changes being made.

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/branched.jpg)

* Next step is to switch to that branch, so type “git checkout <branch_name>”

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/switched.jpg)


4) Making changes:

Demo : Lets make changes Readme.md

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/changes.jpg)


5) Commit the changes made

* First type “ git status”, this command lists all the files to which changes have been made

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/status.jpg)

* Next step is to add only those files which we want to commit by typing “git add <filename>”

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/add.jpg)

* Now its time to commit the changes, so use command “ git commit -m '<heading>' "

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/commit.jpg)


6) Push the commit to repository

Use the command “git push origin <current_working_branch_name>”

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/push.jpg)


7) Compare and pull request

![](https://github.com/abhi-kumar/robocomp/blob/how-to-contribute/pull.jpg)


8) Now its time to discuss over the pull-request in detail with the rest of the contributing team


!!!!!CHEERS!!!!!!! 
