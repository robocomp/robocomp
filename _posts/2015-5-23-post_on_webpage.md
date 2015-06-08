---
layout: post
title: Write a post for robocomp, A step by step guide.
categories: [Tutorial]
tags: [General]
description: In detail explanation as to how to write a post for robocomp
---

#Write a post for robocomp, A step by step guide.


In this tutorial you will be learning about writing a post for robocomp. I assume that you are already familiar with contributing via Github if you are not then you can follow [this article](http://rajathkumarmp.github.io/robocomp/tutorial/2015/05/23/contribute.html) and then come back here.

By now you would have already forked the repository.
Now switch to `gh-pages` branch
You can do this via github client or on the command line by navigating to the directory and executing the command

    `git checkout gh-pages`

After checking out to the github pages branch in your navigate to the `_posts` directory. Here you will find all the posts.

To write a new post. Create a new file and save it as `XYZ.md`

Note that you will be using Github markdown language.

Once you save the file as `XYZ.md`. It will be saved as draft and not published on to the website.

At the header of every article/post you write. Always add this

    ---
    layout:
    title: 
    categories:
    tags:
    description:
    ---


Layout can be `post`, `page` or `default`. Always set the layout as `post`. The title is the title of the post. Categories and Tags should be set accoridingly whichever is applicable. This is helpful in navigating or finding posts on same topic. Description is a short explanation or gist of the entire post.

A sample header looks like this.

    ---
    layout: post
    title: Write a post for robocomp, A step by step guide.
    categories: [Tutorial]
    tags: [General]
    description: In detail explanation as to how to write  a post for robocomp
    ---

After adding the header you can proceed writing the post by using Github Markdown language. Now for the most important step. To publish the post or to change the post from draft to final you will have to rename the file to

    `YYYY-MM-DD-XYZ.md`

That's it and you would successfully published a post on to the robocomp's website.
