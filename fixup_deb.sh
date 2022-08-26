#!/bin/sh
set -e
mkdir fix_up_deb
dpkg-deb -x robocomp-2021.02.01-ubuntu.deb fix_up_deb
dpkg-deb --control robocomp-2021.02.01-ubuntu.deb fix_up_deb/DEBIAN
rm robocomp-2021.02.01-ubuntu.deb
chmod 0644 fix_up_deb/DEBIAN/md5sums
find -type d -print0 |xargs -0 chmod 755
fakeroot dpkg -b fix_up_deb robocomp-2021.02.01-ubuntu.deb
rm -rf fix_up_deb
