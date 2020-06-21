## Robocomp with FCL (The Flexible Collision Library) support

- If you have any previous installation of fcl, make sure you uninstall it.
  To avoid problems with compilation, check that there are no fcl files in
  ```
  ls /usr/local/lib/libfcl.*
  ```
  nor in
  ```
  ls /usr/local/include/fcl
  ```

- Install libfcl-dev from the Ubuntu repository

```bash
sudo apt-get update -y
sudo apt-get install -y libfcl-dev
```

- Change Robocomp's cmake configuration

```bash
cd ~/robocomp/build
cmake-gui ..
```

- Select checkbox FCL_SUPPORT

```bash
push configure button
push generate button
exit

make
sudo make install
```
