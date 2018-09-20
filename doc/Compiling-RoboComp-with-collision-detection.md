## Robocomp with FCL (The Flexible Collision Library) support

- Install libfcl-dev from the Ubuntu repository

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
