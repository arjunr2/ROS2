## ROS2 WALI Build

To build, after setting appropriate toolchain file from WALI repo, run

```shell
./build_full.sh
```

To clean the build, run

```shell
sudo ./clean.sh
```

**NOTE**: There is a weird CMakeRelink error for idlc in cyclone-dds. After running `build.sh`, it will error out.
`build_full.sh` runs `idlc` to the error directory and rebuilds without cleaning, allowing it to build to completion.
