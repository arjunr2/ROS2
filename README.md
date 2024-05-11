## ROS2 WALI Build

To build, after setting appropriate toolchain file from WALI repo, run

```shell
./build.sh
```

**NOTE**: There is a weird CMakeRelink error for idlc in cyclone-dds. After running `build.sh`, it will error out.
Copy `idlc` to the error directory and rebuild without cleaning --- it will proceed to build completely.
To copy idlc, you can just run the `idlc_cp.sh` script
