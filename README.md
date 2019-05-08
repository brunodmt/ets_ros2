ets_msgs: Messages definitions

ets_plugin: ETS/ATS plugin library

ets_cpp_client: Sample C++ client


1. Install ROS2 following https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/
2. Install ETS or ATS (for example, using Steam)
3. Create a ros2 worskpace: mkdir -p ros2_ws/src
4. Clone the repo inside ros2_ws/src
5. Source the ros2 environment setup script `source /opt/ros/crytal/setup.zsh
6. From ros2_ws, build the project `colcon build --symlink-install`
If you happen to get this error:
```
--- stderr: ets_msgs                         
CMake Error at /usr/share/cmake-3.10/Modules/FindPackageHandleStandardArgs.cmake:137 (message):
  Could NOT find FastRTPS (missing: FastRTPS_INCLUDE_DIR FastRTPS_LIBRARIES)
```
run `export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH`
and try to build again
7. Copy the generated plugin library to the ETS folder: `cp install/ets_plugin/lib/ets_plugin/libetsros2.so ~/.local/share/Steam/steamapps/common/Euro\ Truck\ Simulator\ 2/bin/linux_x64/plugins`
8. Run the client with `./install/ets_cpp_client/bin/ets_cpp_client/ets_cpp_client`
9. The first time you are going to run the game you need to:
  1. Right click the game
  2. Select properties
  3. Select SET LAUNCH OPTIONS...
  4. Set it to `LD_LIBRARY_PATH=/opt/ros/crystal/opt/yaml_cpp_vendor/lib:/opt/ros/crystal/opt/rviz_ogre_vendor/lib:/opt/ros/crystal/lib:$LD_LIBRARY_PATH %command%`
10. Run the game

In case the game crashes, you may need to run:
`sudo ldconfig /opt/ros/crystal/lib`


If you want to try `ros2 topic echo /truck`, you need to source the environment from the workspace to have the messages available:
```source install/setup.zsh
ros2 topic echo /truck```