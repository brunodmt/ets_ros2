# Euro Truck Simulator 2 ROS2 Plugin
[ETS2 (Euro Truck Simulator 2)](https://eurotrucksimulator2.com/) & [ATS (American Truck Simulator)](https://americantrucksimulator.com/) SDK plug-in to publish telemetry data using [ROS2](https://index.ros.org/doc/ros2/).

## Project Structure

* **ets_msgs**: Messages definitions
* **ets_plugin**: ETS/ATS plugin library
* **ets_cpp_client**: Sample C++ client

## Supported Environment
This plugin has been developed in Ubuntu 18.04 using ROS2 Crystal and Euro Truck Simulator 2 (v1.34.0.34s). Instructions may vary when using different versions, but with some luck it should work anyway.

Windows is not currently supported.

## Prerequisites
* Install **ROS2** following [Installing ROS2 via Debian Packages](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/)
* Install **colcon** following [Colcon Tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
* Install **ETS** or **ATS** using **Steam**

## Building instructions
1. Create a ROS2 worskpace in a location of your choice: `mkdir -p ros2_ws/src`
2. Clone the repository inside `ros2_ws/src`: `git clone https://github.com/brunodmt/ets_ros2.git`
3. Source the ROS2 environment setup script, selecting the extension based on your shell: `source /opt/ros/crystal/setup.<shell>`.
4. From ros2_ws, build the project: `colcon build --symlink-install`.
   In case you get an error like:
   ```
   --- stderr: ets_msgs                         
   CMake Error at /usr/share/cmake-3.10/Modules/FindPackageHandleStandardArgs.cmake:137 (message):
     Could NOT find FastRTPS (missing: FastRTPS_INCLUDE_DIR FastRTPS_LIBRARIES)
   ```
   run `export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH` and try again.
5. Copy the generated plugin library to the ETS folder:
   ```
   mkdir  ~/.local/share/Steam/steamapps/common/Euro\ Truck\ Simulator\ 2/bin/linux_x64/plugins
   cp install/ets_plugin/lib/ets_plugin/libetsros2.so ~/.local/share/Steam/steamapps/common/Euro\ Truck\ Simulator\ 2/bin/linux_x64/plugins/
   ```
   or the ATS folder:
   ```
   mkdir ~/.local/share/Steam/steamapps/common/American\ Truck\ Simulator/bin/linux_x64/plugins
   cp install/ets_plugin/lib/ets_plugin/libetsros2.so ~/.local/share/Steam/steamapps/common/American\ Truck\ Simulator/bin/linux_x64/plugins/
   ```
6. Source the generated environment setup script, selecting the extension based on your shell: `source install/setup.<shell>`
7. Run the client with `./install/ets_cpp_client/bin/ets_cpp_client/ets_cpp_client`
8. Before running the game for the first time, the **Launch Options** in **Steam** need to be updated:
   1. Right click the game in your Steam library
   2. Select properties
   3. Select `SET LAUNCH OPTIONS...`
   4. Set it to:
      ```
      LD_LIBRARY_PATH=/opt/ros/crystal/opt/yaml_cpp_vendor/lib:/opt/ros/crystal/opt/rviz_ogre_vendor/lib:/opt/ros/crystal/lib:<ros2-workspace>/install/ets_msgs/lib:$LD_LIBRARY_PATH %command%
      ```
      (making sure to replace `<ros2-workspace>` with your ROS2 workspace path.
9. Run the game
