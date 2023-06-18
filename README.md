# SeekCamera Wrapper

Install seekthermal-sdk-dev-4.3.1.6_amd64.deb

    sudo apt-get install ./seekthermal-sdk-dev-4.3.1.6_amd64.deb

Install python wrapper

    sudo pip3 install -e .

Build ROS package

    ros2 run seek_thermal_ros thermal_publisher

Change colorPalette using args: **--ros-args -p colorPalette:=value ** replace value with options mentioned below!

    Available options:
        - WHITE_HOT
        - BLACK_HOT
        - SPECTRA
        - PRISM
        - TYRIAN -- Default
        - IRON
        - AMBER
        - HI
        - GREEN

Change rotationValue using args: **--ros-args -p rotationValue:=value ** replace value with options mentioned below!

    Available options:
        - 0
        - 90 -- Default
        - 180
        - 270
        - 360

Sample image

![Sample RVIZ2](extras/images/sample_rviz2.png?raw=true "Sample RVIZ2")