# Hardware Interface for Ignition Gazebo and ROS2 Control

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=humble)](../../actions/workflows/build_and_test_humble.yaml?query=branch:humble)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_iron.yaml?query=branch:rolling)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

See [ijnek/gz_control_hw_demo](https://github.com/ijnek/gz_control_hw_demo) for how to use this package.

# Supported Combinations

ROS 2 version | Gazebo version | Branch
-- | -- | --
Humble | Fortress (default) | [humble](https://github.com/ijnek/gz_control_hw/tree/humble)
Humble | Garden | [humble](https://github.com/ijnek/gz_control_hw/tree/humble)
Iron | Fortress (default) | [rolling](https://github.com/ijnek/gz_control_hw/tree/rolling)
Iron | Garden | [rolling](https://github.com/ijnek/gz_control_hw/tree/rolling)
Rolling | Edifice | [rolling](https://github.com/ijnek/gz_control_hw/tree/rolling)
Rolling | Fortress (default) | [rolling](https://github.com/ijnek/gz_control_hw/tree/rolling)
Rolling | Garden | [rolling](https://github.com/ijnek/gz_control_hw/tree/rolling)

## Source Installation

Install a Gazebo version that is supported on your platform from the *Supported Combinations* table above.

**If you have selected a Gazebo version that is not the default version for your distro**, then set the `GZ_VERSION` environment variable to the Gazebo version you'd like to compile against. For example:

    export GZ_VERSION=edifice

**If you're using a version of Gazebo that isn't the default for your ROS distribution, you must [install ``ros_gz`` from source](https://github.com/gazebosim/ros_gz/tree/galactic) too.**

> You only need to set this variable when compiling, not when running.

#### Compile ros_ign

The following steps are for Linux and OSX.

1. Create a colcon workspace:

    ```sh
    # Setup the workspace
    mkdir -p ~/ws/src
    cd ~/ws/src

    # Source ROS distro's setup.bash (replace rolling with your distro)
    source /opt/ros/rolling/setup.bash

    # Clone repoitory (**select one** from below, depending on your ROS 2 distro)
    git clone https://github.com/ijnek/gz_control_hw.git -b humble  # Humble
    git clone https://github.com/ijnek/gz_control_hw.git -b rolling  # Rolling
    ```

1. Install dependencies (this may also install Ignition):

    ```sh
    cd ~/ws
    rosdep install --from-paths src --ignore-src --default-yes
    ```

    > If `rosdep` fails to install Ignition libraries and you have not installed them before, please follow [Ignition installation instructions](https://ignitionrobotics.org/docs/latest/install).

1. Build the workspace:

    ```sh
    # Build workspace
    cd ~/ws
    colcon build
    ```

1. Source the workspace:

    ```sh
    # In any new terminal you want to use gz_control_hw, first run
    source ~/ws/src/local_setup.bash
    ```
