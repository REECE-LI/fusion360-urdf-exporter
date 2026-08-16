# Third-party notices

This repository is a derivative work. The current exporter preserves the
historical plugin directory and manifest identity for Fusion 360 compatibility,
while its implementation and output workflow have been substantially changed.

## Fusion2URDF

- Project: [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf)
- Original author: Toshinori Kitamura (`syuntoku14`)
- License: MIT License
- Original copyright notice: Copyright (c) 2018 Toshinori Kitamura

The original project provides the base Fusion 360 URDF exporter concepts,
including Fusion component/joint extraction, inertial properties, and STL
export. Its MIT license permits modification and redistribution provided that
the copyright and license notice are retained.

## Fusion2URDF ROS 2 fork

- Project: [dheena2k2/fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2)
- Author credited by the upstream license: Dheenadhayalan R (`dheena2k2`)
- License: MIT License
- Original copyright notice: Copyright (c) 2022 Dheenadhayalan R

This fork contributed the ROS 2-era plugin layout and related exporter
organization. The current repository no longer generates the old ROS 2 package,
Xacro, launch, Gazebo, transmission, or ros2_control output.

## ROS 2 project lineage

- Project: [runtimerobotics/fusion360-urdf-ros2](https://github.com/runtimerobotics/fusion360-urdf-ros2)
- License: MIT License as published by that project
- Relationship: the local plugin layout and historical demos are from this
  project lineage; the current branch has been refactored for pure URDF + STL
  export.

## Historical ROS demo files

`demos/rosbot/generated_pkg` and `demos/rosbot/modified_pkg` are retained as
historical examples only. Some files include the Apache License 2.0 notices
from the Open Robotics ROS package templates. Those notices apply to the
corresponding demo files and do not change the MIT license of the exporter
code. The current plugin does not import or execute those demo packages.

## Autodesk Fusion 360

The plugin uses the Autodesk Fusion 360 public API (`adsk.core`, `adsk.fusion`)
and must be run inside Fusion 360. Fusion 360 is proprietary software and is
not bundled with this repository. Autodesk trademarks and product rights
remain with Autodesk.
