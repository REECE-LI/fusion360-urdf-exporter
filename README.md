# Fusion 360 URDF Exporter

This Fusion 360 script exports the active robot assembly as one portable URDF
file plus one binary STL mesh per link.

This repository is a maintained derivative of the open-source Fusion URDF
exporter family. It keeps the historical Fusion script directory name
`Fusion_URDF_Exporter_ROS2` for compatibility, but the current exporter has
been simplified to produce URDF + STL only. See
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) for upstream projects,
copyright attribution, and license notes.

## Output

Choosing an output directory always creates:

```text
robot/
├── robot.urdf
└── meshes/
    ├── base_link.stl
    ├── left_link_1.stl
    ├── left_link_2.stl
    └── ...
```

Each top-level Fusion occurrence becomes one URDF link and one STL. If a
top-level occurrence is a subassembly, all of its nested bodies are combined
into that STL; nested parts and internal joints are not exported separately.
The URDF uses relative mesh paths such as `meshes/base_link.stl`, so the robot
folder can be moved without rewriting paths. Visual and collision geometry use
the same high-refinement binary STL. Meshes retain the existing millimetre
export convention and use `scale="0.001 0.001 0.001"` in URDF.

This version does not generate Xacro, ROS 2 packages, launch files, Gazebo
configuration, transmissions, controllers, or sensors.

## Fusion model requirements

- Put every link that should appear in URDF at the top level of the Fusion
  root component. A top-level link may itself contain nested components.
- Naming a top-level occurrence `base_link` is optional. Names such as
  `base_link v4:1` are also recognized. If no such name exists, the exporter
  uses the unique link without a parent joint.
- Add joints between top-level links under the root component.
- When creating a Fusion joint, select the child as Component 1 and the parent
  as Component 2.
- Supported joint types are Rigid, Revolute, and Slider.
- A Revolute joint with no limits becomes a URDF `continuous` joint.
- Revolute and Slider limits must either have both bounds enabled or neither
  bound enabled.
- URDF link names and STL filenames follow the Fusion top-level occurrence
  names. A trailing Fusion suffix such as ` v4:1` is removed. Chinese names are
  preserved; spaces, slashes, colons, and parentheses become underscores.
  Names that become duplicates after removing the version suffix receive
  `_2`, `_3`, and so on.
- Joint names continue to use `joint_1`, `joint_2`, etc. Fusion joint names,
  including names such as `旋转` and `刚性`, are retained in URDF comments.
- Disconnected assemblies, ambiguous roots, incorrectly directed links, and
  unsupported joint types are rejected before output begins.

## Closed-chain mechanisms

URDF requires a tree, while Fusion can contain closed kinematic chains. The
exporter writes the deterministic N−1 spanning-tree joints as standard
`<joint>` elements. Remaining loop-closing joints are placed at the end of the
same `robot.urdf` as custom `<loop_joint>` elements.

Before building the tree, the exporter prioritizes a redundant joint connected
to `base_link` as the loop-closing edge. It removes that joint only when every
link remains reachable from `base_link`, so the actual bridge from the base
into the mechanism is retained. If several removable base-connected joints
exist, the last one in Fusion joint order is preferred. Other redundant edges
fall back to the normal Fusion-order spanning-tree rule.

For a normal Fusion Joint, the physical joint point is read from
`geometryTwoTransform` (with `geometryOneTransform` as a fallback). As-Built
Joints use their joint transform, and older Fusion versions fall back to the
joint geometry or Joint Origin transform. The exporter does not derive the
joint point from the relative component transform.

Each `<loop_joint>` stores the same physical joint point in both link-local
coordinate systems:

```xml
<loop_joint name="joint_10" type="revolute">
  <parent link="base_link"/>
  <child link="left_link_3"/>
  <parent_origin xyz="-0.072803 0.0 0.028171"/>
  <child_origin xyz="..."/>
  <axis xyz="..." frame="parent"/>
</loop_joint>
```

`parent_origin` is `R_WPᵀ (p_WJ - t_WP)` and `child_origin` is
`R_WCᵀ (p_WJ - t_WC)`, converted from Fusion centimetres to metres. The axis
is expressed in the parent link frame. `<loop_joint>` is custom constraint
data, not part of the URDF specification. A MuJoCo-side
preprocessor or importer can inspect it without requiring a second file.

Fusion uses centimetres internally. The exporter converts positions to metres,
inertia from kg/cm² to kg/m², and obtains mass and centre-of-mass values from
Fusion's high-accuracy physical-properties calculation.

## Installation

### Requirements

- Autodesk Fusion 360 with the Fusion 360 API Scripts and Add-Ins feature.
- Windows or macOS. The exporter does not require ROS 2, Python packages, or
  Gazebo to run.

### Install from a GitHub ZIP

1. On GitHub, choose **Code → Download ZIP** and extract the archive.
2. Open Fusion 360 and press **Shift+S** (or open **Utilities → Scripts and
   Add-Ins**).
3. Click the green **+** button and choose **Script or add-in from device**.
4. Select the extracted `Fusion_URDF_Exporter_ROS2` directory itself. Do not
   select the repository root or an individual `.py` file.
5. Select `Fusion_URDF_Exporter_ROS2` under **My Scripts** and click **Run**.

The script can be run directly from the Scripts and Add-Ins dialog; no system
Python installation or `pip install` step is needed.

### Windows

From the extracted repository directory, double-click
`InstallURDFExporter.bat` (or run `InstallURDFExporter.ps1` in PowerShell).
The installer copies the plugin into Fusion's per-user Scripts directory:

```text
%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\Fusion_URDF_Exporter_ROS2
```

Then open Fusion 360 and find `Fusion_URDF_Exporter_ROS2` under
**Utilities → Scripts and Add-Ins → My Scripts**.

### macOS manual installation

Run the following commands from the extracted repository directory:

```bash
mkdir -p "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts"
cp -R Fusion_URDF_Exporter_ROS2 \
  "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

Restart or reopen the Scripts and Add-Ins dialog in Fusion 360. The directory
keeps its historical name so existing Fusion installations and scripts remain
compatible.

## Exporting

1. Open the Fusion robot design.
2. Run `Fusion_URDF_Exporter_ROS2`.
3. Confirm the welcome message.
4. Choose the parent output directory.
5. Wait for the success dialog showing the generated URDF and mesh directory.

The exporter reads the design without copying, renaming, or deleting Fusion
components. Re-exporting replaces `robot.urdf` and rebuilds `robot/meshes/` so
stale meshes from an earlier export are removed.

## Historical demos

`demos/rosbot/generated_pkg` and `demos/rosbot/modified_pkg` are retained as
historical text/configuration examples from the previous ROS 2 package
exporter. They do not represent the current output layout. The current
exporter produces `robot.urdf` and `meshes/*.stl`; closed-chain metadata is
embedded in the URDF. Large historical CAD, STL, and screenshot assets are
kept out of the source release so the repository remains a practical plugin
download; they are not needed to install or run the exporter.

The old upstream `README.pdf` is likewise not included in the source release.
It describes the obsolete ROS 2 package workflow and is not the installation
or output specification for this version; use this README instead.

## Open-source attribution and license

The plugin is distributed under the MIT License; see [`LICENSE`](LICENSE).
This repository contains substantial modifications, including the pure
URDF/STL output format, top-level subassembly export, Fusion geometry-based
joint coordinates, closed-chain metadata, and high-refinement STL export.

The upstream projects remain credited and their MIT notices are preserved in
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md). Files under the historical
ROS demo directories may retain their original Apache License 2.0 notices;
those demos are not imported at runtime by the current exporter.

## Validation

The pure URDF writer can be tested outside Fusion:

```bash
python3 -m unittest discover -s tests -v
```

An end-to-end export still requires Fusion 360 because component geometry and
physical properties are read through the Fusion API.
