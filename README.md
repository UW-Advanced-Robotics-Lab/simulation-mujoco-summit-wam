<toc>

# Table of Contents
[*Last generated: Thu 07 Dec 2023 02:38:20 PM EST*]
- [**Mujoco Simulation Packages**](#Mujoco-Simulation-Packages)
  - [1. Preview:](#1-preview)
    - [1.1 Custom Library to support with MuJoCo 2.2.x](#11-custom-library-to-support-with-mujoco-22x)
    - [1.2 Waterloo Steel Mobile Playground (engine & viewer):](#12-waterloo-steel-mobile-playground-engine--viewer)
  - [2. ToDo:](#2-todo)
  - [3. Note:](#3-note)
  - [4. Installation](#4-installation)
  - [5. Launch:](#5-launch)
    - [5.1 Cart Manipulation:](#51-cart-manipulation)
      - [Variables of simulation:](#variables-of-simulation)
    - [5.2 Environment model location:](#52-map-model-location)
    - [5.3 Peg-in-Hole:](#53-peg-in-hole)
    - [5.4 Map Wall Seperation:](#53-map-wall-seperation)
  - [6. File Hierarchy:](#6-File-Hierarchy)
  - [A*. Appendix:](#A-Appendix)
    - [A.1 Note:](#A1-Note)
    - [A.2 Waterloo Steel Mobile Manipulator Simulation:](#A2-Waterloo-Steel-Mobile-Manipulator-Simulation)
      - [A.2.1 AJoints and MOI:](#A21-AJoints-and-MOI)
    - [A.2.2 Installation Guide Extra](#A22-Installation-Guide-Extra)
      - [A.2.3 Tested Platforms:](#A23-Tested-Platforms)
      - [A.2.4 M1 Macbook Instructions:](#A24-M1-Macbook-Instructions)
      - [A.2.5 Unity module:](#A25-Unity-module)
    - [A.3 MuJoCo-ROS integration](#A3-MuJoCo-ROS-integration)

---
</toc>
# Mujoco Simulation Packages

Mujoco Physics Simulation Package for Waterloo Steel Robot

## 1. Preview:

### 1.1 Custom Library to support with MuJoCo 2.2.x

- [x] Launch Package (includes models) (**This Package**)
- [x] Graphical User Interface / Direct OnOff-screen Render: [uwarl-mujoco-python-viewer](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-viewer)
  -  branching from [jx-mujoco-python-viewer](https://github.com/jaku-jaku/jx-mujoco-python-viewer)
- [x] Main Engine Code [uwarl-mujoco-python-engine](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-engine)
  - branching from [jx-mujoco-python-engine](https://github.com/jaku-jaku/jx-mujoco-python-engine) (similar to [deepmind/dm_control](https://github.com/deepmind/dm_control))
- [x] MuJoCo 2.2.x locking variants [jx-mujoco](https://github.com/jaku-jaku/jx-mujoco)
- [x] [TODO: ROS Integration] for bridging SIL and HIL
- [ ] [TODO: Unity Integration] for rendering and realistic camera views
- [ ] [TODO: Migration to MuJoCo 3] for rendering higher simulation speeds

### 1.2 Waterloo Steel Mobile Playground (engine & viewer):
<img src="./documentation/main.png" alt="waterloo_steel" height="600"/>

## 2. ToDo:
- [x] Full Assembly
- [x] Simulation setup
- [x] Contact Physics [Last Edit: 15/Jun/2022]
- [x] [WAM] Ensure Mechanical Params are Verified
- [x] [BHAND] Ensure Mechanical Params are Verified
- [?] [SUMMIT] Ensure Mechanical Params are Verified
- [x] Control Descriptors
- [x] PID control for base
- [x] ROS integration
- [x] Real-time simulation synchronization
- [ ] Find solution for convex hull of wagon handle
- [ ] Passing all variables in highest level launch file
- [ ] ....

## 3. Note:
> :announcement: Starting version 2.1.2, MuJoCo comes with python bindings, no need to look into mujoco_py package (which only works for 210)
> Migration notes: https://mujoco.readthedocs.io/en/latest/python.html#migration-notes-for-mujoco-py
> Right now, we will use mujoco-viewer based on https://github.com/rohanpsingh/mujoco-python-viewer
> TODO: We will migrate to the official viewer in python bindings later: Watch PR: https://github.com/deepmind/mujoco/pull/201

## 4. Installation
1. Submodule Update
    ```zsh
    $ cd submodules
    $ git submodule update
    ```
2. Install editable [python viewer](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-viewer):
    ```zsh
    $ cd uwarl-mujoco-python-viewer
    $ pip install -e .
    ```
3. Install editable [python engine](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-engine):
    ```zsh
    $ cd uwarl-mujoco-python-engine
    $ pip install -e .
    ```

## 5. Launch:
### 5.1 Cart Manipulation:
Launches the cart manipulation simulation with ROS integration. The launch file will launch:
- Clock publisher for simulation time
- Mujoco Engine and Viewer based on submodules
- ROS controllers which are used for trajectory control of the WAM
- Hardware simulation interface node to connect MuJoCo to ROS controllers
- Trajectory following action server for mobile base 
- Rosbag recorder action server
- `Demo_V09_mujoco.py` node 

```
$ roslaunch waterloo_steel_sim_bringup waterloo_steel_complete_cart_mujoco.launch
```

#### Variables of simulation:
These variables can be changed depending on the simulation. 
- In `launch/mujocolaunch.launch` param `sim_frequency_mujoco`. This changes the frequency of the node which updates the engine. It is synched to the simulation Hardware Simulation Interface for control of the WAM.
- In `components/include_common.xml` param `timestep` for changing the engines stepsize.

> [!IMPORTANT]
> Make sure 1/frequency of the ROS node updating the engine is equal to a multiple of the engine stepsize for real-time simulation.

- In `playground/playground_mobile_wagon_manipulation.xml` comment out the world body and the contact exclusions to simulate without a world. This increases the rendering performance. 
  - You might want to compress the mesh to increase rendering performance, by using MeshLab for example. 
- In `components/include_e7_3rd_floor_Dependencies.xml`, the world .stl file is defined. Change to simulate different world.
- In `src/main.py` the MuJoCo viewer rate can be defined. 
- In `src/main.py` the onboard cameras of the Summit and WAM can be enabled. This can be done by passing `True` in the Engine `_update` function. Default value equals `False`.

### 5.2 Map model location
When needing to change the map in which one is simulating in, follow the following path:
- In `playground/playground_mobile_wagon_manipulation.xml` file:
```
        <!-- E7 3rd floor -->
        <body name="environment" pos="-1.1 -1.35 -0.6" euler="0 0 0.03">
            <include file="../components/include_e7_3rd_floor.xml"/>   
        </body>
```
 edit the following line to point to the desired map description:
```
<include file="../components/include_e7_3rd_floor_Dependencies.xml"/>
```
- In `.../include_e7_3rd_floor_Dependencies.xml`, you have to point to the stl-file representing the map.
- An stl-representation of a 2D-floor-map can be generated using the `map2gazebo.launch` launch file (instructions are [here]()).
- The map mesh is associated with the name `map_e7_3rd_floor`, and this name is again referenced in `include_e7_3rd_floor.xml`. 

- Best that one creates seperate files for `include_e7_3rd_floor.xml` and `include_e7_3rd_floor_Dependencies.xml` when switching to a new simulation map, for better readability.

### 5.3 Peg-in-Hole
<img src="./documentation/peg_in_hole.png" alt="peg_in_hole" height="600"/>
Peg-in-Hole is an action by which a peg (a cylinder in the above image) is inserted into a hole in a work-piece. For reasons best explained [here](https://github.com/google-deepmind/mujoco/discussions/738), MuJoCo converts all meshes into their convex-hulls. This, howeever, prevents the emulation of a peg-in-hole action, or, as in our-case, prevents the BHand on the WAM from grasping the cart-handle.

Here, we detail out the process to be followed for carrying out Peg-in-Hole experiments in MuJoCo:
- In the same post as above, install the recommended repo called `obj2mjcf`, that can be found [here](https://github.com/kevinzakka/obj2mjcf). MuJoCo can use Wavefront OBJ `.obj` files directly.
- If you have the part as a CAD-file, you will need to sve it as either an `IGES` or `.stp` format.
  - AutoDesk Fusion 360 has a paid add-on to perform the conversion directly. It does not support exporting `.obj` natively.
  - SolidWorks also cannot export it natively.
- Use [FreeCAD](https://github.com/FreeCAD/FreeCAD/releases) to convert to Wavefront OBJ `.obj` (there's apparently another `.obj` format in the list which is not called Wavefront OBJ; have not tested the other one out).
- In a folder, keep the `.obj` files (along with its corresponding `.mtl` file; material description) in a folder.
- The call to the `obj2mjcf` function can be made as follows:
```
obj2mjcf --obj-dir ~/folder/having/obj_files --save-mjcf --compile-model --decompose 
```
- For every `.obj` file it finds, it will convert it into a set of convex-shapes that can be assembled together o get-back the original non-convex shape.

In order to test how does the mesh look like, set-up the `.obj' files as demonstrated in this `.xml` [file](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-summit-wam-sim/blob/universal/ros1/arnab/jan-2024/playground/playground_peg_in_hole.xml). Some things to keep in mind are as follows:
- Press `CTRL`+`R` to check the shape of the mesh that MuJoCo used (after taking its convex-hull). This is a great-way of trouble-shooting the cause of possible no Peg-in-Hole action.
- There are two sets of meshes that are being deployed: one under Group 0, and another under Group 1. By [default](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom-group), the MuJoCo viewer will only display (geom)etric-bodies that are labled as `1` (in-fact, `geom`-tags will be labeled as `0` by default, unless it is set to some other group-number).
- The last thing that you need to be aware of is the effect of `contype` [link](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom-contype) and `conaffinity` [link](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom-conaffinity). They are a 32-bit tag that helps pair what pair of bodies are allowed to collide based upon both having a common bit that is 1. So, a `contype=1` (0x0001) is allowed to experience collisions with a body having a `conaffinity=3` (0x0011) (a common 1 bit between 0x0001 and 0x0011), but not with a body having `conaffinity=2` (0x0010) (no common 1 bit between 0x0001 and 0x0010). This can become a source of headache if you use different meshes for visualization (MuJoCo has ended up transforming a part into its convex-hull) and collision (a se of bodies were provided by the user which were individually convex). Thus, while the later has a hole in it to perform Peg-in-Hole, the former does not. So, if we want to turn-off collisions between a set of meshes, especialyy ones that are just being used for visualization, set `contype=0` and `conaffinity=0`.

### 5.4 Map Wall Seperation
If you have tried to place a robot or object between the walls of a map, you will have noticed that the said object disappears. That's because the corridor is not hollow and its convex-hull had been used by MuJoCo. 
<img src="./documentation/mesh_convexification_needed.png" alt="mesh_convexification_needed" height="600"/>
A corridor in a map, as the one shown above, is composed of walls. When you obtain an `stl` file from the `.pgm` file of the 2D-map, all the walls, whether disjoint or not, are stored together, as can be attested by the fact that when hovering the mouse over a segment of the corridor wall, all the walls (disjoint or not) get highlighted. It is not clear wheter the `obj2mjcf` [program](https://github.com/kevinzakka/obj2mjcf) can seperate out such disjoint objects, that were saved in the same `stl`-file, or not. So, here, we highlight the steps needed to be followed for appropriately modelling the corridor-walls:

Upon opening FreeCAD,
1. Import the corridor-mesh you want to split. When hovering the mouse over one of the walls, the whole corridor, which was earlier [unselected], becomes [selcted]. We need to partiotion these dis-joint walls so that we can convexify each dis-joint wall separately.
<img src="./documentation/Unselected_Mesh.png" alt="unselected" height="600"/>
<img src="./documentation/Selected_Mesh.png" alt="selected" height="600"/>
2. Select [Mesh](https://wiki.freecad.org/Mesh_Workbench) from the WorkBench Drop-down [menu](https://wiki.freecad.org/Std_Workbench),
3. Click on the mesh you want to split so that the relevant option in the above toolbar will become selectable. In our case, all discrete walls will be considered together as one mesh; so all of them will be selected (see image),
4. Select the Cut-Mesh [option](https://wiki.freecad.org/Mesh_PolyCut), and draw a (red) closed-polygon around the segment you want to separate out.
5. Afterwards, the following parts will be obtained under the `Model` tab (see [separated]).
<img src="./documentation/After_split_mesh.png" alt="separated" height="600"/>
6. Since we are not able to save individual pieces (right-click the part under `Model` and select `Export Mesh`) as `Wavefront OBJ` files (there's an `.obj` option available, but, as stated before, not sure if this will work or not), we save it as an `stl` file, import it, and then export it as a `Wavefront OBJ` file.

This way, we can split-apart rooms and other corridors so that we can convexify each cut segment. DO NOT attempt to cut for the sake of convexifying the corridor. That's this (link) procedure's job. Our job is only to split the corridor into disjoint continuous walls. In this example we may not need to convexify the walls, but in case of an L-shaped wall (as seen here), we will need to convexify that segment.


## 6. File Hierarchy:
```
.
├── CITATION.cff
├── LICENSE
├── README.md
├── components
│   ├── meshes
│   │   ├── bases
│   │   ├── meshes_bhand
│   │   └── ...
│   ├── robots
│   │   ├── wam_7dof_wam_bhand.urdf.xacro
│   │   └── waterloo_steel_mujoco.urdf.xacro
│   ├── urdf
│   │   ├── bases
│   │   ├── wagon
│   │   ├── wam
│   │   └── wheels
│   ├── include_common.xml
│   ├── include_{assembly-name}_Chain.xml
│   ├── include_{assembly-name}_Dependencies.xml
│   ├── include_{assembly-name}_actuators.xml
│   └── ...
├── documentation
│   └── ...
├── include
│   └── mujoco_ros_control
│   │   ├── mujoco_ros_control.h
│   │   ├── robot_hw_sim_plugin.h
│   │   └── robot_hw_sim.h
├── meshes
│   ├── maps_thirdfloor
│   │   ├── map_e7_3_v6.stl
│   │   └── ...
│   ├── meshes_{module-name}
│   │   ├── {3D-model-component-name}.stl
│   │   └── ...
│   └── ...
├── playground
│   ├── playground_{playground-name}.xml
│   └── ...
├── src
│   ├── {scripts}.py # [launch files]
│   └── ...
├── submodules
│   ├── uwarl-mujoco-python-viewer # [Mujoco Render/Interaction GUI]
│   └── uwarl-mujoco-python-engine # [Main engine code]
└── textures
│   └── ...
x

[ 5 directories, # files ]
```

## A*. Appendix:
### A.1 Note:
- WAM sim file is a CORRECTED and MODIFIED version based on [the official archived MuJoCo model made by Vikash kumar](https://roboti.us/forum/index.php?resources/wam-and-barrett-hand.20/)
    - Findings: The original model has collision disabled, and parameters are incorrectly populated
    - Note: We have modified the original model based on the given stl files completely, and configured MoI based on [the Official Barrett WAM Specification](https://web.barrett.com/support/WAM_Documentation/WAM_InertialSpecifications_AC-02.pdf). 
        - Specifically, we made exactly the same as described in the document, and removed incorrect quaternion parameters for `inertial` , and populated the `inertial` purely based on the centre of the mass and translated the coordinate frames to the stl model frame (manually)
- Shall you have any concern with the parameters, kindly open an issue.

### A.2 Waterloo Steel Mobile Manipulator Simulation:

#### A.2.1 AJoints and MOI:
Joints             |  MOI
:-------------------------:|:-------------------------:
<img src="./documentation/joints.png" alt="waterloo_steel" height="300"/>  |  <img src="./documentation/MoI.png" alt="waterloo_steel" height="300"/>

### A.2.2 Installation Guide Extra
- Install MuJoCo 2.2.x via `$ sudo pip install mujoco`
- Download MuJoCo 2.2.x release package from https://github.com/deepmind/mujoco/releases
- The package "opencv-python-headless", which is installed during the process of setting-up the workspace (see the list of commands execcuted during the process), must be replaced with "opencv-python" [Source](https://github.com/opencv/opencv-python/issues/18):
```zsh
$ pip uninstall opencv-python-headless
$ pip install opencv-python
```
Otherwise, the window for MuJoCo will not open, and the simulation will crash.

#### A.2.3 Tested Platforms:
- [x] M1 Macbook Pro 14" 
- [x] Ubuntu 20.04
- [ ] [TBD] WINDOWS ---x

#### A.2.4 M1 Macbook Instructions:

- Make directory `MuJoCo_v2.2` under `/Applications`
- Copy all files from **MuJoCo 2.2.x** release dmg into `/Applications/MuJoCo_v2.2`

#### A.2.5 Unity module:
- The directory for the `.dylib` has been modified to `/Applications/MuJoCo_v2.2/MuJoCo.app/Contents/Frameworks` under this `submodules/jx-mujoco`


### A.3 MuJoCo-ROS integration
The current MuJoCo-ROS integration is shown below:
<img src="./documentation/mujoco_ros_integration.png" alt="mujoco_ros_integration" width="800"/>

In detail, zoomed in on the Hardware Simulation Interface:

<img src="./documentation/mujoco_ros_integration_detailed.png" alt="mujoco_ros_integration_detailed" width="800"/>

To find more details on the ROS integration of the MuJoCo simulator, see the pdf file in the documentation folder.



<eof>

---
[*> Back To Top <*](#Table-of-Contents)
</eof>