import mujoco
import mediapy as media

xml = """
<mujoco model="playground_wagon_manipulation(v1.0.0)">
    <!--#########################
        ### Compiler Specific ###
        ######################### -->
    <compiler coordinate="local" angle="radian" meshdir="/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/meshes/"/>
    
    <!-- Options: -->
    <option 
        timestep="1e-3"
        iterations="10"
        tolerance="1e-10"
        impratio="1"
        gravity="0 0 -9.81"
        solver="Newton"
        cone="elliptic" 
    />
    <default>
        <geom margin="0.001" solimp="0.99 0.99 0.01" solref="0.01 1" condim="4" /> 
    </default>
    <!-- Statistic -->
    <!-- https://mujoco.readthedocs.io/en/latest/XMLreference.html#statistic -->
    <!-- Looka at this `center` point using the free-camera, which is located at a distance of 1.5x`extent` distance -->
    <!-- Apparently, the camera cannot zoom to within the extent distance, so keep it small. -->
    <!-- The value of `center` does not move the initial location of the camera view-point.-->
    <statistic center="0 0 0" extent="1"/>
    <!-- Visual -->
    <visual>
        <map fogstart="3" fogend="5" force="0.1" znear="0.001"/>
        <quality shadowsize="1024" offsamples="16"/>
        <scale forcewidth="0.05" contactwidth="0.05" contactheight="0.05" 
               connect="0.05" com="0.1" jointlength="0.1" jointwidth="0.01"
               framelength="0.1" framewidth="0.01"/>
        <!-- <global offwidth="1960" offheight="800"/> -->
    </visual>

    <!--############################
        ### Environment Specific ###
        ############################ -->
    <asset>
        <!-- [ENV] Ground PLane -->
        <texture name="groundplane" type="2d" builtin="checker" rgb1=".25 .26 .25" 
            rgb2=".22 .22 .22" width="100" height="100" mark="edge" markrgb=".3 .3 .3"/>
        <material name="MatGnd" texture="groundplane"  texrepeat="5 5" specular="1" shininess=".001" reflectance="0.00001"/>

        <mesh file="meshes_summit/base_link_summit.stl"/>
        <mesh file="meshes_summit/mecanum_LF_1.stl"/>
        <mesh file="meshes_summit/mecanum_LR_1.stl"/>
        <mesh file="meshes_summit/mecanum_RF_1.stl"/>
        <mesh file="meshes_summit/mecanum_RR_1.stl"/>
        <mesh file="meshes_summit/arm_rest.stl"/>
        <mesh file="meshes_sensors/intel_realsense_l515.stl"/>
        <mesh file="meshes_wam/base_link_fine.stl"/>
        <mesh file="meshes_wam/base_link_convex.stl"/>
        <mesh file="meshes_wam/shoulder_link_fine.stl"/>
        <mesh file="meshes_wam/shoulder_link_convex_decomposition_p1.stl"/>
        <mesh file="meshes_wam/shoulder_link_convex_decomposition_p2.stl"/>
        <mesh file="meshes_wam/shoulder_link_convex_decomposition_p3.stl"/>
    </asset>

    <!--##################
        ### Extension: ###
        ################## -->
    <extension>
    </extension>

    <!--##################
        ### Actuators: ###
        ################## -->
    <!-- [SUMMIT] 
        - Note:
            - The configuration below may not reflect the actual robot
    -->
    <actuator>
        <!-- [SUMMIT] control summit base in a effort fashion (Tim van Meijel) -->
        <motor name="smt/pose/x"  joint="smt/pose/x" ctrllimited="true" ctrlrange="-10.0 10.0" gear="100.0"/>
        <motor name="smt/pose/y"  joint="smt/pose/y" ctrllimited="true" ctrlrange="-10.0 10.0" gear="100.0"/>
        <motor name="smt/orie/z"  joint="smt/orie/z" ctrllimited="true" ctrlrange="-10.0 10.0" gear="50.0"/>
    </actuator>
        
    <!-- [WAM 7DOF] 
        - Note:
            - The configuration below may not reflect the actual robot
            - Position gain is based on the config file hosted in WAM PC
    -->
    <actuator>
        <!-- 
            Attributes Descriptions:
            - SPEC: https://web.barrett.com/support/WAM_Documentation/WAM_InertialSpecifications_AC-02.pdf
            - gear: scakes the length of the actuator (different gain / moment arms and velocity, force)
        -->
        <!-- [WAM 7DOF] force control mode -->
        <!-- Limits initialized by the URDF Files!! -->
        <motor name='wam/J1/F' ctrllimited="true" joint='wam/J1' ctrlrange='-80 80' gear="42"      />
    </actuator>

    <!--###############
        ### Config: ###
        ############### -->
    <!-- DEFAULT -->
    <default>
        <!-- The class-name being specified under default maps back to the class-names being specified for a body (childclass/class) , or  -->
        <!-- These names have to be unique; it must not clash with class-names for un-intended bodies/joints/geometries -->
        <default class="summit">
        </default>
        <default class="summit/whl">
            <!-- class-names for joints -->
            <joint axis="1 0 0" damping="0.55"/>
            <!-- Bydefault, hinge-type is the default joint type -->
            <!-- https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint -->
        </default>
        <default class="summit/col">
            <!-- class-names for geometry-type -->
            <geom contype="1" conaffinity="1" group="0" rgba="0.5 0.6 0.7 0.3"/>
        </default>
        <default class="summit/body/viz">
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.6 0.6 0.6 1"/>
        </default>
        <default class="summit/wheel/viz">
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.3 0.3 0.3 1"/>
        </default>
        <default class="summit/camera/viz">
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.7 0.7 0.7 1"/>
        </default>

        <default class="wam">
        </default>
        <default class="wam/col">
            <geom type="mesh" contype="1" conaffinity="1" group="0" rgba="0.5 0.6 0.7 0.6"/>
        </default>
        <default class="wam/viz">
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.7 0.7 0.7 1"/>
        </default>
        <default class="wam/shoulder_yaw_link/viz">
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.8 0.8 0.8 1"/>
        </default>
    </default>

    <worldbody>
        <!-- GND -->
        <light directional="true" cutoff="60" exponent="1" diffuse="1 1 1" specular=".1 .1 .1" pos=".1 .2 1.3" dir="-.1 -.2 -1.3"/>
        <geom name="ground" type="plane" pos="0 0 0" size="24 24 1" conaffinity="1" contype="1" material="MatGnd" friction="0.0001 0.001 0.001" group="1"/>
        <body name="waterloo_steel" pos="0.8 0 0" euler="0 0 -1.57079632679">
            <body name="smt/base_link" childclass="summit" pos="0 0 0">
                <!-- === BASE LINK === === === === -->
                <!-- mobile body -->
                <inertial                                           pos="0 0 0.37" mass="125" diaginertia="1.391 6.853 6.125" />
                <geom name="smt/viz/base_link"                      class="summit/body/viz" mesh="base_link_summit"/>

                <!-- Front Camera -->
                <body name="smt/front/camera" pos="0 0.362 0.373" quat="0 0 0.707107 0.707107">
                    <inertial               pos="0 0 0.015" mass="0.095"/>
                    <geom                   class="summit/camera/viz" size="0.06 0.04" mesh="intel_realsense_l515"/>
                    <camera name="smt/front/camera/intel/rgb" pos="0 0 0" quat="0 0 1 0" fovy="55"/>
                </body>
                <!-- Rear Camera --><!-- quant=w x y z -->
                <body name="smt/rear/camera" pos="0 -0.362 0.373" quat="0.707107 0.707107 0 0"> 
                    <inertial               pos="0 0 0.015" mass="0.095"/>
                    <geom                   class="summit/camera/viz" size="0.06 0.04" mesh="intel_realsense_l515"/>
                    <camera name="smt/rear/camera/intel/rgb" pos="0 0 0" quat="0 0 1 0" fovy="55"/>
                </body>
                <!-- Simplified the collision into boxes + arm rest -->
                <geom name="smt/col/base_link/box_body"             class="summit/col" pos="0 0 0.42" size="0.24 0.32 0.16" type="box"/>
                <geom name="smt/col/base_link/arm_rest"             class="summit/col" pos="0 0.098 0.55" quat="0.5 0.5 -0.5 -0.5" mesh="arm_rest"/>
                <geom name="smt/col/base_link/cylinder_lidar"       class="summit/col" pos="0.246 0.362 0.313" size="0.06 0.044" type="cylinder"/>

                <!-- world placement -->
                <joint name="smt/world_x" armature="0.0001" axis="0 1 0" damping="1e+11" pos="0 0 0" type="slide"></joint>
                <joint name="smt/world_y" armature="0.0001" axis="-1 0 0" damping="1e+11" pos="0 0 0" type="slide"></joint>
                <joint name="smt/world_z" armature="0.0001" axis="0 0 1" damping="1e+0"  pos="0 0 0" type="slide"></joint>

                <!-- control summit base -->
                <joint name="smt/pose/x"                            type="slide" pos="0 0 0.4" axis="0 1 0"  damping="15"/>
                <joint name="smt/pose/y"                            type="slide" pos="0 0 0.4" axis="-1 0 0"  damping="15"/>
                <joint name="smt/orie/z"                            type="hinge" pos="0 0 0.4" axis="0 0 1"  damping="10"/>

                <!-- === whl LINK === === === === -->
                <!-- Mobile whls -->
                <body name="smt/whl/LF_link" pos="0 0 0">
                    <geom name="smt/viz/whl/LF"                    class="summit/wheel/viz" mesh="mecanum_LF_1"/>
                    <geom name="smt/col/whl/LF_cylinder"           class="summit/col" pos="-0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                         pos="-0.220 0.222 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_LF"                       class="summit/whl" pos="-0.264 0.222 0.128"/>
                </body>
                <body name="smt/whl/LR_link" pos="0 0 0">
                    <geom name="smt/viz/whl/LR"                    class="summit/wheel/viz" mesh="mecanum_LR_1"/>
                    <geom name="smt/col/whl/LR_cylinder"           class="summit/col" pos="-0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                         pos="-0.220 -0.223 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_LR"                       class="summit/whl" pos="-0.264 -0.223 0.128"/>
                </body>
                <body name="smt/whl/RF_link" pos="0 0 0">
                    <geom name="smt/viz/whl/RF"                   class="summit/wheel/viz" mesh="mecanum_RF_1"/>
                    <geom name="smt/col/whl/RF_cylinder"          class="summit/col" pos="0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                        pos="0.220 0.222 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_RF"                      class="summit/whl" pos="0.264 0.222 0.128"/>
                </body>
                <body name="smt/whl/RR_link" pos="0 0 0">
                    <geom name="smt/viz/whl/RR"                   class="summit/wheel/viz" mesh="mecanum_RR_1"/>
                    <geom name="smt/col/whl/RR_cylinder"          class="summit/col" pos="0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                        pos="0.220 -0.223 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_RR"                      class="summit/whl" pos="0.264 -0.223 0.128"/>
                </body>

                <!-- === WAM 7DOF HAND LINK === === === === -->
                <!-- Place WAM Module Here -->
                <!-- WAM -->
                <body name="wam/base_link" childclass="wam" pos="0 0.14 0.405" quat="0.707 0 0 0.707">
                    <body name="wam/base" childclass="wam" pos="0 0 0">
                        <inertial   pos="-0.14071720 -0.02017671 0.07995294" mass="9.97059584" fullinertia="0.11760385 0.10916849 0.18294303 0.02557874 0.00161433 0.00640270" /> 
                        <geom       class="wam/viz" mesh="base_link_fine"/>
                        <geom       class="wam/col" mesh="base_link_convex"/>
                    </body>
                    <body name="wam/shoulder_yaw_link" pos="0 0 0.346">
                        <inertial               pos="-0.00443422 -0.00066489 -0.12189039" mass="10.76768767" fullinertia="0.13488033 0.09046330 0.11328369 -0.00012485 0.00213041 -0.00068555" />
                        <joint name="wam/J1"    range="-2.6 2.6" damping="1000" frictionloss="1000" type="hinge" limited="true" pos="0 0 0" axis="0 0 1"/>
                        <geom                   class="wam/shoulder_yaw_link/viz" mesh="shoulder_link_fine"/>
                        <geom                   class="wam/col" mesh="shoulder_link_convex_decomposition_p1"/>
                        <geom                   class="wam/col" mesh="shoulder_link_convex_decomposition_p2"/>
                        <geom                   class="wam/col" mesh="shoulder_link_convex_decomposition_p3"/>
                        <body name="wam/shoulder_pitch_link" pos="0 0 0" quat="0.707107 -0.707107 0 0"> 
                            <!-- ### Inertial Ref: (*1, pg.14) unit: [m, kg, kg * m^2] 
                                Note:
                                    - frame correction to the stl: <x, -z, y>
                            -->
                            <!-- <body name="wam/shoulder_pitch_link" pos="0 0 0"> -->
                            <inertial               pos="-0.00236983 -0.01542114 0.03105614" mass="3.8749" fullinertia="0.02140958 0.01558906 0.01377875 -0.00002461 0.00027172 0.00181920" />
                            <joint name="wam/J2"    range="-2.0 2.0" damping="1000" frictionloss="1000" type="hinge" limited="true" pos="0 0 0" axis="0 0 1"/>
                            <geom                   class="wam/shoulder_pitch_link/viz" mesh="shoulder_pitch_link_fine"/>
                            <geom                   class="wam/col" mesh="shoulder_pitch_link_convex"/>
                            <!-- Place imu sensors here. -->
                            <site name="imu_sensor_site_wam_shoulder_pitch_link" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>

                            <body name="wam/upper_arm_link" pos="0 0 0" quat="0.707107 0.707107 0 0">
                                <!-- ### Inertial Ref: (*1, pg.16) unit: [m, kg, kg * m^2] 
                                    Note: 
                                        - we have to offset the stl model frame to the inertial frame in doc
                                        - pos = <0.045, 0, 0.55> - <0.03825858, 0.00003309, 0.20750770>
                                        - frame correction to the stl: <x, -z, -y>
                                -->
                                <!-- <body name="wam/upper_arm_link" pos="0 0 0"> -->
                                <inertial               pos="0.00674142 -0.00003309 0.3424923" mass="1.8028141" fullinertia="0.05911077 0.05927043 0.00324550 -0.00000738 0.00249612 -0.00001767" />
                                <joint name="wam/J3"    range="-2.8 2.8" damping="500" frictionloss="1000" type="hinge" limited="true" pos="0 0 0" axis="0 0 1"/>
                                <geom                   class="wam/upper_arm_link/viz" mesh="upper_arm_link_fine"/>
                                <geom                   class="wam/col" mesh="upper_arm_link_convex_decomposition_p1"/>
                                <geom                   class="wam/col" mesh="upper_arm_link_convex_decomposition_p2"/>
                                <!-- Place imu sensors here. -->
                                <site name="imu_sensor_site_wam_upper_arm_link" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>
                                
                                <body name="wam/forearm_link" pos="0.045 0 0.55" quat="0.707107 -0.707107 0 0">
                                    <!-- ### Inertial Ref: (*1, pg.22) unit: [m, kg, kg * m^2]
                                        [ Elbow + WristBase (7DOF) ]
                                        Note: 
                                            - 4DOF has a different attribute here
                                            - we have to offset the stl model frame (at the elbow joint) to the inertial frame in doc (z4 aligned with central axis of the elbow)
                                            - pos = <0.00498512 -0.1327162 -0.00022942> - <0.045, 0, 0>
                                            - frame correction to the stl: <x, -z, y>
                                    -->
                                    <!-- <body name="wam/forearm_link" pos="0.045 0 0.55"> -->
                                    <inertial               pos="-0.04001488 -0.1327162 -0.00022942" mass="2.40016804" fullinertia="0.01491672 0.00294463 0.01482922 0.00150604 0.00001741 0.00002109" />
                                    <joint name="wam/J4"    range="-0.9 3.1" damping="200" frictionloss="1000" type="hinge" limited="true" pos="0 0 0" axis="0 0 1"/>
                                    <geom                   class="wam/elbow_link/viz" mesh="elbow_link_fine"/>
                                    <geom                   class="wam/col" mesh="elbow_link_convex"/>
                                    <geom                   class="wam/fore_arm_link/viz" mesh="forearm_link_fine" pos="-.045 -0.0730 0" euler="1.57 0 0"/>
                                    <geom                   class="wam/col" mesh="forearm_link_convex_decomposition_p1" pos="-0.045 -0.0730 0" euler="1.57 0 0"/>
                                    <geom                   class="wam/col" mesh="forearm_link_convex_decomposition_p2" pos="-0.045 -0.0730 0" euler="1.57 0 0"/>
                                    <!-- Place imu sensors here. -->
                                    <site name="imu_sensor_site_wam_forearm_link" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>

                                    <!-- [Optional] ZED Camera on forearm link, TODO: uncertain for inertial params -->
                                    <!-- <body name="wam/sensor/zed" pos="-0.18 -0.15 0" quat="0 0.707107 0 0.707107">
                                        <inertial               pos="0 0 0.015" mass="0.124"/>
                                        <geom                   class="wam/viz" mesh="zed"/>
                                        <geom                   class="wam/col" mesh="zed_collision_mesh"/>
                                        <camera name="camera/zed/R" pos="-0.06 0.03 0.042"  quat="0 0 -0.707107 0.707107" fovy="60"/>
                                        <camera name="camera/zed/L" pos=" 0.06 0.03 0.042"  quat="0 0 -0.707107 0.707107" fovy="60"/>
                                    </body> -->
                                    
                                    <body name="wam/wrist_yaw_link" pos="-0.045 -0.3 0" quat="0.707107 0.707107 0 0">
                                        <!-- ### Inertial Ref: (*1, pg.24) unit: [m, kg, kg * m^2] 
                                            Note:
                                                - frame correction to the stl: <x, z, -y>
                                        -->
                                        <!-- <body name="wam/wrist_yaw_link" pos="-0.045 -0.3 0"> -->
                                        <inertial               pos="0.00008921 0.00435824 -0.00511217" mass="0.12376019" fullinertia="0.00005029 0.00006270 0.00007582 -0.00000005 -0.00000020 0.00000359" /> 
                                        <joint name="wam/J5"    range="-4.8 1.3" damping="50" frictionloss="1000" type="hinge" limited="true" pos="0 0 0" axis="0 0 1"/>
                                        <geom                   class="wam/wrist_yaw_link/viz" mesh="wrist_yaw_link_fine"/>
                                        <geom                   class="wam/col" mesh="wrist_yaw_link_convex_decomposition_p1"/>
                                        <geom                   class="wam/col" mesh="wrist_yaw_link_convex_decomposition_p2"/>
                                        <!-- Place imu sensors here. -->
                                        <site name="imu_sensor_site_wam_wrist_yaw_link" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>

                                        <body name="wam/wrist_pitch_link" pos="0 0 0" quat="0.707107 -0.707107 0 0">
                                            <!-- ### Inertial Ref: (*1, pg.26) unit: [m, kg, kg * m^2] 
                                                Note:
                                                    - frame correction to the stl: <x, -z, y>
                                            -->
                                            <!-- <body name="wam/wrist_pitch_link" pos="0 0 0" > -->
                                            <inertial               pos="-0.00012262 -0.02468336 -0.01703194" mass="0.41797364" fullinertia="0.00055516 0.00045358 0.00024367 0.00000074 0.00000061 0.00004590" />
                                            <joint name="wam/J6"    range="-1.6 1.6" damping="50" frictionloss="1000" type="hinge" limited="true" pos="0 0 0" axis="0 0 1"/>
                                            <geom                   class="wam/wrist_pitch_link/viz" mesh="wrist_pitch_link_fine"/>
                                            <geom                   class="wam/col" mesh="wrist_pitch_link_convex_decomposition_p1"/>
                                            <geom                   class="wam/col" mesh="wrist_pitch_link_convex_decomposition_p2"/>
                                            <geom                   class="wam/col" mesh="wrist_pitch_link_convex_decomposition_p3"/>
                                            <!-- Place imu sensors here. -->
                                            <site name="imu_sensor_site_wam_wrist_pitch_link" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>

                                            <body name="wam/wrist_palm_link" pos="0 0 0" quat="0.707107 0.707107 0 0">
                                                <!-- ### Inertial Ref: (*1, pg.14) unit: [m, kg, kg * m^2] 
                                                    Note:
                                                        - frame correction to the stl: <x, y, z + 0.055>
                                                        - [Optional] We could neglect x,y components as if it is a disc
                                                -->
                                                <!-- <body name="wam/wrist_palm_link" pos="0 0 0"> -->
                                                <inertial               pos="-0.00007974 0.0001613 0.05176448" mass="0.06864753" fullinertia="0.00003773 0.00003806 0.00007408 -0.00000019 0.00000000 0.00000000" />
                                                <joint name="wam/J7"    range="-2.2 2.2" damping="50" frictionloss="1000" type="hinge" limited="true" pos="0 0 0" axis="0 0 1"/>
                                                <geom                   class="wam/wrist_palm_link/viz" mesh="wrist_palm_link_fine"/>
                                                <geom                   class="wam/col" mesh="wrist_palm_link_convex"/>
                                                <!-- Place imu sensors here. -->
                                                <site name="imu_sensor_site_wam_wrist_palm_link" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>
                                                
                                                <!-- [Optional] Intel Lidar Camera (*4) -->
                                                <body name="wam/camera" pos="0 0.085 0.02">
                                                    <inertial               pos="0 0 0.015" mass="0.095"/>
                                                    <geom                   class="wam/viz" mesh="intel_realsense_l515"/>
                                                    <geom                   class="wam/col" mesh="intel_realsense_l515_collision_mesh"/>
                                                    <camera name="wam/camera/intel/rgb" quat="0 0 1 0" fovy="55"/><!-- quant=w x y z -->
                                                </body>
                                                
                                                <!-- ### T-S Sensor (*3) Ref: (*3) unit: [m, kg, kg * m^2] 
                                                    Note:
                                                        - [USER PARAMS] 0.06: mounting position z-axis depends on the actual measurements
                                                        - No need for the collision mesh, as the disk size is think enough that can be negligible as if its an air gap
                                                        - stl: generated from original CAD file [Unit Conversion: 1 g*mm^2 = 1e-9 kg*m^2]
                                                        - frame correction to the stl: <x, z, y>
                                                -->
                                                <body name="wam/torque_sensor_link" pos="0 0 0.060" quat="0.707107 0.707107 0 0">
                                                    <inertial               pos="0 0.0085 0" mass="0.133278" fullinertia="0.00007551 0.0001462 0.00007508 -7.002e-8 -1.199e-7 -5.37e-8" />
                                                    <geom                   class="wam/torque_sensor_link/viz" mesh="torque_sensor_plate"/>
                                                    <!-- Insert Force-Torque Sensor -->
                                                    <!-- https://github.com/google-deepmind/mujoco/issues/353#issuecomment-1162445382 -->
                                                    <site name="force_torque_sensor" type="box" size="0.1 0.1 0.1" rgba="0.9 0 0 0.3"/>
                                                    <!-- ### WAM BHAND 
                                                        Note:
                                                            - [USER PARAMS] 0.01: mounting gap
                                                    -->
                                                    <body name="wam/bhand"  pos="0 0.01 0"  quat="0 0 0.707107 0.707107">
                                                        <include file="../components/include_bhand_Chain.xml"/>
                                                    </body>
                                                </body>
                                            </body>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>
    <!-- CONTACT -->
    <contact>
    <!-- NOTE: Exclude overlapping collision mesh to be used in contact physics.  -->
        <exclude body1="smt/base_link" body2="smt/whl/LF_link"/>
        <exclude body1="smt/base_link" body2="smt/whl/RF_link"/>
        <exclude body1="smt/base_link" body2="smt/whl/RR_link"/>
        <exclude body1="smt/base_link" body2="smt/whl/LR_link"/>

        <exclude body1="wam/base" body2="wam/shoulder_yaw_link"/>
    </contact>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

duration = 3  # (seconds)
framerate = 60  # (Hz)

# Simulate and display video.
frames = []
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
  while data.time < duration:
    # Give joint commands
    data.actuator('smt/orie/z').ctrl = 0.1
    # data.actuator('smt/pose/x').ctrl = 0.1
    # data.actuator('wam/J1/F').ctrl = 42*(0.1-data.joint('wam/J1').qvel[0])
    mujoco.mj_step(model, data)
    if len(frames) < data.time * framerate:
      renderer.update_scene(data, scene_option=scene_option)
      pixels = renderer.render()
      frames.append(pixels)
# Print twist of the MB and the Shoulder-link
print('smt/base_link')
print(data.body('smt/base_link').xpos)
print(data.body('smt/base_link').xquat)
print(data.body('smt/base_link').cvel)
print('wam/base_link')
print(data.body('wam/base_link').xpos)
print(data.body('wam/base_link').xquat)
print(data.body('wam/base_link').cvel)
print('wam/shoulder_yaw_link')
print(data.body('wam/shoulder_yaw_link').xpos)
print(data.body('wam/shoulder_yaw_link').xquat)
print(data.body('wam/shoulder_yaw_link').cvel)
print("MB Yaw-rate")
print(data.joint('smt/orie/z').qvel)
print("Wam Shoulder yaw-Link Yaw-pos")
print(data.joint('wam/J1').qpos)
print("Wam Shoulder yaw-Link Yaw-rate")
print(data.joint('wam/J1').qvel)

print("Hey!")
media.show_video(frames, fps=framerate)