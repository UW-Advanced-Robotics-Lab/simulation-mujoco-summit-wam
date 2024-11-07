import mujoco
import mediapy as media
import mujoco.msh2obj

xml = """
<mujoco model="playground_wagon_manipulation(v1.0.0)">
    <!--#########################
        ### Compiler Specific ###
        ######################### -->
    <compiler coordinate="local" angle="radian"/>
    
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

    <!--###############
        ### Config: ###
        ############### -->
    <!-- DEFAULT -->
    <default>
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
            <geom contype="0" conaffinity="0" group="1" rgba="0.6 0.6 0.6 1"/>
        </default>
        <default class="summit/wheel/viz">
            <geom contype="0" conaffinity="0" group="1" rgba="0.3 0.3 0.3 1"/>
        </default>

        <default class="wam/col">
            <geom type="mesh" contype="1" conaffinity="1" group="0" rgba="0.5 0.6 0.7 0.6"/>
        </default>
        <default class="wam/viz">
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.7 0.7 0.7 1"/>
        </default>
    </default>

    <worldbody>
        <!-- GND -->
        <light directional="true" cutoff="60" exponent="1" diffuse="1 1 1" specular=".1 .1 .1" pos=".1 .2 1.3" dir="-.1 -.2 -1.3"/>
        <geom name="ground" type="plane" pos="0 0 0" size="24 24 1" conaffinity="1" contype="1" material="MatGnd" friction="0.0001 0.001 0.001" group="1"/>
        <body name="waterloo_steel" pos="0 0 0" euler="0 0 -1.57079632679">
            <body name="smt/base_link" pos="0 0 0">
                <!-- === BASE LINK === === === === -->
                <!-- mobile body -->
                <inertial                                           pos="0 0 0.37" mass="125" diaginertia="1.391 6.853 6.125" />
                <geom name="smt/viz/base_link"                      class="summit/body/viz" pos="0 0 0.42" size="0.24 0.32 0.16" type="box"/>

                <!-- Simplified the collision into boxes + arm rest -->
                <geom name="smt/col/base_link/box_body"             class="summit/col" pos="0 0 0.42" size="0.24 0.32 0.16" type="box"/>

                <!-- world placement -->
                <joint name="smt/world_x" armature="0.0001" axis="1 0 0" damping="1e+11" pos="0 0 0" type="slide"></joint>
                <joint name="smt/world_y" armature="0.0001" axis="0 1 0" damping="1e+11" pos="0 0 0" type="slide"></joint>
                <joint name="smt/world_z" armature="0.0001" axis="0 0 1" damping="1e+0"  pos="0 0 0" type="slide"></joint>

                <!-- control summit base -->
                <joint name="smt/pose/x"                            type="slide" pos="0 0 0.4" axis="1 0 0"  damping="15"/>
                <joint name="smt/pose/y"                            type="slide" pos="0 0 0.4" axis="0 1 0"  damping="15"/>
                <joint name="smt/orie/z"                            type="hinge" pos="0 0 0.4" axis="0 0 1"  damping="10"/>

                <site name="imu_sensor_site_mb" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>

                <!-- === whl LINK === === === === -->
                <!-- Mobile whls -->
                <body name="smt/whl/LF_link" pos="0 0 0">
                    <geom name="smt/viz/whl/LF"                    class="summit/wheel/viz" pos="-0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <geom name="smt/col/whl/LF_cylinder"           class="summit/col" pos="-0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                         pos="-0.220 0.222 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_LF"                       class="summit/whl" pos="-0.264 0.222 0.128"/>
                </body>
                <body name="smt/whl/LR_link" pos="0 0 0">
                    <geom name="smt/viz/whl/LR"                    class="summit/wheel/viz" pos="-0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <geom name="smt/col/whl/LR_cylinder"           class="summit/col" pos="-0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                         pos="-0.220 -0.223 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_LR"                       class="summit/whl" pos="-0.264 -0.223 0.128"/>
                </body>
                <body name="smt/whl/RF_link" pos="0 0 0">
                    <geom name="smt/viz/whl/RF"                   class="summit/wheel/viz" pos="0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <geom name="smt/col/whl/RF_cylinder"          class="summit/col" pos="0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                        pos="0.220 0.222 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_RF"                      class="summit/whl" pos="0.264 0.222 0.128"/>
                </body>
                <body name="smt/whl/RR_link" pos="0 0 0">
                    <geom name="smt/viz/whl/RR"                   class="summit/wheel/viz" pos="0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <geom name="smt/col/whl/RR_cylinder"          class="summit/col" pos="0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                    <inertial                                                        pos="0.220 -0.223 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                    <joint name="smt/whl_RR"                      class="summit/whl" pos="0.264 -0.223 0.128"/>
                </body>

                <!-- === WAM 7DOF HAND LINK === === === === -->
                <!-- Place WAM Module Here -->
                <!-- WAM -->
                <body name="wam/base_link" pos="0 -0.24 0.505" quat="0.707 0 0 0.707">
                    <inertial   pos="-0.14071720 -0.02017671 0.07995294" mass="9.97059584" fullinertia="0.11760385 0.10916849 0.18294303 0.02557874 0.00161433 0.00640270" /> 
                    <geom       class="wam/viz" type="cylinder" size="0.1 0.1" rgba="0.7 0 0 1"/>
                    <geom       class="wam/col" type="cylinder" size="0.1 0.1"/>
                    <site name="imu_sensor_site_wam" type="box" size="0.05 0.05 0.05" rgba="0.9 0 0 0.3"/>
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

        <exclude body1="smt/base_link" body2="wam/base_link"/>
    </contact>
    <sensor>
        <framepos name="global_pos_mb_body" objtype="body" objname="smt/base_link"/>
        <framepos name="global_pos_mb_site" objtype="site" objname="imu_sensor_site_mb"/>
        <framepos name="global_pos_wam_body" objtype="body" objname="wam/base_link"/>
        <framepos name="global_pos_wam_site" objtype="site" objname="imu_sensor_site_wam"/>
    </sensor>
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
    # data.actuator('smt/pose/x').ctrl = 0.1
    # Mobile-base position
    sensor_data_mb_body = data.sensor('global_pos_mb_body').data.copy()
    sensor_data_mb_site = data.sensor('global_pos_mb_site').data.copy()
    # Wam position
    sensor_data_wam_body = data.sensor('global_pos_wam_body').data.copy()
    sensor_data_wam_site = data.sensor('global_pos_wam_site').data.copy()
    mujoco.mj_step(model, data)
    if len(frames) < data.time * framerate:
      renderer.update_scene(data, scene_option=scene_option)
      pixels = renderer.render()
      frames.append(pixels)

# Print twist of the MB and the Shoulder-link
print('smt/base_link')
print("Position of smt/base_link's origin w.r.t world-frame: "+str(data.body('smt/base_link').xpos))
print("Position of smt/base_link's origin w.r.t world-frame from sensor at body: "+str(sensor_data_mb_body))
print("Position of smt/base_link's origin w.r.t world-frame from sensor at site: "+str(sensor_data_mb_site))
print('wam/base_link')
print("Position of wam/base_link's origin w.r.t world-frame: "+str(data.body('wam/base_link').xpos))
print("Position of wam/base_link's origin w.r.t world-frame from sensor at body: "+str(sensor_data_wam_body))
print("Position of wam/base_link's origin w.r.t world-frame from sensor at site: "+str(sensor_data_wam_site))

# print("MB Yaw-rate: "+str(data.joint('smt/orie/z').qvel))

media.show_video(frames, fps=framerate)