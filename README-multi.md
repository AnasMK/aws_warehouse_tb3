# AWS Small Warehouse with Multiple Turtlebot 3 Robots

This is a documnetation for running simulation and navigation of multible TB3s in the AWS small warehouse world.  
The outcome to have in the end is:
```
                            /map
/robot1/robot_state_publisher    /robot2/robot_state_publisher
/robot1/robot_pose_ekf           /robot2/robot_pose_ekf
/robot1/amcl                     /robot2/amcl
/robot1/move_base                /robot2/move_base
```

## Simulation

Launch file for one robot `one_robot.launch`
```xml
<launch>
    <arg name="robot_name"/>
    <arg name="init_pose"/>

    <node name="spawn_minibot_model" pkg="gazebo_ros" type="spawn_model"
     args="$(arg init_pose) -urdf -param /robot_description -model $(arg robot_name)"
     respawn="false" output="screen" />

    <node pkg="robot_state_publisher" type="robot_state_publisher" 
          name="robot_state_publisher" output="screen"/>

    <!-- The odometry estimator, throttling, fake laser etc. go here -->
    <!-- All the stuff as from usual robot launch file -->
</launch>
```

The main launch file for multiple robots `robots.launch`
```xml
<launch>
  <!-- No namespace here as we will share this description. 
       Access with slash at the beginning -->
  <param name="robot_description"
    command="$(find xacro)/xacro.py $(find turtlebot_description)/urdf/turtlebot.urdf.xacro" />

  <!-- BEGIN ROBOT 1-->
  <group ns="robot1">
    <param name="tf_prefix" value="robot1_tf" />
    <include file="$(find your_pkg)/launch/one_robot.launch" >
      <arg name="init_pose" value="-x 1 -y 1 -z 0" />
      <arg name="robot_name"  value="Robot1" />
    </include>
  </group>

  <!-- BEGIN ROBOT 2-->
  <group ns="robot2">
    <param name="tf_prefix" value="robot2_tf" />
    <include file="$(find your_pkg)/launch/one_robot.launch" >
      <arg name="init_pose" value="-x -1 -y 1 -z 0" />
      <arg name="robot_name"  value="Robot2" />
    </include>
  </group>
</launch>

```

simulation launch file `simulation.launch`
```xml
<launch>
  <param name="/use_sim_time" value="true" />

  <!-- start world -->
  <node name="gazebo" pkg="gazebo" type="gazebo" 
   args="$(find your_pkg)/worlds/your.world" respawn="false" output="screen" />

  <!-- start gui -->
  <node name="gazebo_gui" pkg="gazebo" type="gui" respawn="false" output="screen"/>

  <!-- include our robots -->
  <include file="$(find your_pkg)/launch/robots.launch"/>
</launch>

```

For more information, check the simulation part from [this link](https://answers.ros.org/question/41433/multiple-robots-simulation-and-navigation/?answer=41434#post-id-41434)


## Navigation

You need to setup amcl and move base file for each robot. 

- amcl file

```xml
<launch>
  <!-- Arguments -->
  <arg name="use_map_topic"  default="false"/>
  <arg name="scan_topic"     default="/robot1/scan"/>
  <arg name="initial_pose_x" default="1.0"/>
  <arg name="initial_pose_y" default="1.0"/>
  <arg name="initial_pose_a" default="0.0"/>
  <arg name="odom_frame_id" default="/robot1_tf/odom"/>
  <arg name="base_frame_id" default="/robot1_tf/base_footprint"/>
  <arg name="global_frame_id" default="/map"/>

  <!-- AMCL -->
  <node pkg="amcl" type="amcl" name="amcl_robot1">

    <param name="use_map_topic"             value="false"/>
    <param name="min_particles"             value="500"/>
    <param name="max_particles"             value="3000"/>
    <param name="kld_err"                   value="0.02"/>
    <param name="update_min_d"              value="0.20"/>
    <param name="update_min_a"              value="0.20"/>
    <param name="resample_interval"         value="1"/>
    <param name="transform_tolerance"       value="1.0"/>
    <param name="recovery_alpha_slow"       value="0.00"/>
    <param name="recovery_alpha_fast"       value="0.00"/>
    <param name="initial_pose_x"            value="$(arg initial_pose_x)"/>
    <param name="initial_pose_y"            value="$(arg initial_pose_y)"/>
    <param name="initial_pose_a"            value="$(arg initial_pose_a)"/>
    <param name="gui_publish_rate"          value="50.0"/>

    <!-- <param name="tf_prefix" value="robot1_tf"/> -->
    <!-- <remap from="scan"                      to="$(arg scan_topic)"/> -->
    <param name="laser_max_range"           value="3.5"/>
    <param name="laser_max_beams"           value="180"/>
    <param name="laser_z_hit"               value="0.5"/>
    <param name="laser_z_short"             value="0.05"/>
    <param name="laser_z_max"               value="0.05"/>
    <param name="laser_z_rand"              value="0.5"/>
    <param name="laser_sigma_hit"           value="0.2"/>
    <param name="laser_lambda_short"        value="0.1"/>
    <param name="laser_likelihood_max_dist" value="2.0"/>
    <param name="laser_model_type"          value="likelihood_field"/>

    <param name="odom_model_type"           value="diff"/>
    <param name="odom_alpha1"               value="0.1"/>
    <param name="odom_alpha2"               value="0.1"/>
    <param name="odom_alpha3"               value="0.1"/>
    <param name="odom_alpha4"               value="0.1"/>
    <param name="odom_frame_id"             value="$(arg odom_frame_id)"/>
    <param name="base_frame_id"             value="$(arg base_frame_id)"/>
    
    <param name="global_frame_id"           value="$(arg global_frame_id)"/>

    <remap from="scan"                      to="$(arg scan_topic)"/>
    <remap from="initialpose"               to="/robot1/initialpose"/>
    <remap from="amcl_pose"               to="/robot1/amcl_pose"/>
    <remap from="particlecloud"               to="/robot1/particlecloud"/>


  </node>
</launch>

```

You need to copy this file for each robot.

- Move base file

```xml

<launch>
  <!-- Arguments -->
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="cmd_vel_topic" default="cmd_vel" />
  
  <arg name="move_forward_only" default="false"/>

  <arg name="odom_frame_id" default="robot1_tf/odom"/>
  <arg name="base_frame_id" default="robot1_tf/base_footprint"/>
  <arg name="global_frame_id" default="map"/>
  <arg name="odom_topic" default="/robot1/odom" />
  <arg name="laser_topic" default="/robot1/scan" />
  
  <!-- move_base -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base_1" output="screen">
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/dwa_local_planner_params_$(arg model).yaml" command="load" />


    <param name="global_costmap/global_frame" value="$(arg global_frame_id)"/>
    <param name="global_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
    <param name="local_costmap/global_frame" value="$(arg odom_frame_id)"/>
    <param name="local_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
    <param name="DWAPlannerROS/global_frame_id" value="$(arg odom_frame_id)"/>
    <param name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)" />

    <remap from="cmd_vel"                                                          to="/robot1/cmd_vel"/>
    <remap from="odom"                                                             to="$(arg odom_topic)"/>
    <remap from="scan"                                                             to="$(arg laser_topic)"/>
    
    <remap from="map"                                                              to="/map" />
    <remap from="/move_base_simple/goal"                                           to="/robot1/move_base_simple/goal" />
    <remap from="/move_base/TebLocalPlannerROS/global_plan"                        to="/robot1/move_base/TebLocalPlannerROS/global_plan" />
    <remap from="/move_base/TebLocalPlannerROS/local_plan"                         to="/robot1/move_base/TebLocalPlannerROS/local_plan" />
    <remap from="/move_base/TebLocalPlannerROS/teb_markers"                        to="/robot1/move_base/TebLocalPlannerROS/teb_markers" />
    <remap from="/move_base/TebLocalPlannerROS/teb_markers_array"                  to="/robot1/move_base/TebLocalPlannerROS/teb_markers_array" />
    <remap from="/move_base/TebLocalPlannerROS/teb_poses"                          to="/robot1/move_base/TebLocalPlannerROS/teb_poses" />
    <remap from="/move_base/global_costmap/costmap"                                to="/robot1/move_base/global_costmap/costmap" />
    <remap from="/move_base/global_costmap/costmap_updates"                        to="/robot1/move_base/global_costmap/costmap_updates" />
    <remap from="/move_base/local_costmap/costmap"                                 to="/robot1/move_base/local_costmap/costmap" />
    <remap from="/move_base/local_costmap/costmap_updates"                         to="/robot1/move_base/local_costmap/costmap_updates" />
    <remap from="/move_base/local_costmap/footprint"                               to="/robot1/move_base/local_costmap/footprint" />
    
    <remap from="/move_base/GlobalPlanner/parameter_descriptions"                  to="/robot1/move_base/GlobalPlanner/parameter_descriptions" />
    <remap from="/move_base/GlobalPlanner/parameter_updates"                       to="/robot1/move_base/GlobalPlanner/parameter_updates" />
    <remap from="/move_base/GlobalPlanner/plan"                                    to="/robot1/move_base/GlobalPlanner/plan" />
    <remap from="/move_base/GlobalPlanner/potential"                               to="/robot1/move_base/GlobalPlanner/potential" />
    <remap from="/move_base/TebLocalPlannerROS/obstacles"                          to="/robot1/move_base/TebLocalPlannerROS/obstacles" />
    <remap from="/move_base/TebLocalPlannerROS/parameter_descriptions"             to="/robot1/move_base/TebLocalPlannerROS/parameter_descriptions" />
    <remap from="/move_base/TebLocalPlannerROS/parameter_updates"                  to="/robot1/move_base/TebLocalPlannerROS/parameter_updates" />
    <remap from="/move_base/cancel"                                                to="/robot1/move_base/cancel" />
    <remap from="/move_base/current_goal"                                          to="/robot1/move_base/current_goal" />
    <remap from="/move_base/feedback"                                              to="/robot1/move_base/feedback" />
    <remap from="/move_base/global_costmap/footprint"                              to="/robot1/move_base/global_costmap/footprint" />
    <remap from="/move_base/global_costmap/inflation_layer/parameter_descriptions" to="/robot1/move_base/global_costmap/inflation_layer/parameter_descriptions" />
    <remap from="/move_base/global_costmap/inflation_layer/parameter_updates"      to="/robot1/move_base/global_costmap/inflation_layer/parameter_updates" />
    <remap from="/move_base/global_costmap/obstacle_layer/clearing_endpoints"      to="/robot1/move_base/global_costmap/obstacle_layer/clearing_endpoints" />
    <remap from="/move_base/global_costmap/obstacle_layer/parameter_descriptions"  to="/robot1/move_base/global_costmap/obstacle_layer/parameter_descriptions" />
    <remap from="/move_base/global_costmap/obstacle_layer/parameter_updates"       to="/robot1/move_base/global_costmap/obstacle_layer/parameter_updates" />
    <remap from="/move_base/global_costmap/parameter_descriptions"                 to="/robot1/move_base/global_costmap/parameter_descriptions" />
    <remap from="/move_base/global_costmap/parameter_updates"                      to="/robot1/move_base/global_costmap/parameter_updates" />
    <remap from="/move_base/global_costmap/static_layer/parameter_descriptions"    to="/robot1/move_base/global_costmap/static_layer/parameter_descriptions" />
    <remap from="/move_base/global_costmap/static_layer/parameter_updates"         to="/robot1/move_base/global_costmap/static_layer/parameter_updates" />
    <remap from="/move_base/goal"                                                  to="/robot1/move_base/goal" />
    <remap from="/move_base/local_costmap/obstacle_layer/parameter_descriptions"   to="/robot1/move_base/local_costmap/obstacle_layer/parameter_descriptions" />
    <remap from="/move_base/local_costmap/obstacle_layer/parameter_updates"        to="/robot1/move_base/local_costmap/obstacle_layer/parameter_updates" />
    <remap from="/move_base/local_costmap/parameter_descriptions"                  to="/robot1/move_base/local_costmap/parameter_descriptions" />
    <remap from="/move_base/local_costmap/parameter_updates"                       to="/robot1/move_base/local_costmap/parameter_updates" />
    <remap from="/move_base/local_costmap/static_layer/parameter_descriptions"     to="/robot1/move_base/local_costmap/static_layer/parameter_descriptions" />
    <remap from="/move_base/local_costmap/static_layer/parameter_updates"          to="/robot1/move_base/local_costmap/static_layer/parameter_updates" />
    <remap from="/move_base/parameter_descriptions"                                to="/robot1/move_base/parameter_descriptions" />
    <remap from="/move_base/parameter_updates"                                     to="/robot1/move_base/parameter_updates" />
    <remap from="/move_base/result"                                                to="/robot1/move_base/result" />
    <remap from="/move_base/status"                                                to="/robot1/move_base/status" />
    <remap from="/move_base_simple/goal"                                           to="/robot1/move_base_simple/goal" />
  </node>
</launch>

```
you need to copy this file for each robot


- Include the file in main navigation file.
```xml
<launch>
  <!-- Arguments -->
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="map_file" default="$(find turtlebot3_navigation)/maps/map.yaml"/>
  <arg name="open_rviz" default="true"/>
  <arg name="move_forward_only" default="false"/>

  <!-- Map server -->
  <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)">
    <param name="frame_id" value="/map" />
  </node>

  <!-- AMCL -->
  <include file="$(find aws_warehouse_tb3)/launch/robot1_amcl.launch"/>
  <include file="$(find aws_warehouse_tb3)/launch/robot2_amcl.launch"/>

  <!-- Move Base -->
  <include file="$(find aws_warehouse_tb3)/launch/move_base_1.launch"/>
  <include file="$(find aws_warehouse_tb3)/launch/move_base_2.launch"/>

  <!-- rviz -->
  <group if="$(arg open_rviz)"> 
    <node pkg="rviz" type="rviz" name="rviz" required="true"
          args="-d $(find aws_warehouse_tb3)/rviz/multi_turtlebot3_navigation.rviz"/>
  </group>
</launch>

```

For more information, check [this video](https://www.youtube.com/watch?v=es_rQmlgndQ)
 