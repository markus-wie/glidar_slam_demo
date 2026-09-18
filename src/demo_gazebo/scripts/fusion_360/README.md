# Post Processing

This directory contains scripts for post-processing generated sdf models from a Fusion 360 design. This is strictly refferring to the "FusionSDF" plugin for Fusion 360. Other plugins act differently and may not be compatible with these scripts.

## Necessary Preperations

The great advantage to using Fusion for designing your robot is that you can greatly simplify the process of creating a sdf model. You can just create your design, assemble it using joints, and then export it as a sdf model.
During that process, the mass and inertia of the links are automatically exported, since they are being calculated by Fusion. You can also specify the material properties and appearance of the links, which will alter the visual and inertial properties of your design.

It is of great importance to keep the Fusion assembly in a strict "top-down" topology. You are always to attach "free-floating" elements as children to your design. You cannot make topology random!
Example:

It is correct to do:
```
base_link -> link_1 -> link_2 -> link_3
```

But not to do:
```
base_link -> link_1 -> link_2 <- link_3
```

It is important to make sure this topology is kept in your design. When using the Joint tool in Fusion, you are first selecting the child link and then the parent link. Think of moving the child to the parent, therefore selecting the child first.


### Loops

If you want to include loops, to model closed loop kinematic, you also have to be very careful to keep the topology in a "top-down" manner, where the joints joining the loop are both the parents of the loop-joining link. Visually the hierarchy should look something like the following:
```
            -> link_1 -> link_2 ->
          /                        \
base_link                           joining_link
          \                        /
            -> link_3 -> link_4 ->
```

This will be correctly exported by the FusionSDF plugin, but Gazebo/SDFormat will not be able to spawn the model, since it does not support direct loops. That doesnt mean you cannot simulate loops. Gazebo is able to that perfectly fine. 
But you need to apply a little trick: The model must look like a perfect tree structure without any loops when spawning. After the spawn, however, you can attach the loop-joining links again. This can be achieved with the "DetachableJoint" plugin. You can use it to rigidly attach two loop joining links together after the spawn. Please look at examples or the documentation of the "DetachableJoint" plugin for more information on syntax. The general idea, however, is presented in the following:

### DetachableJoint plugin for loops
DetachableJoints are rigid joints that attach **after** the model has been spawned. Since these joints are strictly rigid, we need to alter the topology of the our model a little bit, if they primarily consist of non-rigid joints. The idea is to create at the loop joining joint a "dummy link" that models the kind of joint that is needed at that joint. Then this dummy is used as a surrogate for the loop-joining link in addition with the DetachableJoint. Look at the following example from before with added characters defining what kind of joint is used. 'r' stands for a revolute joint, 'f' for a fixed joint:

```
            -> link_1 ->
          r/            \r
base_link                 joining_link
          r\            /r
            -> link_3 ->
```

To close the loop using the DetachableJoint plugin, we need to create a dummy link at any of the four revolute joints. 

```
            -> link_1 ->r dummy_link ->
          r/                           \DJ
base_link                                joining_link
          r\                           /r
            ->          link_3       ->
```

Notice where the revolute joint is placed and where the DetachableJoint (DJ) is placed. The DJ will be a fixed/rigid joint as soon as the model is spawned. Before that, the SDF will look like it has no loops, since the DJ is not actually a joint, but just a plugin, that creates a joint at runtime.

Some resources on this topic are:
 - https://github.com/gazebosim/gz-physics/issues/719
 - https://github.com/srmainwaring/ros2_closed_loop_demo/tree/wips/wip-update-physics
 - https://robotics.stackexchange.com/questions/115457/stuck-trying-to-use-a-detachable-joint-to-simulate-closed-loop-system-in-gazebo
 - https://github.com/gazebosim/gz-physics/issues/25
 - https://gazebosim.org/api/gazebo/6/detachablejoints.html

#### Physics solver

In order to simulate loops, you need to make sure that the physics solver uses the solver_type "pgs" (Projected Gauss-Seidel).
One example of how you can set up the physics settings in your world file is the following:
```xml
<physics name="1ms" type="ignore">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate> 
    <dart>
    <solver>
        <solver_type>pgs</solver_type> <!-- Must be pgs. pgs is needed for closed loop -->
    </solver>
    <collision_detector>ode</collision_detector>
    </dart>
    <ode>
    <solver>
        <type>quick</type>
        <iters>50</iters>
        <sor>1.4</sor>
    </solver>
    <constraints>
        <cfm>0</cfm>
        <erp>1</erp>
        <contact_max_correcting_vel>0</contact_max_correcting_vel>
        <contact_surface_layer>0</contact_surface_layer>
    </constraints>
    </ode>

</physics>
```

## Scripts

In this directory there are two scripts that can help you with making an SDF export using FusionSDF ready for simulation in Gazebo. 
There are a couple of issues that need to be solved:
 - The sdf file will most likely contain countless collisions that are not needed for simulation. We only need certain kinds of collisions, e.g. for the wheels. In that case, we **cannot** use stl files or anything else, but should strictly use the provided primitives by SDFormat. The wheels for example can be modeled as spheres or cylinders.
 - If you have loops in your design, they must be unrolled and replaced with DetachableJoints
 - Some Fusion plugins (e.g. FusionSDF) interpret revolute joints as "continuous" joints in SDFormat. This stems from revolute joints mean to have a certain range of motion. For an unlimited range of motion, the "continuous" joint type can be used. However, in the current version of Gazebo and SDFormat, the "continuous" joint type is not supported. Therefore, we need to replace all "continuous" joints with "revolute" joints and set the limits to be infinite. The limits are infinite by default.


### Remove Collisions
The script remove_collisions.py works directly in the XML file and removes all collision tags.

Collisions must then be added either directly in the sdf file or where the file is imported using the <experimental:params> tag from sdf.

Run this script first.

```bash
python remove_collisions.py ../../models/car/fusion_export/model.sdf ../../models/car/fusion_export/model.sdf
```

### Simplify Meshes
The script simplify_meshes.py searches for repeated meshes and replaces them with a single common mesh.
FusionSDF exports a separate mesh for each link, even if the same mesh is used multiple times.

Also replaces continuous joints with revolute joints.

This script will also remove all the unnecessary meshes, but it will ask you before it deletes anything.

Run this script second.

```bash
python simplify_model.py ../../models/car/fusion_export/model.sdf ../../models/car/fusion_export/model.sdf ../../models/car/fusion_export/meshes/
```

### Unroll Loops
The script unroll_loops.py works using the sdformat python bindings. It detects loops in the model and unrolls them by creating dummy links and replacing the loop-joining joints with DetachableJoints.
It is important to comply with the above discussed topology. After running the script, the sdf file should be ready for closed loop simulation.

It is not recommended to use this script, since it completely rewrites the file and adds all sdf parameters with their default values. This will increase the size of this file by a decent amount.
What this file does, can be easily done by hand as well.
