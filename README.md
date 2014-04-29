ISIRTaskManager
===============

Module to manage tasks for ISIRController:
Create/update tasks from XML files.

Requirements:
=============

* tinyxml
* ISIRController

XML Specifications:
===================

tasks:
------
1. <tasks> :
     Lists of tasks.
2. Attributes
 * None
3. Elements
 * <task>

task:
-----
1. <task> :
     Task description.
2. Attributes
 * id (required) :
     The task identifier. It will be used to retrieve/update it.
 * type (required) :
     The type of the task, it can be ACCELERATION, TORQUE, FORCE
 * active (required) :
     1 if the task is active, else 0
3. Elements
 * <param> (required) :
     Parameters of the task
 * <feature> (required) :
     The feature of the task

param:
------
1. <param> :
    Task parameters
2. Attributes
 * w (required) :
     The weight of the task
 * kp (required) : 
     The stiffness of the task
 * kd (required) :
     The damping of the task
3. Elements
 * None

feature:
--------
1. <feature> :
    The feature of the task.
2. Attributes 
 * type (required) :
     The type of the feature, can be fullstate, partialstate, frame, com, contact
3. Elements
 * <part> (required for fullstate, partialstate, frame):
     This elements specifies if the free flyer is considered.
 * <dofs> (required for partialstate, frame, com) :
     The selected degrees of freedom.
 * <objective> (required for fullstate, partialstate, frame, com) :
     The desired state of the task
 * <segment> (required for contact) :
     The name of the segment which is in contact.
 * <local_offset> (required for contact) :
     The offset of the contact point relative to the frame of the segment.
 * <mu> (required for contact) :
     Friction cone coefficient
 * <margin> (required for contact) :
     Margin

part:
-----
1. <part> :
    This elements specifies if the free flyer is considered.
2. Attributes
 * value (required) :
     INTERNAL, FREE_FLYER, FULL_STATE
3. Elements
 * None

dofs:
-----
1. <dofs> : 
    The selected degrees of freedom.
2. Attributes
 * value (required) :
     For partialstate, it is the list of the degrees of freedom index. For frame tasks, it is
     RXYZ, XYZ, R, RX, XY...
3. Elements
 * None

objective:
----------
1. <objective> :
    The desired state of the task 
2. Attributes
 * None
3. Elements
 * <q_des> (for fullstate, partialstate) :
     Desired joint angles ex : value="0.1 0.2 0.3"
 * <qd_des> (for fullstate, partialstate) :
     Desired joint angles velocity
 * <qdd_des> (for fullstate, partialstate) :
     Desired joint angles acceleration
 * <tau_des> (for fullstate, partialstate) :
     Desired joint torques
 * <pos_des> (for frame) :
     Desired position
 * <vel_des> (for frame) :
     Desired velocity
 * <acc_des> (for frame) :
     Desired acceleration

q_des, qd_des, qdd_des, tau_des, mu, margin:
--------------------------------------------
1. <q_des> :
    Desired joint angles
2. Attribute
 * value (required) : 
     the desired value
3. Elements
 * None

pos_des, vel_des, acc_des:
--------------------------
1. <pos_des> :
    Desired position
2. Attributes
 * xyz : 
     the desired translation
 * rpy : 
     the desired roll pitch yaw
3. Elements
 * None

segment:
--------
1. <segment>
     The segment involved in the contact
2. Attributes
 * name (required) :
     the name of the segment.
3. Elements
 * None

