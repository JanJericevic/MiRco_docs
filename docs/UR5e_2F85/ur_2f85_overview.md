# UR5e & Robotiq 2F-85 overview

 <div class="row">
  <div class="column">
   <img src="../img/ur5e.png" alt="UR5e robot" width="400"/>
  </div>
  <div class="column">
    <img src="../img/2f85.png" alt="Robotiq gripper" width="400"/>
  </div>
</div> 

**UR5e** is a collaborative industrial robotic manipulator from Universal Robots (UR). The manipulator weighs 20 kg, has 6 degrees of freedom (DOF), a load capacity of 5 kg and a working area with a radius of 850 mm. The manipulator has force and obstacle detection capabilities and includes various configurable safety features. 

Application design is done via the Polyscope software package on a touch-screen teach pendant.  

**2f85** is Robotiq's adaptive robotic gripper for use with collaborative robots. The gripper has adjustable gripping stroke from 0 to 85 mm with a resolution of 0.4 mm. It is capable of parallel gripping with a force of 20-235 N and a speed of 20-150 mm/s. The gripper is connected to the manipulator on the flange, with communication via the Modbus RTU protocol.

**This documentation describes only the ROS control** of the manipulator with the gripper. For control and application design using the teach pendant see their original documentation.