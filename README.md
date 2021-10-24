# 					**Smart Wheelchair**

#### What is smart wheelchair?

* This project aims to implement a convolutional neural network (CNN), which will work in wheelchair navigation, in order to correct errors, avoid collisions and provide an improvement for wheelchair users.

#### Considerations:

* The project was created by Prof. Brenna Argall and Dr. Jarvis Schultz. https://github.com/patilnabhi/nuric_wheelchair_model_02
* The GIPAR group (Innovation, Research and Robotic Automation Group), of the IFBA (Federal Institute of Bahia), chose to choose and continue this project for academic research purposes in the field of robotics automation.

#### Project Objectives:

1. Model a church, which will be the scenario of the simulations

   1.1. We will use Gazebo 7,
   SketchUp, Blender and the Phobos framework for building the 3D model of the church

2. Implement covolutional neural network for integrated navigation in python

   2.1. Implement covolutional neural network for integrated navigation in python

   2.2. We'll use Visual Studio Code for work

#### Modifications:

After the installation of Ubuntu 16.04, ROS Kinetic and Gazebo 7:

1. **Some dependencies were installed for ROS and Gazebo:**

   ```bash
   $ sudo apt-get update && sudo apt-get upgrade
   $ sudo apt-get install ros-kinetic-teleop-twist-keyboard
   $ sudo apt-get install ros-kinetic-slam-gmapping
   $ sudo apt-get install ros*controller*
   $ sudo apt install ros-kinetic-gazebo-ros
   $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs
   $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
   $ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
   $ chmod 755 ./install_ros_kinetic.sh
   $ bash ./install_ros_kinetic.sh
   $ sudo apt-get install ros-kinetic-joy
   $ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy \
     ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc \
     ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan \
     ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python \
     ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
     ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server \
     ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
     ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view \
     ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
   
   $ source /opt/ros/kinetic/setup.bash`
   ```

2. **The wheelchair files was downloaded:**

   ```bash
   $ cd catkin_ws/src
   $ git clone https://github.com/HeliNeres/nuric_wheelchair_model_02.git
   ```
   
3. **The 3D models and world files were downloaded as follows:**

   Some 3D models and worlds were made to test the wheelchair.

   1. At first we should go to the personal folder and press Ctrl+H, to show the hidenn folders.

   3. Hidden folders will have a dot in front of the folder name, go into the .gazebo/models folder and paste all the downloaded and extracted models (https://github.com/osrf/gazebo_models). These models are official gazebo but the first time we open the gazebo they are not loaded, they are downloaded slowly in the background, we do this to resolve this issue.

   3. In the same folder mentioned above (.gazebo/models), we will put this downloaded and extracted  models (https://drive.google.com/drive/folders/1eCY0whsAKlkAV1njhwJ4ldrsXNSLktvn?usp=sharing). After that, the folder should contain the standart gazebo models and ne new ones created by the GIPAR students.

   4. To test the models created by the GIPAR students we should open Gazebo at fist.

   5. When open Gazebo go to the insert tab, there is a list of 3D objects, look for Wheelchair_blue_V2. By clicking on this name you will notice that the computer will darken the screen and it will take a while to load, after a while a wheelchair will appear blue, you can also test other 3D objects like 'Instruments', among others on the list but some 3D models have many polygons, are big and heavy, demanding a lot from the computer.

   6. Now we will download the worlds modeled with these 3D objects mentioned above (https://drive.google.com/drive/folders/1lyt8zwWRCWPFYftUvpOsjq1Ecszys3uI?usp=sharing). After extracting, put them in the nuric_wheelchair_model_02/world folder.

   7. To test the worlds, right click on some blank part of the words folder and choose 'open in terminal', and type:

      `$ gazebo church_V2.world`

      Wait between 3 to 10 minutes for the gazebo to open the world church_V2.world.



4. **To use the chair in the mentioned environments, some adjustments were made:**

   The adjustments below were made to load the wheelchair in one of the downloaded worlds and to do the mapping of the environment.
   1. **The world launch file has been changed**

      ```bash
      $ cd catkin
      $ catkin_make
      $ cd src/nuric_wheelchair_model_02/launch 
      $ gedit empty_world.launch
      ```
      
      1. **The excerpt below has been removed**

      ```xml
      <arg name="world_name" default="worlds/empty.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
      ```
      
      2. **And replaced by the following excerpt**

      ```xml
      <arg name="world_name" default="$(find nuric_wheelchair_model_02)/worlds/house.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
      ```
      
   2. **The wheelchair launch file has been changed**

      ```bash
      $ cd catkin
      $ catkin_make
      $ cd src/nuric_wheelchair_model_02/launch
      $ gedit wheelchair.launch
      ```
      
      1. **The excerpt below has been removed**
   
         ```xml
         <?xml version="1.0"?>
         
         <launch>
         
             <include file="$(find nuric_wheelchair_model_02)/launch/empty_world.launch" >
                 <arg name="paused" value="false"/>
                 <arg name="gui" value="true"/>
                 <arg name="headless" value="false"/>
         
                 <arg name="world_name" value="worlds/empty.world" />
             </include>
         
             <include file="$(find nuric_wheelchair_model_02)/launch/wheelchair_base.launch" />
         
         	<!-- joy node(s) -->
           	<node respawn="true" pkg="joy" type="joy_node" name="wheelchair_joy" />
           	<node respawn="true" pkg="nuric_wheelchair_model_02" type="joy_teleop.py" name="joy_teleop" />
         
         	<!-- get_caster_joints node -->
         	<node respawn="true" pkg="nuric_wheelchair_model_02" type="get_caster_joints.py" name="get_caster_joints" />
         
         </launch>
         ```
      
      2. **And replaced by the following excerpt**
   
         ```xml
         <?xml version="1.0"?>
         
         <launch>
         
             <!--1ª add for slam -->
             <arg name="paused" value="false"/>
             <arg name="use_sim_time" value="true"/>
             <arg name="gui" value="true"/>
             <arg name="recording" value="false"/>
             <arg name="headless" default="false"/>
             <arg name="debug" value="false"/>
         
             <param name="use_sim_time" value="true"/>
         
             <param name="frame_id" type="string" value="laser_joint"/>
          <arg name="world_name" value="worlds/empty.world" />
             <!-- the end 1ª step-->
         
         <include file="$(find nuric_wheelchair_model_02)/launch/empty_world.launch" >
         	<!--2ª add for slam -->
         	 <arg name="debug" value="$(arg debug)" />
          	<!--the end 2ª step -->
                 <arg name="paused" value="false"/>
                 <arg name="gui" value="true"/>
                 <arg name="headless" value="false"/>
         
                 <!--<arg name="world_name" value="worlds/empty.world" />-->
             </include>
         
             <include file="$(find nuric_wheelchair_model_02)/launch/wheelchair_base.launch" />
         
         	<!-- joy node(s) -->
           	<node respawn="true" pkg="joy" type="joy_node" name="wheelchair_joy" />
           	<node respawn="true" pkg="nuric_wheelchair_model_02" type="joy_teleop.py" name="joy_teleop" />
         
         	<!-- get_caster_joints node -->
         	<node respawn="true" pkg="nuric_wheelchair_model_02" type="get_caster_joints.py" name="get_caster_joints" />
         
         
         
          <!--3ª add for slam -->
         
         <param name="base_frame" value="base_footprint"/>
             <param name="odom_frame" value="odom"/>
             <param name="map_update_interval" value="5.0"/>
             <param name="maxUrange" value="6.0"/>
             <param name="maxRange" value="8.0"/>
             <param name="sigma" value="0.05"/>
             <param name="kernelSize" value="1"/>
             <param name="lstep" value="0.05"/>
             <param name="astep" value="0.05"/>
             <param name="iterations" value="5"/>
             <param name="lsigma" value="0.075"/>
             <param name="ogain" value="3.0"/>
             <param name="lskip" value="0"/>
             <param name="minimumScore" value="100"/>
             <param name="srr" value="0.01"/>
             <param name="srt" value="0.02"/>
             <param name="str" value="0.01"/>
             <param name="stt" value="0.02"/>
             <param name="linearUpdate" value="0.5"/>
             <param name="angularUpdate" value="0.436"/>
             <param name="temporalUpdate" value="-1.0"/>
             <param name="resampleThreshold" value="0.5"/>
             <param name="particles" value="80"/>
           <!--
             <param name="xmin" value="-50.0"/>
             <param name="ymin" value="-50.0"/>
             <param name="xmax" value="50.0"/>
             <param name="ymax" value="50.0"/>
           make the starting size small for the benefit of the Android client's memory...
           -->
             <param name="xmin" value="-1.0"/>
             <param name="ymin" value="-1.0"/>
             <param name="xmax" value="1.0"/>
             <param name="ymax" value="1.0"/>
         
             <param name="delta" value="0.05"/>
             <param name="llsamplerange" value="0.01"/>
             <param name="llsamplestep" value="0.01"/>
             <param name="lasamplerange" value="0.005"/>
             <param name="lasamplestep" value="0.005"/>
         
             <node pkg="rosbag" type="play" name="rosbag" args="--clock -r 5"/>
         <!--the end 3ª step -->
         
         </launch>
         ```
      
   3. **The mapping launch file has been changed**
   
      ```bash
      $ cd catkin
      $ catkin_make
      $ cd src/nuric_wheelchair_model_02/launch
      $ gedit gmapping.launch
      ```
      
      1. **The code bellow has been used in the file**
   
         ```xml
         <?xml version="1.0"?>
         <launch>
           <arg name="scan_topic"  default="/scan" />
           <arg name="base_frame"  default="base_footprint"/>
           <arg name="odom_frame"  default="/odom"/>
         
           <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"></node>
         
           <node pkg="rviz" type="rviz" name="rviz"></node>
         
           <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
             <param name="base_frame" value="$(arg base_frame)"/>
             <param name="odom_frame" value="$(arg odom_frame)"/>
             <param name="map_update_interval" value="5.0"/>
             <param name="maxUrange" value="6.0"/>
             <param name="maxRange" value="8.0"/>
             <param name="sigma" value="0.05"/>
             <param name="kernelSize" value="1"/>
             <param name="lstep" value="0.05"/>
             <param name="astep" value="0.05"/>
             <param name="iterations" value="5"/>
             <param name="lsigma" value="0.075"/>
             <param name="ogain" value="3.0"/>
             <param name="lskip" value="0"/>
             <param name="minimumScore" value="200"/>
             <param name="srr" value="0.01"/>
             <param name="srt" value="0.02"/>
             <param name="str" value="0.01"/>
             <param name="stt" value="0.02"/>
             <param name="linearUpdate" value="0.5"/>
             <param name="angularUpdate" value="0.436"/>
             <param name="temporalUpdate" value="-1.0"/>
             <param name="resampleThreshold" value="0.5"/>
             <param name="particles" value="80"/>
           <!--
             <param name="xmin" value="-50.0"/>
             <param name="ymin" value="-50.0"/>
             <param name="xmax" value="50.0"/>
             <param name="ymax" value="50.0"/>
           make the starting size small for the benefit of the Android client's memory...
           -->
             <param name="xmin" value="-1.0"/>
             <param name="ymin" value="-1.0"/>
             <param name="xmax" value="1.0"/>
             <param name="ymax" value="1.0"/>
         
             <param name="delta" value="0.05"/>
             <param name="llsamplerange" value="0.01"/>
             <param name="llsamplestep" value="0.01"/>
             <param name="lasamplerange" value="0.005"/>
             <param name="lasamplestep" value="0.005"/>
             <remap from="scan" to="$(arg scan_topic)"/>
           </node>
         </launch>
         ```
      
   4. **The navigation files has been created**
   
      ```bash
      $ cd catkin_ws/src/nuric_wheelchair_model_02
      $ mkdir navigation_wheelchair
      $ cd navigation_wheelchair
      $ gedit navigation_wheelchair.launch
      ```
      
      1. **The code bellow has been used in the file**
   
         ```xml
         <?xml version="1.0"?>
         <launch>
           <!-- Arguments -->
           <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
         <arg name="map_file" default="$(find nuric_wheelchair_model_02)/maps/map.yaml"/>
           <arg name="open_rviz" default="true"/>
           <arg name="move_forward_only" default="false"/>
         
         <!-- wheelchair -->
         
         	<arg name="rvizconfig" default="$(find nuric_wheelchair_model_02)/config/wheelchair_urdf.rviz" />
         
         	<!-- push robot_description to factory and spawn robot in gazebo -->
           <node name="spawn_robot" pkg="gazebo_ros" type="spawn_model"
                   args="-param robot_description
                         -unpause
                         -urdf
                         -model robot_description"
                   respawn="false" output="screen" />
         
         	<!-- start robot state publisher -->
           	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" respawn="false" output="screen" />
         
         
         	<param name="use_gui" value="false"/>
           <!-- Map server -->
           <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)"/>
         
           <!-- AMCL -->
           <include file="$(find nuric_wheelchair_model_02)/launch/amcl.launch"/>
         
           <!-- move_base -->
           <include file="$(find nuric_wheelchair_model_02)/launch/move_base.launch">
             <arg name="model" value="$(arg model)" />
             <arg name="move_forward_only" value="$(arg move_forward_only)"/>
           </include>
         
           <!-- rviz -->
           <group if="$(arg open_rviz)"> 
             <node pkg="rviz" type="rviz" name="rviz" required="true"
                   args="-d $(find nuric_wheelchair_model_02)/rviz/wheelchair_navigation.rviz"/>
           </group>
         </launch>
         ```
   
5. **Launching the simulation**


   1. **Launch the wheelchair with in the determined world**

      ```bash
      $ cd catkin_ws
      $ cd src
      $ cd nuric_wheelchair_model_02
      $ cd launch
      $ roslaunch wheelchair.launch
      ```

   2. **To execute the mapping launch the gmapping file in parallel**

      ```bash
      $ roslaunch gmapping.launch
      ```


#### Documentation Overview:

* This documentation explains the code structure and address the following 2 topics -

#####A. 3D model of new wheelchair

![3D model](http://abhipatil.me/wp-content/uploads/2016/12/wheelchair_01-420x488.png)

* The relevant files are present in 2 main directories, namely  [`urdf`] and [`meshes`]

* `urdf` : This directory contains the `xacro` files required to build the 3D model in simulation.
* Main highlights -
	* `joint_states` are published using the `gazebo_ros_control` plugin (particularly, `libgazebo_ros_joint_state_publisher.so` plugin)
	* The differential drive controller uses the `libgazebo_ros_diff_drive.so` plugin
	* The hokuyo laser controller uses `libgazebo_ros_laser.so` plugin to gather laser-scan data
	* The kinect camera controller uses `libgazebo_ros_openni_kinect.so` plugin to generate `rgb` and `depth` data

* `meshes` directory contain the collada `.dae` files of the wheelchair

* Raw SolidWorks files `.SLDPRT & .SLDASM` are available in the [`3d_model_sw`] directory
	* [SimLab Composer] software is used to convert the `.SLDPRT & .SLDASM` files into collada `.dae` files for URDF compatibility
	* [MeshLab] software is used to determine the moments of inertia and center of gravity parameters of the wheelchair


#####B. UKF implementation for estimation of CWOs

* The UKF algorithm implementation consists of 3 main steps, as outlined below –

	(a) **Initialize:**

	* Initialize state and controls for the wheelchair (mean and covariance)

	(b) **Predict:**

	* Generate sigma points using [Julier’s Scaled Sigma Point] algorithm
	* Pass each sigma points through the dynamic motion model to form a new prior
	* Determine mean and covariance of new prior through unscented transform

	(c) **Update:**

	* Get odometry data (measurement of pose of wheelchair)
	* Convert the sigma points of prior into expected measurements (points corresponding to pose of wheelchair – x, y  and theta  are chosen)
	* Compute mean and covariance of converted sigma points through unscented transform
	* Compute residual and Kalman gain
	* Determine new estimate for the state with new covariance


* The UKF code (Python) is produced below (Click on functions to look at its complete implementation): 

	* Frist, we create a function to represent dynamic motion model `f(x)` and the measurement function `h(x)`
	* The dynamic motion model is implemented using 4th-order Runge-Kutta method ([ode2], [rK7])
	* Measurement data for this implementation comes from wheelchair's odometry - hence, the measurement function returns the 3rd, 4th and 5th elements representing x, y and theta (pose of wheelchair)
	

		```
		import numpy as np
		
		def fx(x, dt):	
			sol = ode2(x)
			return np.array(sol)
		
		def hx(x):
			return np.array([x[3], x[2], normalize_angle(x[4])])
		
		```

	* Next, we create sigma points using the Julier Scaled Sigma Point algorithm. ([JulierSigmaPoints])


		```
		points = JulierSigmaPoints(n=7, kappa=-4., sqrt_method=None)
		```
	
	* The [`UKF`] class incorporates the UKF algorithm as follows -


		```
		class UKF(object):
	
		    def __init__(self, dim_x, dim_z, dt, hx, fx, points, sqrt_fn=None, 
		    				x_mean_fn=None, z_mean_fn=None, residual_z=None, residual_z=None):    
		``` 
	
	* [`predict`] function passes each of the sigma points through `fx` and calculate new set of sigma points
	* The mean (x) and covariance (P) are obtained via unscented transform as shown below -
	
		```
		    def predict(self, UT=None, fx_args=()):
	
		        dt = self._dt
	
		        if not isinstance(fx_args, tuple):
		            fx_args = (fx_args,)
	
		        if UT is None:
		            UT = unscented_transform
	
		        sigmas = self.points_fn.sigma_points(self.x, self.P)
	
		        for i in xrange(self._num_sigmas):
		            self.sigmas_f[i] = self.fx(sigmas[i], dt, *fx_args)
		        
		        self.x, self.P = UT(self.sigmas_f, self.Wm, self.Wc, self.Q, self.x_mean, self.residual_x)
		        # print self.x


	        def unscented_transform(sigmas, Wm, Wc, noise_cov=None, mean_fn=None, residual_fn=None):
	
			    kmax, n = sigmas.shape
	
			    x = mean_fn(sigmas, Wm)
	
			    P = np.zeros((n,n))
			    for k in xrange(kmax):
			        y = residual_fn(sigmas[k], x)
			        P += Wc[k] * np.outer(y, y)
	
			    if noise_cov is not None:
			        P += noise_cov
	
			    return (x, P)
	    ```
	
	    * The [`update`] function first generates sigma points from expected measurement data
	    * The measurement mean (zp) and covariance (Pz) is obtained via unscented transform of the above generated sigma points
	    * Next, the Kalman gain (K) and residual gain (y) is calculated 
	    * Finally, the new mean (x) and covariance (P) is obtained, given K and y
	
	    ```
		    def update(self, z, R=None, UT=None, hx_args=()):
	
		        if z is None:
		            return
	
		        if not isinstance(hx_args, tuple):
		            hx_args = (hx_args,)
	
		        if UT is None:
		            UT = unscented_transform
	
		        R = self.R
	
		        for i in xrange(self._num_sigmas):
		            self.sigmas_h[i] = self.hx(self.sigmas_f[i], *hx_args)
	
		        zp, Pz = UT(self.sigmas_h, self.Wm, self.Wc, R, self.z_mean, self.residual_z)
	
		        Pxz = zeros((self._dim_x, self._dim_z))
		        for i in xrange(self._num_sigmas):
		            dx = self.residual_x(self.sigmas_f[i], self.x)
		            dz = self.residual_z(self.sigmas_h[i], zp)
		            Pxz += self.Wc[i] * outer(dx, dz)


		        self.K = dot(Pxz, inv(Pz))
		        self.y = self.residual_z(z, zp)
	
		        self.x = self.x + dot(self.K, self.y)
		        self.P = self.P - dot3(self.K, Pz, self.K.T)
		```
	
	* The above functions from UKF class are imported in the main file [`ukf_wheelchair.py`] and implemented as follows -
	
		```
		kf = UKF(dim_x=7, dim_z=3, dt, fx, hx, points, 
					sqrt_fn=None, x_mean_fn=state_mean, z_mean_fn=meas_mean, 
					residual_x, residual_z)
	
		x0 = np.array(self.ini_val)
	
		kf.x = x0
		kf.Q *= np.diag([.0001, .0001, .0001, .0001, .0001, .01, .01])
		kf.P *= 0.000001
		kf.R *= 0.0001
	
		move_time = 4.0
		start = rospy.get_time()
	
		while (rospy.get_time() - start < move_time) and not rospy.is_shutdown():	
			pub_twist.publish(wheel_cmd)
	
			z = np.array([odom_x, odom_y, odom_theta])
			zs.append(z)
	
			kf.predict()
			kf.update(z)
	
			xs.append(kf.x)
		```


####References:

1. [Kalman and Bayesian Filters in Python](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=3&cad=rja&uact=8&ved=0ahUKEwjGrb6QsObQAhVHw2MKHdvLBsMQFggmMAI&url=http%3A%2F%2Frobotics.itee.uq.edu.au%2F~elec3004%2Ftutes%2FKalman_and_Bayesian_Filters_in_Python.pdf&usg=AFQjCNEOmFJeO0npgom8AT2vrbTJgrvDaA)

2. Analysis of Driving Backward in an Electric-Powered Wheelchair, *Dan Ding, Rory A. Cooper, Songfeng Guo and Thomas A. Corfman (2004)*

3. A New Dynamic Model of the Wheelchair Propulsion on Straight and Curvilinear Level-ground Paths, *Felix Chenier, Pascal Bigras, Rachid Aissaoui (2014)*

4. A Caster Wheel Controller For Differential Drive Wheelchairs, *Bernd Gersdorf, Shi Hui*

5. Kinematic Modeling of Mobile Robots by Transfer Method of Augmented Generalized Coordinates,
*Wheekuk Kim, Byung-Ju Yi, Dong Jin Lim (2004)*

6. [Mobile Robot Kinematics](http://www.cs.cmu.edu/~rasc/Download/AMRobots3.pdf)

7. Dynamics equations of a mobile robot provided with caster wheel, *Stefan Staicu (2009)*





[robotic wheelchair]:http://argallab.smpp.northwestern.edu/index.php/research/robot-platforms/smart-wheelchair/
[Prof. Brenna Argall]:http://users.eecs.northwestern.edu/~argall/
[assistive & rehabilitation robotics laboratory (argallab)]:http://argallab.smpp.northwestern.edu/
[Rehabilitation Institute of Chicago (RIC)]:http://www.ric.org/
[`urdf`]:https://github.com/patilnabhi/nuric_wheelchair_model_02/tree/master/urdf
[`meshes`]:https://github.com/patilnabhi/nuric_wheelchair_model_02/tree/master/meshes
[`3d_model_sw`]:https://github.com/patilnabhi/nuric_wheelchair_model_02/tree/master/3d_model_sw
[SimLab Composer]:http://www.simlab-soft.com/3d-products/simlab-composer-main.aspx
[Julier’s Scaled Sigma Point]:http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1025369
[ode2]:https://github.com/patilnabhi/nuric_wheelchair_model_02/blob/master/src/ukf_wheelchair.py#L192-L227
[rK7]:https://github.com/patilnabhi/nuric_wheelchair_model_02/blob/master/src/ukf_helper.py#L109-L175
[JulierSigmaPoints]:https://github.com/patilnabhi/nuric_wheelchair_model_02/blob/master/src/ukf_helper.py#L202-L332
[`UKF`]:https://github.com/patilnabhi/nuric_wheelchair_model_02/blob/master/src/ukf.py#L10-L101
[`predict`]:https://github.com/patilnabhi/nuric_wheelchair_model_02/blob/master/src/ukf.py#L53-L68
[`update`]:https://github.com/patilnabhi/nuric_wheelchair_model_02/blob/master/src/ukf.py#L72-L101
[`ukf_wheelchair.py`]:https://github.com/patilnabhi/nuric_wheelchair_model_02/blob/master/src/ukf_wheelchair.py
[MeshLab]:http://meshlab.sourceforge.net/
[Dr. Jarvis Schultz]:http://nxr.northwestern.edu/people/jarvis-schultz
