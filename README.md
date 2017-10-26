## Folding Deformable Objects using Predictive Simulation and Trajectory Optimization (IROS 2015)

This project was implemented using both Linux (Ubuntu) and Windows machines. 
The Ubuntu is for robot control using ROS, and the Windows machine is for folding plan generation. 
It is possible to move the part on the Windows machine to Ubuntu with some efforts.

A video demo of the project is [here](http://www.cs.columbia.edu/~yli/IROS2015/index.html).

## Baxter Robot

- To start, a [MoveIt](http://moveit.ros.org/install/) package and the [ROS](http://www.ros.org/) platform are required. In our project, we used the Hydro version, and we have not tested on other versions of the ROS.

- A ROS launch file can be found in `/src/folding_control/launch`.

- A communication module for passing data between the Linux and the Windows machines is included in `src/folding_control/src/xmlrpclib-1.0.1`. 
The IP addresses of your Linux and Windows machines are needed for the communication.
The Linux server side is automatically launched after the `fold.py` is launched.

- Some commom parameters such as the calibration matrix of the Prime Sense is stored in `/src/folding_control/src/parameters`.

- All the source code can be found in `/src/folding_control`.



## Folding Planner

- The [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) library and [GL](http://freeglut.sourceforge.net/) library are required. 

- If you are using Ubuntu, you can jump into `/folding_planner_cmake` and compile the things inside for key points detection. 

- The program requires additional three libraries to compile, which are `aerialeye.lib`, `blackbill.lib`, and `bluetail.lib`. 
Here we only provide binaries (compiled using Visual Studio 2010) for research purposes, which can be found in `src/folding_planner/external/libs`. 
All the header files are in `src/folding_planner/external/hdrs`.
If they are not compatible with your system, please contact the authors. 


- We provide a `.vcproj` project file, which can be added to any Visual Studio project (2010 or later). Then some properties such as lib path can be modified to your local path.

- To launch the server on a Windows machine, terminal or [cygwin](https://www.cygwin.com/) (recommended) can be used. 
The communication module `src/folding_control/src/xmlrpclib-1.0.1` needs to be copied over to the Windows machine.
Sample usage can be found in `src/folding_control/src/communication_module.py`.

- Sample trajectories from the off-line simulation are in `src/folding_planner/traj`. 
The garments include a sweater, a pair of pants, and a mid-size towel.

- We also provide three test images for your quick test of the key point localization in `/images`.
A sample result is shown below. The images from the left to right: initial contour template, detected key points, and key points mapping back to the original image.
![sample](https://github.com/Yinxiaoli/IROS2015/blob/master/keypoints_sweater.png)

- All the source code can be found in `/src/folding_planner`.



## Citation 

Please kindly cite our paper if our code helps your research.

* Folding Deformable Objects using Predictive Simulation and Trajectory Optimization, Yinxiao Li, Yonghao Yue, Danfei Xu, Eitan Grinspun, and Peter Allen,
Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Hamburg, Sept. 2015

bibtex:

	@inproceedings{Li_IROS2015,
	title = {Folding Deformable Objects using Predictive Simulation and Trajectory Optimization},
	author = {Yinxiao Li and Yonghao Yue and Danfei Xu and Eitan Grinspun and Peter K. Allen},
	booktitle = {Proceedings of the 27th IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	year = 2015
	}