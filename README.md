On-line POMDP Planning Toolkit (OPPT)
==========================================================================

# OPPT
On-line POMDP Planning Toolkit (OPPT) is a C++ software-toolkit for approximating POMDP solutions on-line. To ease implementation of POMDP models, OPPT allows users to specify the state, action, and observation spaces via a configuration file, and uses a plug-in architecture to implement the core components of a POMDP, that is, the transition, observation and reward functions.

## Installing the dependencies

### On-line installation script
OPPT provides an installation script that installs all the dependencies:

	cd <oppt_root_folder>
	chmod +x install_dependencies.sh && . ./install_dependencies.sh --use-ros
	
This script will install the dependencies inside the */usr/local* folder, hence, it will ask you for your sudo password. It has been tested on a clean Ubuntu 16.04 installation.
If you wish to not install the optional ROS Kinetic dependency, run the above command without the "--use-ros" option.

### Step-by-step installation

1. Install the core dependencies

        sudo apt-get install build-essential git cmake mercurial pkg-config libboost-all-dev libtinyxml-dev libeigen3-dev libassimp-dev libfcl-dev
        
2. Install Gazebo and it's dependencies by following the instructions according to the [Gazebo website](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
        
3. Download and install libspatialindex-1.8.5

        wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz
        tar zxfv spatialindex-src-1.8.5.tar.gz
        cd spatialindex-src-1.8.5
        mkdir build && cd build
        cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
        make -j2 && sudo make install
        
4. (Optional) If you want to use the GUI provided in OPPT, you need to have a working installation of ROS Kinetic (at least the ROS-base stack along with the ros-kinetic-rviz stack). Furthermore your ROS environment has to be set up (please refer to http://wiki.ros.org/kinetic/Installation/Ubuntu section 1.6). If you do not have ROS Kinetic installation, you can install it using

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-get update
        sudo apt-get install ros-kinetic-ros-base ros-kinetic-rviz
        
## Setting-up ROS Kinect (optional)
If you have a working ROS Kinetic installation, make sure that the ROS environment is set-up properly.
For a clean ROS installation, this is done via

    source /opt/ros/kinetic/setup.sh
    
If you fail to do this, you won't be able to use the viewer.        

## Building and installing OPPT

    cd <oppt_root_folder>/src/
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=<BUILD_TYPE> -DCMAKE_INSTALL_PREFIX=<oppt_install_location> ..
    make && make install

OPPT supports three build types: Debug, RelWithDebInfo and Release. The default build type is RelWithDebInfo

## Configuring the OPPT runtime environment

In order to obtain resources such as SDF models, plugins, etc., OPPT uses a filesystem API

    oppt::FileExists("filename")
    oppt::FindFile("filename")
    
which locates resources inside folders specified in the 

    OPPT_RESOURCE_PATH
    
environment variable. To configure your current environment, either call

    source <oppt_install_location>/share/oppt/setup.sh
    
or add this line to your bash environment file
    
This will add the 

    <oppt_install_location>/share/oppt/models
    <oppt_install_location>/share/oppt/plugins

folders to the resource environment variable. If you want to add additional folders, use

    export OPPT_RESOURCE_PATH=$OPPT_RESOURCE_PATH:<additional_folder>

## Quick start 

After the OPPT environment has been configured, navigate to the

    <oppt_root_folder>/bin
    
directory. Here you should see the 'abt' executable. If you have installed the optional ROS Kinetic dependency, you should see two additional executables, 'viewer' and 'playVideo'.
The problem configuration files for the problems that ship with OPPT are inside the

    <oppt_root_folder>/cfg
    
directory. To run the ABT solver for any of the problems, run

    ./abt --cfg <oppt_root_folder>/cfg/<ProblemConfigurationFile>.cfg
    
Note that the path to the problem configuration file must be an absolute path.

If you want a visualization of the simulation runs, open a separate terminal and run

    opptViewer
    
BEFORE running the ABT solver.

## Documentation        

The OPPT HTML documentation can be found inside the 

	<oppt_root_folder>/docs/html
	
folder

## Changelog
### v0.1
* Initial public release

### v0.2
* Inside the modified ODE physics engine used by Gazebo, a cumulated joint angle parameter per joint is maintained during a physics update. This joint angle parameter is not being reset when setting the Gazebo world state, causing non-deterministic results. This is fixed by resetting the parameter before each physics engine update.
* Added virtual setUserData and getUserData the the OPPTPlugin. These methods can be overwritten and used to share objects amongst different plugins.
* Build the core OPPT framework as a shared library that solvers link against. This reduces compilation time when many solvers are being compiled.
* Added APIs to the Robot class that allows the user to set custom collision functions
* Added support for trac_ik inverse kinematics engine

### v0.3
* Added support for Gazebo9
* Many bugfixes and performance improvements

## Publication
Marcus Hoerger, Hanna Kurniawati, and Alberto Elfes. A Software Framework for Planning under Partial Observability. *Proc. IEEE/RSJ Int. Conference on Intelligent Robots (IROS)*. 2018. [[PDF]](http://rdl.cecs.anu.edu.au/papers/iros18_oppt.pdf).

<details><summary>Bibtex</summary>
<p>

```
@inproceedings{hoerger18:OPPT,
  title={A Software Framework for Planning Under Partial Observability},
  author={Hoerger, Marcus and Kurniawati, Hanna and Elfes, Alberto},
  booktitle={Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={1--9},
  year={2018},
  organization={IEEE}
}
```

</p>
</details>

## Contact
For questions, bug reports or suggestions please contact
Marcus Hoerger, E-Mail: hoergems@gmail.com 
