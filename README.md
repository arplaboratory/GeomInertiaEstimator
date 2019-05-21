# GeomInertiaEstimator
a nonlinear Kalman Filter based ROS package that allows to estimate inertia and geometric parameters of multirotors in-flight and re-estimates them online

By combining rotor speed measurements with data from an Inertial Measurement Unit (IMU) and any kind of pose sensor, an Unscented Kalman Filter (UKF) or Extended Kalman Filter (EKF) estimates inertia parameters (mass, moment of inertia, position of center of mass) and geometric parameters (position of IMU, position of pose sensor).

We facilitate the setup process and demonstrate the performance of the estimator by providing an example bag file containing the data from one of our experiments (_config/lissajous_trajectory.bag_). Furthermore, we provide a layout file (_config/PlotJuggler_Layout.xml_) for [PlotJuggler](http://wiki.ros.org/plotjuggler) to plot and analyse the estimates more easily.

  <img src="https://raw.githubusercontent.com/arplaboratory/GeomInertiaEstimator/master/config/Multirotor.svg?sanitize=true" width="100%" height="140">
  
<!--video -->

## Reference
Please cite the following publication in case you are using the package in an academic context:

Wüest V, Kumar V, Loianno G. "**Online Estimation of Geometric and Inertia Parameters for Multirotor Aerial Vehicles**." _2019 IEEE International Conference on Robotics and Automation (ICRA)_. IEEE, 2019.
```
@inproceedings{wueest2018estimation,
  title={Online Estimation of Geometric and Inertia Parameters for Multirotor Aerial Vehicles},
  author={W{\"u}est, Valentin and Kumar, Vijay and Loianno, Giuseppe},
  booktitle={2019 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={},
  year={2019},
  organization={IEEE}
}
```
In the publication you can find details about:
* parameter definitions
* derivations of models
  * system dynamics
  * measurements
* filter implementation on _SO_(3)
* nonlinear observability analysis
* experimental results

## License
Please be aware that this code was originally implemented for research purposes and may be subject to changes and any fitness for a particular purpose is disclaimed.
To inquire about commercial licenses, please contact [Valentin Wüest](mailto:valentinwueest@gmail.com) and [Giuseppe Loianno](mailto:loiannog@nyu.edu).
```
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

## Installation
Clone the _GeomInertiaEstimator_ repo into your catkin workspace:
```
cd ~/catkin_ws/src/
git clone git@github.com:arplaboratory/GeomInertiaEstimator.git
```

Build the _GeomInertiaEstimator_ package:
```
catkin_make --pkg geom_inertia_estimator --cmake-args -DCMAKE_BUILD_TYPE=Release
```

In case an error message appears, try running the last step again.

## Usage
To use the estimator, first enter the parameters of your multirotor in _config/quad_params.yaml_.

Make sure that the three topics _IMU_, _pose_ and _motor rpm_ are published. 

Then, remap these topics in _launch/estimator.launch_ and launch the estimator by executing:
```
roslaunch geom_inertia_estimator estimator.launch
```

## Example
Firstly, install _PlotJuggler_ if you have not already:
```
sudo apt-get install ros-$ROS_DISTRO-plotjuggler
```

In a terminal window, start the roscore:
```
roscore
```

In a second terminal window, start the estimator:
```
roslaunch geom_inertia_estimator estimator.launch
```

In a third one, play the example experiment bag file:
```
roscd geom_inertia_estimator/
rosbag play config/lissajous_trajectory.bag --pause
```

You can now plot the estimates using plotjuggler by executing this command in a fourth window:
```
roscd geom_inertia_estimator/
rosrun plotjuggler PlotJuggler -l config/PlotJuggler_Layout.xml
```
When prompted, hit "_Yes (Both Layout and Streaming)_", "_OK_", and "_Create empty placeholders_". You can then unpause the bag play by clicking on the rosbag terminal window and hitting _SPACEBAR_. Now, enjoy following the plots being drawn!

## Remark
If you intend to change the mathematical model of the estimator, please use the Unscented Kalman Filter (UKF) instead of the Extended Kalman Filter (EKF), as we have not yet provided the Matlab functions used to calculate the linearized state transition model.
