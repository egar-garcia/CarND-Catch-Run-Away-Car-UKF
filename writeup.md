**Run Away Robot with Unscented Kalman Filter - Bonus Challenge**


Self-Driving Car Engineer Nanodegree Program

In this project, not only an UKF is implemented, but it is also used to catch an escaped car driving in a circular path.
The run away car will be being sensed by a stationary sensor, that is able to measure both noisy lidar and radar data. The capture vehicle will need to use these measurements to close in on the run away car. To capture the run away car the capture vehicle needs to come within .1 unit distance of its position. However the capture car and the run away car have the same max velocity, so if the capture vehicle wants to catch the car, it will need to predict where the car will be ahead of time.

[//]: # (Image References)
[BonusChallengeVideo]: ./Docs/CatchTheRunAwayCar.mov

### Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./UnscentedKF
```



### Discussion

This is the bonus challenge for the project "Unscented Kalman Filter" which solution can be found at [https://github.com/egar-garcia/CarND-Unscented-Kalman-Filter-Project](https://github.com/egar-garcia/CarND-Unscented-Kalman-Filter-Project)

A video of one execution can be found in the file ```Docs/CatchTheRunAwayCar.mov```

![BonusChallengeVideo]

To resolve this challenge the strategy was to implement a couple of methods (```PredictAhead()``` and ```GetCatchingPoint```) to predict a point of intersection ahead in the future, considering the speed of the running car (which is expected to be the same as the one of the persecutor), so the point of intersection would be the estimated to cover the distance that separates the two vehicles, as the persecutor gets closer the estimate gets more accurate.

The mechanism is that after executing the predict/update process, a prediction for a point of intersection takes place, then the persecutor vehicle is directed towards that point.
