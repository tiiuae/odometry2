# Odometry2

## TODO:
* [x] Create a service to set gps origin for control
* [ ] Autonomous estimator switch using covariances
* [ ] Service to reset and restart hector
* [ ] Service to manually switch estimator
* [ ] Fuse z for hector estimator
* [x] Obtain z coordinate separately for hector fusion
* [ ] Obtain Hector covariances

## Estimation notes

* Fuse baro directly or z position from pixhawk odom local?
* Publishing odometry and tfs only from active estimator?
  * The answer is yes
* How to load matrices?
 
* [ ] getHeading 
* [ ] getHeadingRate
* [ ] control inputs in predictions?
* [ ] Do not fuse garmin when tilt too large





