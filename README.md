# Autonomous navigation and parking of a SVEA robot

## Description

This repository contains the source-code for autonomously controlling the Small Vehicles for Autonomy (SVEA) platform [1]. More precisely, it contains the implementaion of a nonlinear model predictive controller that allows the vehicle to navigate towards a parking spot and then perform a parking maneuver for both parallel and perpendicular parking. In addition, it contains safety elements in the form of avoiding collisions with moving obstacles in its near vicinity.


## Setup

### Dependencies

The package requires that the SVEA framework https://github.com/KTH-SML/svea is installed.

## Demo

A demonstration of an implementation on the real robot can be seen in the video below.


<video width="580" height="340" controls>
  <source src="https://github.com/loizoshad/svea_mpc/blob/main/demo/parallel.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>


## Collaborators

This project was developed as part of a course project at KTH Royal Institute of Technology by the following students:

* Loizos Hadjiloizou
* Erik Anderbeg
* Philipp Katterbach
* Gunnar Sigurðsson
* Johan Ahnfalk
* Vidar Greinsmark
* Johan Hallberg
