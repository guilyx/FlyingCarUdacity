[![Udacity - Fly Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](https://www.udacity.com/course/flying-car-nanodegree--nd787)
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<br />
<p align="center">
    <img src="misc/controls-menu.gif" alt="Simulator" width="540" height="400">
  </a>

  <h3 align="center">3D Motion Planning - Autonomous UAV Nanodegree</h3>

  <p align="center">
     Path planner using various discretization of the configuration space for A* search.
    <br />
    <a href="https://github.com/guilyx/autonomous-uav"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/guilyx/autonomous-uav">View Demo</a>
    ·
    <a href="https://github.com/guilyx/autonomous-uav/issues">Report Bug</a>
    ·
    <a href="https://github.com/guilyx/autonomous-uav/issues">Request Feature</a>
  </p>
</p>



## Table of Contents

* [About the Project](#about-the-project)
* [Setup](#setup)
* [Run](#run)
  * [Simulator](#simulator-planning)
  * [Matplotlib](#obstacle-csv-planning)
* [Roadmap](#roadmap)
* [Contribute](#contribute)
* [License](#license)
* [Contact](#contact)
* [Contributors](#contributors)

## About the Project

<p align="center">
    <img src="misc/planning.gif" alt="Planning" width="600" height="450">
  </a>
</p>

This is the second project of the UDACITY Flying Cars Nanodegree. It contains more or less elaborate planning and discretization techniques. As of now only scripts for the Udacity Simulator are available, but a matplotlib based representation of the configuration space and a ploted path of all the algorithms used will eventually come.


## Setup

1. [Download miniconda](https://conda.io/miniconda.html) and install it.
2. Star the project (hehe).
2. Clone the project. `git clone https://github.com/guilyx/autonomous-uav.git`
3. Create the miniconda environment. `conda env create -f environment.yml`
4. Activate the environment. `source activate fcnd`.
5. If everything went well you can now use the projects scripts.
6. Download latest version of the [UAV Simulator from Udacity](https://github.com/udacity/FCND-Simulator-Releases/releases)

You should now be ready to go.

## Run

### Udacity Simulator Planning

These four scripts will build your plan for the simulator with different planning/discretizing approaches. As of now they all use A* to find the optimal path and prune the colinear points of the path by default. ( comment function call to remove that )

1. Grid discretization : `python src/motion_planning_grid.py`
2. Medial Axis discretization : `python src/motion_planning_medialaxis.py`
3. Voronoi Graph discretization : `python src/motion_planning_voronoi.py`
4. Probabilistic Graph discretization : `python src/motion_planning_probabilisticroadmap.py` # Under Development

Medial Axis and Grid discretization have diagonal actions activated by default, you can change the value in the MotionPlanner constructor. Note that all the scripts use arguments to define the goal position. Use `--goal_lon=x --goal_lat=y --goal_alt=z` to use a custom destination. A default one is defined so it's not mandatory.

### Matplotlib Planning

To Do : Use algorithms outside the simulator on the grid and display the found path.

1. Grid discretization : `python src/grid_search.py`
2. Medial Axis discretization : `none`
3. Voronoi Graph discretization : `none`
4. Probabilistic Graph discretization : `none`
5. Receding Horizon : `none`
6. Potential Field : `none`

## Abstractions

### Differences between Backyard Flyer and Motion Planner

Backyard Flyer has one fewer state as it's never planning : the waypoints are already hardcoded to draw a square-ish. For the motion-planner, a "Planning" state was added to compute the waypoints during this state. The UAV will take-off to a defined altitude after entering the planning state. Your waypoints will be calculated and the drone will follow the path.

## Roadmap

See the [open issues](https://github.com/guilyx/autonomous-uav/issues) for a list of proposed features (and known issues).

## Contribute

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

### Contribute on proposed features

1. Choose any open issue from [here](https://github.com/guilyx/autonomous-uav/issues). 
2. Comment on the issue: `Can I work on this?` and get assigned.
3. Make changes to your fork and send a PR.

Otherwise just create the issue yourself, and we'll discuss and assign you to it if serves the project !

To create a PR:

Follow the given link to make a successful and valid PR: https://help.github.com/articles/creating-a-pull-request/

To send a PR, follow these rules carefully, **otherwise your PR will be closed**:

1. Make PR title in this formats: 
```
Fixes #IssueNo : Name of Issue
``` 
```
Feature #IssueNo : Name of Issue
```
```
Enhancement #IssueNo : Name of Issue
```

According to what type of issue you believe it is.

For any doubts related to the issues, i.e., to understand the issue better etc, comment down your queries on the respective issue.

## License

Distributed under the MIT License. See `LICENSE` for more information.

## Contact

Based on seed project from Udacity ( 3D Motion Planning - Flying Cars Nanodegree )
Erwin Lejeune - [@spida_rwin](https://twitter.com/spida_rwin) - erwin.lejeune15@gmail.com

## Contributors

- [Erwin Lejeune](https://github.com/Guilyx)

[contributors-shield]: https://img.shields.io/github/contributors/guilyx/autonomous-uav.svg?style=flat-square
[contributors-url]: https://github.com/guilyx/autonomous-uav/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/guilyx/autonomous-uav.svg?style=flat-square
[forks-url]: https://github.com/guilyx/autonomous-uav/network/members
[stars-shield]: https://img.shields.io/github/stars/guilyx/autonomous-uav.svg?style=flat-square
[stars-url]: https://github.com/guilyx/autonomous-uav/stargazers
[issues-shield]: https://img.shields.io/github/issues/guilyx/autonomous-uav.svg?style=flat-square
[issues-url]: https://github.com/guilyx/autonomous-uav/issues
[license-shield]: https://img.shields.io/github/license/guilyx/autonomous-uav.svg?style=flat-square
[license-url]: https://github.com/guilyx/autonomous-uav/blob/master/LICENSE.md
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/erwinlejeune-lkn

