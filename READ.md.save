# MuJoCo Screw-Propell

A physics-accurate MuJoCo simulation of a bio-inspired screw-driven snake robot that moves using counter-rotating Archimedes screws instead of traditional undulation.  
This project explores how rotational motion can be converted into translational thrust through frictional coupling with deformable terrain, inspired by amphibious and soft-ground robots.

---

## Overview

This simulation models a robotic platform equipped with two counter-rotating screws that generate thrust along the ground.  
Each screw is represented by an array of capsule geometries arranged helically, creating a high-fidelity approximation of a threaded cylinder.

The environment is a soft, high-friction surface that allows realistic interaction with the screw threads, letting the robot move forward or backward purely through rotation.

---

## Features

- **Dual Counter-Rotating Screws**  
  Opposite-handed helical screws (`jr` and `jl`) linked by a gear constraint to cancel lateral torque and produce forward thrust.

- **Realistic Frictional Contact**  
  Tuned `solref` and `solimp` parameters simulate deformable terrain for stable, continuous contact dynamics.

- **Procedural Geometry Generation**  
  `make_screw_xml.py` generates helical ridges automatically — adjustable pitch, radius, handedness, and number of segments.

- **Analytical Validation**  
  `equations.txt` includes axial thrust, torque, and power balance equations derived from frictional screw mechanics.

- **Interactive Visualization**  
  `run.py` launches the MuJoCo viewer and simulates dynamic motion usin
