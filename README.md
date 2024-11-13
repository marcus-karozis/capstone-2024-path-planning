# Path Planning V2

Author: Marcus Karozis

## Overview

This package is a path planning package for UTSMA

## To Do

### cone comparison

- implement toml config file
- cone_detection_MOE_rate
- move cone pos validity checking to TL Vector derivation function


### midline derivation

- optimise finding opposite cone so that it only checks cones immediatly infront and behinde opposite index

### raceline derivation

- delayed until rest of program is working
- currently just returns midline

### tracklimit vector derivation

- optimise so that the bst isnt recreated every time and is intead updated
- improve scanArea function to be more accurate and robust
- improve cone likelihood scoring to search more than depth 1
- check for loop closure in the track limit vector derivation
- BST should only be used for track limit derivation and finding likely paths, track limits should otherwise be stored in a vector or path object
- create function to convert BST to path object
