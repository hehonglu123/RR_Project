# Workspace Setup:
* Tesseract Feature Branch
* [Tesseract Robot Raconteur](https://github.com/johnwason/tesseract_robotraconteur)
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur)
* [Robot Raconteur Companion](https://github.com/johnwason/robotraconteur_companion)

* Build command:
$`catkin config --cmake-args -DROBOTRACONTEUR_ROS=1 -DCMAKE_BUILD_TYPE=Release`
$`catkin build`
# Environment Variable:
$`export TESSERACT_SUPPORT_DIR=/home/<username>/<workspace>/src/tesseract/tesseract/>`

# URDF modification:
Change all `username` to actual local username in `urdf/combined.urdf`

# Run Instructions:
source workspace
## RR Tesseract Service
$`tesseract_robotraconteur_service --urdf-file=urdf/combined.urdf --srdf-file=urdf/combined.srdf`
## Planner
$`python3 planner_test2.py`
