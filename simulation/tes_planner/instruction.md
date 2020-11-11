# Workspace Setup:
* Tesseract Feature Branch
* [Tesseract Robot Raconteur](https://github.com/johnwason/tesseract_robotraconteur)

# URDF modification:
Change all `username` to actual local username in `urdf/combined.urdf`

# Run Instructions:
source workspace
## RR Tesseract Service
`tesseract_robotraconteur_service --urdf-file=urdf/combined.urdf --srdf-file=urdf/combined.srdf`
## Planner
`python3 planner2.py`
