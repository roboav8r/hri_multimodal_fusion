# hri_multimodal_fusion
Package to fuse multiple HRI data modalities. Currently exploring using graph model fusion with GTSAM backend.

# Prerequisites
Install Eigen and create a softlink:
sudo apt-get install libeigen3-dev
cd /usr/include/
sudo ln -s eigen3/Eigen/ Eigen


Install GTSAM using the instructions on the Borglab GitHub repo:
https://github.com/borglab/gtsam

#!bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
python3 -m pip install -r python/requirements.txt 
mkdir build
cd build
cmake .. -DGTSAM_BUILD_PYTHON=1
make check # optional, runs C++ unit tests
make python-test # optional, runs Python unit tests - FAILED
make
make python-install



# Usage

## Calibration

### Converting rosbags to .csvs 
```
rostopic echo -b src/hri_multimodal_fusion/data/2023-04-26-14-49-44.orig.bag -p /tf > tf.csv
```
