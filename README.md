# hri_multimodal_fusion
Package to fuse multiple HRI data modalities. Currently exploring using graph model fusion with GTSAM backend.

# Prerequisites
Install Eigen and create a softlink:
sudo apt-get install libeigen3-dev
cd /usr/include/
sudo ln -s eigen3/Eigen/ Eigen


Install GTSAM:

#!bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
python3 -m pip install -r python/requirements.txt
mkdir build
cd build
cmake ..
make check # optional, runs unit tests
sudo make install

cmake .. -DGTSAM_BUILD_PYTHON=1
make # TODO - fails here



# Usage

## Calibration
To calibrate the OAK-D sensor, send a command to run the bag and label service:
```
rosservice call /bag_and_label_oakd_data "label_data:
  labels:
  - size: 1
    label: 'person'
    activity: 'sitting'
filepath: 'data/test'
duration_sec: 10" 
```


### Converting rosbags to .csvs 
```
rostopic echo -b src/hri_multimodal_fusion/data/2023-04-26-14-49-44.orig.bag -p /tf > tf.csv
```

# Additional Resources and References
https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf
https://gtsam-jlblanco-docs.readthedocs.io/en/latest/Overview.html

# Improvements
- Make template class for filters, transition models, sensor models based on number of spatial dimensions and derivatives
- Make motion models and sensor models static
- TODO make transition model TransitionModels::parser function to go from params to variables
