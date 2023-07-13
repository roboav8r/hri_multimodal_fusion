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

## Launching the system with OAK-D camera
```
roslaunch hri_multimodal_fusion oakd_filter.launch training:=true # Just launch the node, which publishes object class and position
roslaunch hri_multimodal_fusion oakd_filter.launch visualization:= true # Launch the node with RViz
roslaunch hri_multimodal_fusion oakd_filter.launch visualization:= true training:=true # Launch the training node to record data
```

## Recording training data
To calibrate the OAK-D sensor, send a command to run the bag and label service while the training node is running. For example:
```
rosservice call /bag_and_label_sensor_data "label_data:
  labels:
  - size: 1
    label: 'person'
    activity: 'standing'
    motiontype: 0
filepath: 'data/person_standing_lab_3'
duration_sec: 30"
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
- Make transition model TransitionModels::parser function to go from params to variables

Multi-Sensor
- obs_models.hpp: 
  - Make ObsModelParams::SensorMdl into vector of <SensorMdl>s
  - push_back in extractsensorparams
  - define sensor frames
- Make callbacks for each sensor - put in Sensors namespace
  - Figure out how to pass callback function reference with filter.update() reference
- filters.hpp
  - Make subs_ vector member variable 
  - Define filter frame
- inference_filter.cpp
  - populate filter.subs with new sensors
- train_node.cpp
  - specify .bagfile topics based on sensor configuration file

Visualization
- Performance improvements with marker array message
- Convert visualization from marker publisher to rviz plugin
- make topics parameters instead of fixed `/tracks`

Error checking
- Ensure obs_model.yaml has consistent number of classes, measurements, and measurement likelihood matrix
- Ensure each sensor has the same number of class labels (meas labels can differ though). Can do this in obs_models.hpp ExtractSensorParams