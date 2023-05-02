# hri_multimodal_fusion
Package to fuse multiple HRI data modalities. Currently exploring using graph model fusion with GTSAM backend.

# Usage

## Calibration

### Converting rosbags to .csvs 
```
rostopic echo -b src/hri_multimodal_fusion/data/2023-04-26-14-49-44.orig.bag -p /tf > tf.csv
```
