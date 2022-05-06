# chefbot-perception
This is the source code for my senior research project to perceive the items in our work station. We leveraged color masking and the OpenCV library to filter images to perceieve the plates, cups, food and stove on/off state.
 

## Requirements
- Valid ROS distribution (Tested on ROS Melodic with Ubuntu 18.04)
  - If your machine does not contain run on Linux, you can use [Docker](https://docs.docker.com/get-docker/) and the custom Dockerfile I wrote which includes the necessary packages and installments
- OpenCV


## Notes
- The azure_image_saver.py and oven_perception.py ROS nodes need to be in executable mode; use `chmod +x file_name.py`
- The parameters for the bounding box surrounding the oven light can be found in oven_light_params.yaml. These parameters must be loaded into the ROS Raram Server from the config directory by running the following: 
```
rosparam load oven_light_params.yaml
```
