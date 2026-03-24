# OpenCDA-Deployment

Since the repository maintainers have not updated the project's deployment documentation for a long time, many links are difficult to reproduce. We have tried out a configuration that is relatively new and has no conflicting dependencies. 

### Platform

AMD64 Processor / Windows 11

AMD64 Processor / Ubuntu 24.04.4 LTS

Not limited to the two environments mentioned above, you can try it on more platforms and provide feedback on the test results.

### Steps

##### Download Carla

Look into https://github.com/carla-simulator/carla, and find version 0.9.12, download the Carla's pre-compiled package and additional maps for your platform. Extract the maps' compressed file to Carla's root path in order to merge their content. 

##### Setup basic environment

Before this step, make sure you have git and conda on your computer. 

```bash
git clone https://github.com/dunegym/OpenCDA-Deployment.git
cd OpenCDA-Deployment
conda env create -f environment.yml
conda activate opencda
pip install -r requirements.txt
```

If the above command executes successfully, the basic functionality of OpenCDA can now run on your device.

##### Test basic environment

Before running the scripts from OpenCDA, you should start Carla's service first. For Linux users, run "path/to/carla/CarlaUE4.sh"; for Windows users, run "path/to/carla/CarlaUE4.exe"; if an interface showing a picture of city appears on your screen, the service is running. 

To test basic environment, run: 

```python
python opencda.py -t single_2lanefree_carla -v 0.9.12
```

In this scenario, a single Connected Automated Vehicle will be spawned at a 2-lane highway customized map. This CAV will try to reach the assigned destination with a desired speed of 100km/h and manage to safely interact with the surrounding traffic flow. 

```python
python opencda.py -t platoon_stability_2lanefree_carla -v 0.9.12
```

In this scenario, a platoon with 4 members will be placed at the 2-lane highway map. The platoon leader will dramatically increases and decreases its speed to test whether the members can still keep the desired time gap. In this way, the platoon stability is verified. 

```python
python opencda.py -t platoon_joining_2lanefree_carla -v 0.9.12
```

In this scenario, a platoon will drive on the mainline together with a mixed traffic flow. A single CAV will come from the merging lane, communicate with the platoon to cooperatively merge into the mainline, and simultaneously join the platoon.

##### Setup yolo environment

This section is only needed for the users who want to test perception algorithms. By default, OpenCDA does not require pytorch installed and it retrieves the object positions from the server directly. Once perception module is activated, then OpenCDA will use yolov5 with pytorch to run object detection. 

```bash
pip install -r requirements_yolov5.txt
```

It may warn you to install urllib3>=2.6.0 and Python>=3.9, but Carla 0.9.12 only supports python==3.7/3.8, so just ignore it. 

##### Test yolo environment

To test yolo environment, run: 

```bash
python opencda.py -t single_town06_carla  -v 0.9.12 --apply_ml
```

The apply_ml flag will import the pytorch library and load Yolov5 model (Thus Pytorch is required) for object detection. Thus, in this scenario, the perception, localization, planning and control modules will all be activated. 

```bash
python opencda.py -t platoon_joining_town06_carla  -v 0.9.12 --apply_ml
```

A single CAV will try to overtake several human-driven vehicles to join the platoon from the back. 

##### Setup SUMO environment

SUMO is a trafic simulator often used with CARLA. Some of the functions in OpenCDA need SUMO. Look into https://sumo.dlr.de/docs/Downloads.php to download it for your platform. 

Then we can use SUMO's python api:

```bash
pip install traci==1.26.0
```

##### Test SUMO environment

```bash
python opencda.py -t platoon_joining_2lanefree_cosim -v 0.9.12
```

Cooperative merge and joining a platoon under co-simulation. 

```bash
python opencda.py -t single_town06_cosim  -v 0.9.11 --apply_ml
```

This scenario applies Sumo to generate the traffic flow instead of using Carla traffic manager. Yolov5 and simple Lidar fusion are used to detect object 3D bounding box. 
