# Search_and_Rescue_Drone_using_Reinforcement_Learning
## About the Project 
This project leverages **Reinforcement Learning (RL)** to enable autonomous drones for **Search and Rescue (SAR)** missions in hazardous environments. Using **Proximal Policy Optimization (PPO)**, the drone learns to navigate disaster zones, locate victims, and optimize flight paths while avoiding obstacles. Implemented in **ROS** and tested in **Gazebo**, the system balances exploration and efficiency to improve SAR operations. Experimental results show that PPO-driven drones enhance mission reliability and adaptability, offering a scalable solution for autonomous rescue efforts.
## System Architecture 
The system architecture integrates ROS2 middleware, Gazebo simulator, OpenAI Gym, and Stable Baselines 3 for PPO training and deployment.

![drone sensors readings flow](https://github.com/user-attachments/assets/63a82bec-17ba-4ad5-a0d5-ff770bfb6150)

## Simulation Environment 
The drone was tested in a simulation environment created in Gazebo

![gazebo env](https://github.com/user-attachments/assets/afc78db3-b650-4e9e-b8b9-f774a73cafc3)

## Running the repo

Clone this repository into your ROS2 workspace

```
cd ~/ros2_ws/src
git clone <repository_url>

```
Navigate to the root of your workspace and build the package
```
cd ~/ros2_ws
colcon build 
```

### Source the workspace
```
source install/setup.bash
```
### Lauch the SJTU drone
```
ros2 launch drone_rl drone_rl_start.launch.py

```
### Now the drone is spawned in the environment and now to t ake off the drone
```
ros2 topic pub /demo/takeoff std_msgs/msg/Empty {}

```

### To start the training of PPO


```
ros2 launch drone_rl start_training.launch.py
```

The drone based on reward functions defined, will try to get to the goal and resets everytime it hits an obstacle

 ![Screenshot from 2024-12-11 01-06-15](https://github.com/user-attachments/assets/a5dec793-817a-4463-bf7d-b02e9a68e0f6)


### To test the checkpoint saved, run

```
ros2 launch drone_rl test.launch.py
```

## Results

Few metrics captured by Tensorboard

### Clip Fraction

![clip_fraction_0](https://github.com/user-attachments/assets/ca4e52a9-2d8e-43f5-9039-21f0b394f142)


### Entropy Loss

![entropy_loss_0](https://github.com/user-attachments/assets/233d8b91-e4f3-4a3b-9711-8f75edec60d8)

### KL Divergence

![kl-divergence_0](https://github.com/user-attachments/assets/7f6f4809-24e7-4d81-b58f-d0964cabd601)


### Inference Video

https://drive.google.com/file/d/1rX_Co7UapMUwAzIdfQrSaqxVZ5ZzpA7F/view?pli=1
