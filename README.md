# Verd



<p align="justify">
  
This project was carried out as part of a course in the mechatronics engineering course at the University of São Paulo, Brazil. The objective was to design a manipulator that could carry out an activity freely chosen by the students

</p>

# Setting Up

<p align="justify">

This work was made using Docker containers, so you will neede to have Docker installed.

You can install it following the steps [here](https://docs.docker.com/get-started/get-docker/)

</p>

## Step 1 - Cloning the git repository

To copy the repository, run the following command in terminal

</p>

```bash
git clone https://github.com/Gabrielcancado/Verd.git
```

It should create a folder called Verd in the location where you ran the command.

## Step 2 - Building Docker Container

*Inside the "Verd" folder on your computer (created with previous command).

```bash
bash docker/scripts/build.sh 
```

## Step 3 - Running Docker Container

<p align="justify">

If everything went well up to this point, the following command should start your container

</p>

```bash
bash docker/scripts/run.sh
```

You will need to execute that command every time you want to turn on the container

## Step 4 - Setting up ROS Workspace

<p align="justify">

Now, inside the container, its necessary to execute these commands to correctly setup the environment

```
colcon build
```
```
source install/setup.bash
```

## Step 5 - Runnig the simulation

<p align="justify">

To run the simulation on Rviz of the manipulator, just use the following command.

```
ros2 launch manipulator3 display.launch.py
```






