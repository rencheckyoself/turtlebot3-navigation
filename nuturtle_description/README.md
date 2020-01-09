# Description
This package contains files to create robot descriptions suitable for simulation. It contains various urdf files and basic debugging, testing, and visualization code for the robots used in the other packages.

# How to run:
Copy the following into a console window: </br>
```roslaunch nuturtle_description view_diff_drive.launch```


# File Tree:

```
├── config
│   ├── diff_params.yaml - contains parameters to define a generic diff drive robot
│   └── view_urdf.rviz - rviz config file
├── launch
│   └── view_diff_drive.launch - main launch file to view diff drive robot
└── urdf
    └── diff_drive.urdf.xacro - xacro file that generates a diff drive robot
```
