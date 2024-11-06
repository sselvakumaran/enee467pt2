# ROS Workspace for Lab Exercises

Go to the [Read the Docs](https://enee467.readthedocs.io/en/latest/) page to get started!

If you wish to have your own copy of this workspace on GitHub to save the work, create a new
repository on your GitHub account using this repository as a **template**. Otherwise you can directly
clone this repository and name the workspace folder with the group name.

Modify the `<workspace-name>` field and clone the repository to the `home` directory.

```bash
git clone https://github.com/ENEE467/lab-workspace.git ~/<workspace name>
```

## Further Steps

<!-- TODO: Add links -->
- [Opening the workspace folder in VSCode](https://enee467.readthedocs.io/en/latest/Setup.html#opening-the-workspace-in-visual-studio-code)
- [Lab 7](https://enee467.readthedocs.io/en/latest/Lab7.html)
- [Lab 8]()


# Lab 7
## REPORT:
### 1. Give the pose in World Coordinates of the camera as determined in the initial calibration.
(0.06013711189526902, 0.6189519265470212, 1.266066252511441)
### 2. Compute and include in your report the square root of the sum of the squares of the components of the quaternion. How close is it to one?
sqrt(sum(q.^2)) = 20.3724
This is quite a ways away from 1, but likely mostly due to the LSE of 19.5459 for the last component.
### 3. Give the sample mean and sample covariance of the errors in the calibration as determined by the second set of data points.
Mean error vector: 
 0.363706
 0.853423
  1.78729
-0.424018
-0.152155
-0.842074
  2.33657

Covariance matrix: 
[  0.273933   0.133482    0.44935 -0.0711276  0.0799115  -0.112811   0.542731 ;
  0.133482   0.611983    1.11569  -0.298115  -0.159228  -0.587114    1.47603  ;
   0.44935    1.11569    2.39474  -0.526562  -0.231846   -1.14549    3.05489  ;
-0.0711276  -0.298115  -0.526562   0.173779  0.0740058   0.343596  -0.746645  ;
 0.0799115  -0.159228  -0.231846  0.0740058   0.114299   0.173656  -0.300766  ;
 -0.112811  -0.587114   -1.14549   0.343596   0.173656     1.0582   -1.75418  ;
  0.542731    1.47603    3.05489  -0.746645  -0.300766   -1.75418    4.16758  ]
### 4. Give your assessment of the quality of the calibration.
  a. How close to zero is the sample mean of the error? Note that a common test of the size of a vector is its length (In mathematics, it’s norm).
The norm of our Mean error vector is 3.2290, which is relatively low and most of the error comes in the z direction.
  b. Are the position measurements more reliable than the orientation measurements?
On average, the orientation measurements were more accurate.

# Lab 8
## Report
### 1. Include properly labelled plots of both squares and the circle. These can be a single plot for each figure. Orient the plot to put the entire figure in the plane of the plot. Note that these plots are based on data from the camera

### 2. Analytically determine the upper bounding circle and the lower bounding circle for the 20-sided polygon you use to approximate a circle in your code for the robot.

### 3. Include plots of the actual (the robot) and theoretical (the simulation) squares.

### 4. Explain the rationale for your solution to the maximum square problem.

### 5. Report the smallest time required to traverse the maximum square.
