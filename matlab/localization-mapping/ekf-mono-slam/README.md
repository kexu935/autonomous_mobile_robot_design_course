#ekf-mono-slam

Here the **EKFmonocularSLAM** package is simply redistributed

#EKFmonocularSLAM

EKFmonocularSLAM contains Matlab code for EKF SLAM from a 6 DOF motion monocular image sequence. The algorithm takes as input a monocular image sequence and its camera calibration and outputs the estimated camera motion and a sparse map of salient point features. The code includes state-of-the-art contributions to EKF SLAM from a monocular camera: inverse depth parametrization for 3D points and efficient 1-point RANSAC for spurious rejection.
Further information

**Authors**
Javier Civera; J. M. M. Montiel;

**Long Description**
This code contains a complete EKF SLAM system from a monocular sequence, taking as input a sequence from a monocular camera and estimating the 6 DOF camera motion and a sparse 3D map of salient point features. The key contributions in this code are two:

1) INVERSE DEPTH POINT PARAMETRIZATION. This point model suits the projective nature of the camera sensor, allowing to code in the SLAM map very distant points naturally captured by a camera. It is also able to code infinite depth uncertainty, allowing undelayed initialization of point features from the first frame they were seen. At any step, the measurement equation shows the high degree of linearity demanded by the EKF.

2) 1-POINT RANSAC. Robust data association has proven to be a key issue in any SLAM algorithm. 1-Point RANSAC is an algorithm based on traditional random sampling but adapted to the EKF. Specifically, it exploits probabilistic information from the EKF to greatly improve the efficiency over standard RANSAC. 


**Hardware/Software Requirements**
Matlab. 

**Papers Describing the Approach**
Javier Civera, Oscar G. Grasa, Andrew J. Davison and J. M. M. Montiel: 1-Point RANSAC for EKF Filtering. Application to Real-Time Structure from Motion and Visual Odometry , Journal of Field Robotics, 2010 (link)

Javier Civera, Andrew J. Davison, J. M. M. Montiel: Inverse Depth Parametrization for Monocular SLAM , IEEE Transactions on Robotics, 2008 (link)

**License Information**
This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
The authors allow the users of OpenSLAM.org to use and modify the source code for their own research. Any commercial application, redistribution, etc has to be arranged between users and authors individually and is not covered by OpenSLAM.org.

EKFmonocularSLAM is licenced under GNU GPL. See http://www.gnu.org/copyleft/gpl.html for details.

*** Copyright and V.i.S.d.P.: Javier Civera; J. M. M. Montiel; *** 

