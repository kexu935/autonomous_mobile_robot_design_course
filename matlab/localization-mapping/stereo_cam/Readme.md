# Depth Estimation from Stereo Camera

1. Load Camera Parameters
-------------------------

The following camera parameters are considered:

**Parameters of Two Cameras**
*        CameraParameters1: 
*        CameraParameters2: 

1.  Camera Intrinsics
*                 IntrinsicMatrix: [3x3 double]
*                    FocalLength: [530.3463 528.7529]
*                PrincipalPoint: [316.8934 234.5534]
*                         Skew: 0

2. Lens Distortion
*            RadialDistortion: [-0.3438 0.1400]
*        TangentialDistortion: [0 0]

3. Camera Extrinsics
*            RotationMatrices: [3x3x12 double]
*         TranslationVectors: [12x3 double]

4. Accuracy of Estimation
 *       MeanReprojectionError: 0.2172
*          ReprojectionErrors: [48x2x12 double]
*           ReprojectedPoints: [48x2x12 double]

5. Calibration Settings
       *                 NumPatterns: 12
       *                 WorldPoints: [48x2 double]
       *                  WorldUnits: 'mm'
       *                EstimateSkew: 0
       * NumRadialDistortionCoefficients: 2
       * EstimateTangentialDistortion: 0

   **Inter-camera Geometry**
*        RotationOfCamera2: 
*     TranslationOfCamera2: 
*        FundamentalMatrix: 
*          EssentialMatrix: 

   **Accuracy of Estimation**
*    MeanReprojectionError: 

 **Calibration Settings**
*              NumPatterns: 
*              WorldPoints: 
*               WorldUnits: 


2. Rectification
-------------------------
Image rectification is a transformation process used to project two-or-more images onto a common image plane. This process has several degrees of freedom and there are many strategies for transforming images to the common plane. It is used in computer stereo vision to simplify the problem of finding matching points between images (i.e. the correspondence problem).


3. Disparity Computation
-------------------------
In rectified stereo images any pair of corresponding points are located on the same pixel row. For each pixel in the left image compute the distance to the corresponding pixel in the right image. This distance is called the disparity, and it is proportional to the distance of the corresponding world point from the camera.

4. Scene Reconstruction
-------------------------
pointCloud = reconstructScene(disparityMap,stereoParams) returns an array of 3-D world point coordinates that reconstruct a scene from a disparity map. The stereoParams input must be the same input that you use to rectify the stereo images corresponding to the disparity map.

5. People Detection
-------------------------
People detection using Histogram of Oriented Gradient (HOG) features and a trained Support Vector Machine classifier. It detects unoccluded people in an upright position. 

6. Determine the Distance of Each Person to the Camera
-------------------------
Find the 3-D world coordinates of the centroid of each detected person and compute the distance from the centroid to the camera in meters.

