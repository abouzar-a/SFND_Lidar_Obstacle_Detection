[//]: # (Image References)

[image1]: ./images/pcdstream_1.gif
[image2]: ./images/ransac-linie-animiert.gif
[image3]: ./images/Ransac2D.jpg
[image4]: ./images/kdtree5.jpg
[image5]: ./images/KdTree2D.jpg
[image6]: ./images/final_video.jpg

## Eduardo Ribeiro de Campos - March 2021

# Lidar Obstacle Detection Project
## Sensor Fusion Nanodegree Program

![Lidar Stream][image1]

This project, part of the Sensor Fusion Nanodegree, focuses on obstacle detection using Lidar data. The course was taught by [Michael Maile](https://www.linkedin.com/in/michael-maile-ab7a078/), Director of Sensor Fusion & Localization, and [Aaron Brown](https://www.linkedin.com/in/awbrown90/), Senior AV Software Engineer, both from [MBRDNA](https://www.mbrdna.com/) (Mercedes-Benz Research & Development North America, Inc).

The course explored best practices for processing Lidar data, which involved building a pipeline encompassing the following key steps:

1. **Segmentation (RANSAC)**
2. **Clustering (KD-Tree)**
3. **Bounding Boxes**
4. **Filtering and Downsampling (Voxel Grid & Region of Interest)**

## 1. Segmentation

Identifying obstacles within Lidar data begins with segmenting points that correspond to the road surface. Non-obstacle objects, like the road itself, need to be distinguished from actual obstacles. This is achieved using Planar Segmentation with the `RANSAC` (Random Sample Consensus) algorithm.

The gif below demonstrates the RANSAC method in action for fitting a line despite the presence of outliers.

![RANSAC Line Fitting][image2]

In this process, a minimal subset of points (two for a line, three for a plane) is selected to fit a model. The algorithm then counts inliers—points close to the model—across multiple iterations. The iteration with the most inliers is deemed the best model. This method is implemented in [line_ransac2d.cpp](./src/quiz/ransac/line_ransac2d.cpp). To run the simulation:

```bash
cd src/quiz/ransac/build
./quizRansac
