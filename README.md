# Robotics - Estimation and Measurement
## 1. Color Learning and Target Detection
Estimating a Gaussian model for ball color and developing a ball detection function based on the color model. The figure below depicts an example of an input image, a binary image based on the color model, and a segmented binary image after filtering.
![ball](/images/ball.jpg)

## 2. Kalman Filter and Target Tracking
Implementing a Kalman filter for ball tracking in 2D space. Imagine a soccer robot wants to predict the ball’s location constantly for planning its next motion. The figure below depicts an example of ball location measurements, location estimates, and a predicted location at certain time.
![Ball-Location](/images/Ball-Location.jpg)

## 3. Occupancy Grid Mapping
Implementing the Occupancy Grid Mapping algorithm for a 2D floor map by incorporating range sensor readings and known poses at each of the measurement times in order to build a map. The figure below depicts an example of range measurements (green dots) from a mobile robot (red dot with an indicated heading) and an intermediate occupancy grid map.
![OGM](/images/OGM.jpg)
