# COVIS-CamCalib
Camera Calibration and Pose Computation using OpenCV and ViSP

Done as a part of the Computer Vision(COVIS) course, offered by Prof. Vincent Fremont at Ecole Centrale de Nantes.

### Explanation:

This project uses a set of images to calibrate the camera, by estimating the Intrinsic and Extrinsic Jacobians. We use the Perspective Projection Model of a camera as our fundamental model.
---

### Usage:
  1. Clone the repository onto your computer. You can either download the zip or use `git clone`
  1. Create the build directory and navigate into it `mkdir build && cd build`
  1. Run cmake to configure the project `cmake ..`
  1. Once configuration is done, you can use `make` to build the project. This will generate two executables: `./record` for saving the images, and `./calibration` for calibrating the camera.

---

### Requirements:

  * OpenCV : Open-Source Computer Vision Library. If you do not have it, you can use [the OpenCV website !](www.opencv.org) to download the library.
  * ViSP : Visual Servoing Platform. Visit [the ViSP website !](https://visp.inria.fr/) for download and build instructions

  We use CMAKE to configure our project. If you do not have it, then you can install it by using the command `sudo snap install cmake`
