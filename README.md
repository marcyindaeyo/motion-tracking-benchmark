# Motion Tracking Benchmark for Wearable and Vision Systems

Benchmarking an IMU-based motion tracking system against a vision-based tracking system using a unified evaluation pipeline.

---

## Overview

This project evaluates upper-body motion tracking using:

- IMU-based system (**BodyRig**)
- Vision-based system (**MediaPipe + RealSense**)
- Optical tracking (**HTC VIVE**) as ground truth

The goal is to evaluate how well low-cost tracking systems perform under realistic movement conditions.

<img src="figures/setup.PNG" width="500">

---

## Systems Compared

### IMU-based system
BodyRig wearable tracking suit with 5 IMU sensors.

<img src="figures/IMU.PNG" width="350">

### Vision-based system
MediaPipe Pose + Intel RealSense D455.
<img src="figures/camera.PNG" width="600">

### Ground truth
HTC VIVE optical tracking system.

---

## Experiment Design

Participants (N=9) performed 10 upper-body tasks under normal and challenging conditions:
- basic tasks: static hold, point-to-point, circle drawing, fast waving
- applied tasks: bottle pick & place, USB insertion, simulated cutting
- challenging: visual occlusion, electromagnetic interference (EMI). These perturbations allow evaluation of robustness in real-world scenarios. 



## Data Processing Pipeline

<img src="figures/pipeline.png" width="700">

Key processing steps:

**Time Synchronization**

Data from multiple PCs were aligned using timestamp exchange and cross-correlation to ensure frame-level correspondence across systems. 

**Spatial Alignment**

All trajectories were transformed into a common coordinate frame.
A Kabsch-based rigid transformation was used to align the coordinate systems of the IMU and camera outputs with the optical tracking reference. 


---

## Evaluation Metrics

The benchmark focuses on four performance dimensions:

- **Accuracy**: 3D wrist position error compared to VIVE ground truth, measured with MAE and RMSE
- **Precision**: trial-to-trial repeatability across repeated motions
- **Latency**: delay between real movement and system output
- **Bandwidth**: sampling rate stability and temporal jitter

---




## Code Structure

```text
acquisition/
  camera_recording.py        # RGB-D data collection with MediaPipe + RealSense

src/
  preprocessing/
    align_sources.m          # global and local time alignment
    process_task_segment.m   # task-wise segmentation

  alignment/
    Kabsch.m                 # spatial alignment to reference frame

  pipeline/
    run_pipeline.m           # loads raw data and performs alignment

  evaluation/
    evaluate_accuracy.m      # computes MAE / RMSE
    evaluate_precision.m     # computes repeatability metrics


``` 

## Example Results

Example outputs include:

- 3D trajectory comparison across systems  

<img src="figures/accuracy.PNG" width="600">

- Time-series tracking plots  
- Accuracy comparison across tasks  
- Frame rate stability plots  

<img src="figures/bandwidth.png" width="600">



## Key Contributions

- A benchmarking framework for evaluating wearable and vision-based motion tracking systems  
- A multimodal data alignment pipeline integrating IMU, vision, and optical tracking  
- Quantitative evaluation across accuracy, latency, precision, and bandwidth  
- Analysis of tracking performance under realistic perturbations  


## Notes

The original dataset and full experimental code are not publicly available due to collaboration agreements with the research lab and industry partner.  

Example scripts and documentation are provided to illustrate the benchmarking pipeline.


