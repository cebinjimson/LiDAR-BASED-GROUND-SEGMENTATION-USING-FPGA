# Real-Time LiDAR-Based Ground Segmentation on FPGA

A hardware-accelerated perception pipeline designed to segment raw 3D LiDAR point clouds into ground and non-ground (obstacle) points in real time. By leveraging parallel processing, fixed-point arithmetic, and pipelined streaming architectures on FPGA hardware, this system achieves deterministic low latency and high throughput for autonomous vehicles and robotics applications.

---

## Overview

LiDAR point clouds contain large volumes of data per frame, with 50% to 70% of points typically representing the road surface. Processing full point clouds on general-purpose CPUs or GPUs can introduce high latency and power overhead. 

This project implements an optimized hardware pipeline that:
- Filters ground returns to isolate potential obstacles.
- Extracts drivable surface profiles.
- Operates at sensor line-rate with low power consumption and microsecond-level latency.

---

## Features

- **Hardware Acceleration:** Pipelined architecture implemented in Verilog / SystemVerilog.
- **Fixed-Point Quantization:** Optimized word-length representations to reduce DSP slice usage while preserving sub-centimeter accuracy.
- **Line Buffer Streaming:** On-chip BRAM buffers to perform local neighborhood operations without stalling for full frame transfers from external memory.
- **Python Verification & Simulation:** High-level algorithmic reference model, visualization utilities, and automated testbench generation.

---

## System Architecture

```text
  [Raw LiDAR Data Stream (Ethernet / AXI-Stream)]
