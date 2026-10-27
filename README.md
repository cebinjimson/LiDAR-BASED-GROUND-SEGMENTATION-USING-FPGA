# ⚡ Real-Time LiDAR Ground Segmentation on FPGA

[![FPGA](https://img.shields.io/badge/Target-AMD%20Xilinx%20Zynq%20%7C%20UltraScale%2B-red.svg)](#)
[![HDL](https://img.shields.io/badge/Language-Verilog%20%7C%20SystemVerilog-blue.svg)](#)
[![Python](https://img.shields.io/badge/Python-3.8%2B-brightgreen.svg)](#)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-orange.svg)](#contributing)

A high-throughput, sub-millisecond perception engine for autonomous navigation. This repository accelerates 3D point cloud ground segmentation on FPGA hardware, stripping millions of terrain points per second to isolate drivable surfaces and potential obstacles at sensor line-rate.

---

## 🚀 Key Highlights

* **Ultra-Low Latency:** Streaming architecture processes data at wire speed via line buffers, delivering **microsecond-level deterministic response** without waiting for complete frame dumps.
* **Hardware-Optimized Arithmetic:** Implements parameterized fixed-point quantization (Q-format) to slash DSP slice utilization and power draw while preserving sub-centimeter spatial accuracy.
* **Zero Frame-Buffer Bottlenecks:** Uses on-chip BRAM/URAM circular buffers to perform sliding-window elevation gradient calculations directly on incoming scanlines.
* **Dual-Track Verification:** End-to-end Python golden model (with 3D visualizers) paired with bit-accurate RTL testbenches to validate algorithm behavior before synthesis.

---

## 📊 Performance at a Glance

| Metric | Target Specification |
| :--- | :--- |
| **Input Ingestion** | AXI4-Stream / Raw Sensor Packet Stream |
| **Throughput** | $\ge$ 50–100 FPS (comfortably exceeds typical 10–20 Hz LiDAR feeds) |
| **Data Representation** | Quantized Fixed-Point (e.g., Q12.4 / Q16.16) |
| **Memory Footprint** | Zero external DDR roundtrips during segmentation |
| **Typical Power Budget** | $\sim$2–10 W (embedded FPGA profile vs. 50–150 W automotive GPUs) |

---

## 🛠 Hardware Pipeline

```text
       Raw LiDAR Stream (Ethernet / AXI4-Stream)
                          │
                          ▼
        ┌───────────────────────────────────┐
        │   Coordinate Projection & Scale   │  ◄── Fixed-point conversion
        └───────────────────────────────────┘
                          │
                          ▼
        ┌───────────────────────────────────┐
        │   Sliding Line Buffers (BRAM)     │  ◄── Neighborhood cache (no DDR stalls)
        └───────────────────────────────────┘
                          │
                          ▼
        ┌───────────────────────────────────┐
        │   Parallel Gradient & Slope Core  │  ◄── Pipelined DSP arithmetic units
        └───────────────────────────────────┘
                          │
                          ▼
        ┌───────────────────────────────────┐
        │    Dynamic Threshold & Filter     │  ◄── Elevation & delta-angle classification
        └───────────────────────────────────┘
                          │
                          ▼
        [Classified Stream: Ground / Non-Ground Tags]
