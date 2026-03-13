# Status

Local image processing and streaming toolkit for robotics and computer vision experiments.

This repository is a work-in-progress experimental project.

## Goal

PX4 and Ardupilot compatibility in one rpi image.

## Overview

`img_local` is a small experimental project designed to work with image data locally.  
The repository contains tools and scripts for capturing, processing, and handling images in a local environment, with a focus on robotics and drone-related workflows.

The project combines several technologies and languages:

- **Python** — scripting and automation
- **C++** — performance-critical components
- **JavaScript / HTML** — lightweight UI
- **Shell scripts** — environment and pipeline automation

The repository structure suggests usage in development setups where image data must be processed or streamed locally, for example in robotics.

## Repository Structure

- config/   Configuration files
- drone/    Drone-related code and components
- scripts/  Utility scripts for running and managing the project
- Makefile  Build and execution helpers

## Requirements

Typical environment for running the project:

- Linux / Windows WSL
- Python 3.x
- C++ toolchain (gcc / clang)
- Make

## Usage

Clone the repository:

```bash
git clone https://github.com/arutofu/img_local.git
cd img_local
make help
```

Additional scripts are available in the scripts/ directory depending on the task you want to perform.
