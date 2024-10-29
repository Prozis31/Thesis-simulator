# Simulator for Thesis: UERANSIM-based Network Simulation with Orchestrator Implementation

This repository contains a simulator designed as part of a thesis project, aimed at evaluating network behaviors in Public Network (PN) and Non-Public Network (NPN) environments. Built upon UERANSIM, the simulator incorporates an orchestrator for automated management and coordination within network slices.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)


## Overview

This project serves as a research-grade network simulation framework, exploring the capabilities of 5G networks in both PN and NPN contexts. By utilizing **UERANSIM**, a renowned open-source 5G core and RAN simulator, alongside a custom orchestrator implementation, this simulator allows users to analyze and observe network slice management, performance metrics, and automated network orchestration.

### Purpose

The primary purpose of this simulator is to evaluate network performance, load balancing, and slice orchestration strategies in PN and NPN environments. The simulator provides insights into network behavior through different configurations and scenarios.

### Files Included

- `simulation_PN_NPN.py`: Core simulator logic for public and non-public network interactions.
- `simulation_NPN_PN.py`: Configuration and simulation logic for NPN in relation to PN environments.


## Features

- **UERANSIM Integration**: Leveraging UERANSIM for simulating 5G core and RAN interactions.
- **Network Slice Orchestration**: A custom orchestrator to manage and automate network slice configurations and adjustments.
- **Public & Non-Public Network Modes**: Simulation of PN and NPN behaviors, allowing for analysis in both environments.
- **Configurable Scenarios**: Flexibility to set up various network scenarios, modify network parameters, and observe performance metrics.
- 
### Orchestrator Functions

The orchestrator included in the simulator provides various automated management functions, such as:

* Network slice creation and deletion
* Performance monitoring
* Automated scaling and resource allocation

## Requirements

To successfully run the simulator, ensure the following dependencies are met:

- **UERANSIM** (latest version) — Follow installation instructions from the [UERANSIM GitHub repository](https://github.com/aligungr/UERANSIM).
- **Python 3.8+**
- **pip** for Python package management
