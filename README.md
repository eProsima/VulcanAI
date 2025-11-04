# VulcanAI

<div align="justify">
VulcanAI is a library that provides a framework to easily and flexibly create powerful AI applications.
Leveraging LLM capabilities, VulcanAI acts as an intelligent assistant,
capable of understanding and executing complex tasks by decomposing them into manageable sub-tasks in any environment.
VulcanAI excels at planning and reasoning, providing a deeper level of human-context understanding to robotic systems.
VulcanAI is completely compatible with <a href="https://docs.vulcanexus.org/en/jazzy/" target="_blank">Vulcanexus</a>
and works marvelously to control and even debug ROS 2 robots and applications.
</div>
<br>
<div align="justify">
It relies on the concept of tools to extend its capabilities and adapt to different environments.
Tools are special components that provide specific functionalities to VulcanAI,
allowing its agents to interact with the external world.
</div>
<br>

> \[!IMPORTANT]\
> VulcanAI is currently in active development and current version is in Beta stage. New features and improvements are being added regularly and API might suffer changes.

## VulcanAI Installation

The following section provides step-by-step instructions to install and configure VulcanAI.

### Requirements

- Python 3.10 or higher

### Virtual environment

It is recommended to create a virtual environment
before installing VulcanAI to avoid potential conflicts with other Python packages.

To create a virtual environment, run the following command:

```bash
python3 -m venv vulcanai_venv && \
source vulcanai_venv/bin/activate
```

If venv or pip are not installed in your system,
you can install them using your package manager with the following commands:

```bash
sudo apt update && \
sudo apt install python3-venv python3-pip
```

### Installation via pip

The fastest and simplest way to install VulcanAI is using pip. To do so, open a terminal and run the following command:

```bash
pip install eprosima-vulcanai
```

This will install the latest stable version of VulcanAI and all its dependencies.

> \[!WARNING]\
> Pip installation will be available soon.

### Installation from source

If you prefer to install VulcanAI from source, follow these steps:

1. Clone the repository in the desired location and navigate to it:

```bash
git clone https://github.com/eProsima/VulcanAI.git VulcanAI && cd VulcanAI
```

2. Install VulcanAI:

```bash
python3 -m pip install .
```

### Verification

To check the installation, you can run the following command
to see if pip is able to list the package:

```bash
pip show eprosima-vulcanai
```


### Running VulcanAI from virtual environment with Vulcanexus

Virtual environments have their own Python path,
so Vulcanexus ROS packages will not be found when running VulcanAI from the virtual environment.
This section explains how to properly fix this to configure the environment to run VulcanAI with Vulcanexus.

To run VulcanAI with Vulcanexus, you need to have a Vulcanexus distribution installed in your system.
You can follow the official Vulcanexus installation guide for
[binaries](https://vulcanexus--256.org.readthedocs.build/en/256/rst/installation/linux_binary_installation.html#linux-binary-installation)
or [docker](https://vulcanexus--256.org.readthedocs.build/en/256/rst/installation/docker.html#docker-installation)

Once you have a Vulcanexus distribution installed and a virtual environment for VulcanAI created,
you need to append the VulcanAI virtual environment’s Python path to the PYTHONPATH environment variable.
In this way, both Vulcanexus and VulcanAI will be able to find the required Python packages.

To do so, source Vulcanexus and then run the following command in the terminal where you want to run VulcanAI:

```bash
source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && \
export PYTHONPATH='/<path/to/vulcanai_venv>/lib/python3.x/site-packages':$PYTHONPATH
```