# Dev Container Setup for Lab Computers

Clone the repository in the `home/` directory without changing the name of the workspace folder.

```bash
git clone https://github.com/ENEE467/lab-workspace.git
```

## Building the Docker image

Start a terminal session and change to `.devcontainer/ral-lab/` directory

```bash
cd ~/lab-workspace/.devcontainer/ral-lab
```

Build the Docker image with the given arguments

```bash
docker build \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  --build-arg USERNAME=467-terp \
  -t enee467/lab_ws_image \
  -f ral-lab.Dockerfile \
  .
```

## Starting the Dev Container

Now opening the workspace folder in VSCode with the `ral-lab` Dev Container configuration will
start the container named after the workspace folder.

This page on [Read the Docs](https://enee467.readthedocs.io/en/latest/Setup.html#opening-the-workspace-in-visual-studio-code)
describes the steps in detail.

`lab-workspace` folder can now be deleted after testing.

## Description

The lab setup uses a more stable, finalized development environment which doesn't change very often.
Hence all the project-specific setup (i.e. package dependencies) is *baked* into the Docker image.

- This minimizes the setup time for containers.

- Allows the containers to be *ephemeral* to save storage resources on lab computers.

- Makes it easier to transfer the work to other lab computers by just moving the workspace folder
  (without the build artifacts).

>**Note to maintainers:** The Dockerfile and the container setup script for this configuration will
always be the final stage of applying the new changes and updates to the lab exercises.
