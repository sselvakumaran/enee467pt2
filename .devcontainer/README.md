# Dev Container Setup for PCs

Everything from building the Docker image to starting the container is handled by the Dev Containers extension using the configuration file.

Go to this page on Read the Docs for detaled steps.

### Description

This setup uses a multi-stage image that's minimal with core dependencies and does the project
specific setup (like installing package dependencies) in the container instead. So the behavior with
this setup is different from lab setup.

- While the image is small and simple, the container size is large. But only one container is
  used per machine in this scenario.

- The first time setup for starting the container is significantly longer since all the package
  dependencies are installed at this stage rather than while building the image.

- Container *stays* on the machine and is only started/stopped during the sessions.

However, it is flexible and quicker to add/modify the project-specific setup in the container setup
shell script rather than the Dockerfile. The container can be easily destroyed and rebuilt to test
the changes without touching the image.

Then the finalized project-specific setup in the container setup script can be transferred to the
lab Dockerfile and apply them to the image used on the lab computers.

> **Note to maintainers:** Use this setup when developing/modifying the lab exercises and then
transfer the finalized changes to the lab Docker setup.
