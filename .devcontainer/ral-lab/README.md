# Setting up the workspace on lab computers

## Building the Docker image

Start a terminal session and change to `.devcontainer/ral-lab/` directory

```bash
cd ~/<workspace-folder-name>/.devcontainer/ral-lab
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

## Starting the Devcontainer

Now opening the workspace folder in VSCode with the `ral-lab` devcontainer configuration will
start the container named after the workspace folder.

Open VSCode in the workspace folder from shell

```bash
code ~/<workspace-folder-name>
```
