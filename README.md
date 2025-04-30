# ROBOTIS OP3 - Webots ROS 2 Simulation Workspace

This workspace contains ROS 2 packages for simulating the ROBOTIS OP3 robot in the Webots simulator and controlling it using the Robotis framework and custom nodes.

## Project Structure

*   `src/`: Contains the source code for all ROS 2 packages.
    *   `ROBOTIS-OP3-Simulations/op3_webots_ros2/`: Interface between ROS 2 and the Webots OP3 model.
    *   `ROBOTIS-OP3/`: Official high-level modules (manager, walking, etc.).
    *   `ROBOTIS-Framework/`: Core Robotis libraries.
    *   `ROBOTIS-Framework-msgs/`: Messages for the framework.
    *   `ROBOTIS-OP3-msgs/`: Messages specific to OP3 modules.
    *   `ROBOTIS-Math/`: Math library.
    *   `webots_ros2/`: Core Webots ROS 2 interface packages.
    *   `op3_get_up_behavior/`: Custom node for get-up behavior.
    *   `yolo_detector_node/`: Custom node for YOLO detection.
    *   `yolo_visualizer_node/`: Custom node to visualize YOLO detections.
    *   `op3_joint_demux/`: Bridge node for joint commands.
    *   `bascorro_demo/`: Demo package.
    *   *(Add other custom packages here)*
*   `scripts/`: Utility scripts for building the workspace.
*   `docker/`: Contains files related to the Docker setup.
    *   `entrypoint.sh`: Script executed when the Docker container starts.
*   `Dockerfile`: Defines how to build the Docker image for this project.
*   `.dockerignore`: Specifies files/directories to exclude from the Docker build context.

## Prerequisites

1.  **Docker:** Install Docker Desktop for Windows (or Docker Engine on Linux). Ensure the Docker daemon is running. [https://www.docker.com/products/docker-desktop/](https://www.docker.com/products/docker-desktop/)
2.  **WSL2:** Ensure WSL2 is installed and integrated with Docker Desktop.
3.  **Webots:** Install Webots R2023b or compatible on your Windows host machine. [https://cyberbotics.com/](https://cyberbotics.com/)
4.  **(Optional) X Server for Windows:** If you need to run GUI applications (like `rqt_image_view`) from *inside* the Docker container (advanced), install an X Server like VcXsrv or X410 on Windows and configure it. This README currently assumes GUIs like Webots run natively on Windows.

## Docker Setup

This project uses Docker to provide a consistent build and runtime environment based on ROS 2 Jazzy.

### Building the Docker Image

1.  Open a terminal (like PowerShell or CMD) **on your Windows host** in the root directory of this workspace (`ros2_ws`).
2.  Run the Docker build command:

    ```bash
    docker build -t op3-jazzy-ws .
    ```

    *   `-t op3-jazzy-ws`: Tags the image with the name `op3-jazzy-ws`. You can change this tag.
    *   `.`: Specifies the current directory as the build context (where the `Dockerfile` is located).
    *   This process will take a while the first time as it downloads the base image and installs all dependencies. Subsequent builds will be faster due to caching.

## Running the Simulation with Docker

This setup runs the ROS 2 control stack inside the Docker container and connects to the Webots simulator running natively on your Windows host.

### Steps:

1.  **Start Webots:** Launch Webots on Windows. Open the specific world file for this project:
    *   File -> Open World... -> Navigate to `path\to\your\ros2_ws\src\ROBOTIS-OP3-Simulations\op3_webots_ros2\worlds\robotis_op3_extern.wbt`
    *   Ensure the simulation isn't paused and the console shows it's waiting for external controller connections (check the OP3 robot node's `controller` field is `<extern>`).

2.  **Find Host IP Address:** The Docker container needs the IP address of your Windows host *as seen from within WSL2*.
    *   Open a **WSL2 terminal** (e.g., Ubuntu).
    *   Run the command:
        ```bash
        ip route | grep default | awk '{print $3}'
        ```
    *   Note down the IP address displayed (e.g., `172.25.112.1`).

3.  **Run the Docker Container:**
    *   Open a terminal (PowerShell or CMD) **on your Windows host**.
    *   Execute the `docker run` command, replacing `<HOST_IP_FROM_WSL>` with the IP address you found in the previous step:

        ```bash
        docker run -it --rm \
          --network host \
          -e WEBOTS_CONTROLLER_URL="tcp://<HOST_IP_FROM_WSL>:1234/ROBOTIS%20OP3/" \
          op3-jazzy-ws \
          ros2 launch op3_webots_ros2 robot_launch2.py
        ```

    *   **Explanation of Options:**
        *   `-it`: Interactive TTY (allows seeing logs, using Ctrl+C).
        *   `--rm`: Removes the container when it stops.
        *   `--network host`: Shares the host's network stack for easy connection to Webots.
        *   `-e WEBOTS_CONTROLLER_URL=...`: Sets the crucial environment variable for the `op3_extern_controller` to find Webots. **Remember to insert the correct IP and ensure the robot name encoding (`ROBOTIS%20OP3`) matches the one in Webots.**
        *   `op3-jazzy-ws`: The name of the Docker image you built.
        *   `ros2 launch op3_webots_ros2 robot_launch2.py`: The command executed inside the container via the entrypoint script. This launches your main ROS 2 stack.

4.  **Interact (if applicable):**
    *   If your launch file starts `teleop_twist_keyboard`, a separate `xterm` window might appear *on your Windows desktop* (if an X Server is running and configured) or you might see its output/input request in the main Docker terminal (depending on exact setup). Focus the teleop window/terminal to send commands.
    *   If you launched visualization nodes, you might need to run `rqt` or `rviz2` either natively on Windows (connecting to the ROS topics exposed via host networking) or inside the container (requires X server setup).

5.  **Stop the Container:** Press `Ctrl+C` in the terminal where you ran `docker run`.

## Development Workflow with Docker

For active development:

1.  **Modify Code:** Edit code in the `src/` directory on your Windows host using your preferred editor (e.g., VS Code).
2.  **Rebuild Inside Container (Faster):**
    *   Start an interactive bash shell in a *new* container, mounting your source code:
        ```bash
        docker run -it --rm \
          --network host \
          -v "%cd%\src:/ros2_ws/src" \
          op3-jazzy-ws \
          bash
        ```
        *(Note: `%cd%\src` works in CMD/PowerShell on Windows to mount the current directory's `src` folder. Adjust path if needed.)*
    *   Inside the container's bash shell:
        ```bash
        # The entrypoint already sourced everything
        cd /ros2_ws
        colcon build --symlink-install --packages-select <package_you_modified>
        # Test run your nodes or launch files here
        exit
        ```
3.  **Rebuild Image (If Dockerfile or Dependencies Change):** If you modify the `Dockerfile` or need to install new system/python dependencies, rebuild the entire image using `docker build -t op3-jazzy-ws .`.

## Notes

*   Ensure Docker Desktop is configured to use the WSL2 backend.
*   Network configuration (`--network host`) might differ or require adjustments in pure Linux environments or complex network setups.
*   Running GUI applications from Docker usually requires additional setup (X11 forwarding, configuring `DISPLAY` environment variable).