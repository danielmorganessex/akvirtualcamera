# akvirtualcamera, virtual camera for Mac and Windows

akvirtualcamera is virtual camera implemented as a DirectShow filter in Windows, and as a CoreMediaIO plugin in Mac.

## Features

* Supports emulated camera controls in capture devices (brightness, contrast, saturation, etc.).
* Configurable default picture in case no input signal available.
* **Webcam Splitting (New!):** Allows a single physical webcam to be split into multiple virtual camera outputs, all showing the same feed. This is managed via the `AkVCamManager` command-line tool. (Currently, macOS has base implementation, Windows is in progress).

## Webcam Splitting via AkVCamManager CLI (New Feature)

The `AkVCamManager` command-line tool now includes features to manage webcam splitting. This allows you to take one physical camera and create multiple virtual camera outputs from it.

**Note:** This feature is under development. macOS has a base implementation, while Windows support is still being built out. Frame conversion logic, especially on macOS, is undergoing refinement, so video quality in split outputs may vary during testing.

### Available Commands:

1.  **`list-physical-cameras`**
    *   **Purpose:** Displays a list of physical video capture devices (webcams) connected to your system. Use the ID or name from this list when configuring a split.
    *   **Usage:** `./AkVCamManager list-physical-cameras`

2.  **`split-webcam <physical_cam_id_or_name> <num_splits> [base_name_prefix]`**
    *   **Purpose:** Configures a physical camera to be split into a specified number of virtual cameras.
    *   **Arguments:**
        *   `<physical_cam_id_or_name>`: The unique ID or friendly name of the physical camera (from `list-physical-cameras`).
        *   `<num_splits>`: The number of virtual cameras to create (e.g., 2, 3).
        *   `[base_name_prefix]` (Optional): A prefix for the names of the created virtual cameras. Defaults to "SplitCam". Virtual cameras will be named e.g., "SplitCam 1", "SplitCam 2".
    *   **Usage:** `./AkVCamManager split-webcam "Integrated Camera" 2 MyVirtualCams`
    *   **Details:** This command registers the new virtual cameras with the system. It attempts to set a compatible default video format for them based on the physical camera's capabilities.

3.  **`start-split <physical_cam_id_or_name>`**
    *   **Purpose:** Starts capturing video from the specified physical camera and streams it to all its configured virtual split outputs.
    *   **Usage:** `./AkVCamManager start-split "Integrated Camera"`

4.  **`stop-split <physical_cam_id_or_name>`**
    *   **Purpose:** Stops capturing from the physical camera and halts streaming to its virtual splits. The virtual cameras remain registered.
    *   **Usage:** `./AkVCamManager stop-split "Integrated Camera"`

5.  **`remove-split <physical_cam_id_or_name>`**
    *   **Purpose:** Removes a webcam split configuration. This stops any active streaming for the split and unregisters all associated virtual cameras.
    *   **Usage:** `./AkVCamManager remove-split "Integrated Camera"`

### Example Workflow:

```bash
# List available physical cameras
./AkVCamManager list-physical-cameras

# Output might show something like:
# ID                             Name
# unique_id_cam1                 Integrated FaceTime HD Camera
# another_id_usb_cam             Logitech USB Cam

# Configure your FaceTime camera to be split into 2 virtual cameras
# named "MyCam 1" and "MyCam 2"
./AkVCamManager split-webcam "unique_id_cam1" 2 "MyCam"

# Start the splitting process
./AkVCamManager start-split "unique_id_cam1"

# Now, "MyCam 1" and "MyCam 2" should be available in applications
# and show the feed from your FaceTime camera.

# To stop the stream
./AkVCamManager stop-split "unique_id_cam1"

# To remove the split configuration and virtual cameras
./AkVCamManager remove-split "unique_id_cam1"
```

## Build and Install

Visit the [wiki](https://github.com/webcamoid/akvirtualcamera/wiki) for a comprehensive compile and install instructions.

## Downloads ##

[![Download](https://img.shields.io/badge/Download-Releases-3f2a7e.svg)](https://github.com/webcamoid/akvirtualcamera/releases)
[![Daily Build](https://img.shields.io/badge/Download-Daily%20Build-3f2a7e.svg)](https://github.com/webcamoid/akvirtualcamera/releases/tag/daily-build)
[![Total Downloads](https://img.shields.io/github/downloads/webcamoid/akvirtualcamera/total.svg?label=Total%20Downloads&color=3f2a7e)](https://tooomm.github.io/github-release-stats/?username=webcamoid&repository=akvirtualcamera)

## Donations ##

If you are interested in donating to the project you can look at all available methods in the [donations page](https://webcamoid.github.io/donations).

## Status

[![Linux MinGW](https://github.com/webcamoid/akvirtualcamera/actions/workflows/linux-mingw.yml/badge.svg)](https://github.com/webcamoid/akvirtualcamera/actions/workflows/linux-mingw.yml)
[![Mac](https://github.com/webcamoid/akvirtualcamera/actions/workflows/mac.yml/badge.svg)](https://github.com/webcamoid/akvirtualcamera/actions/workflows/mac.yml)
[![Windows MSYS](https://github.com/webcamoid/akvirtualcamera/actions/workflows/windows-msys.yml/badge.svg)](https://github.com/webcamoid/akvirtualcamera/actions/workflows/windows-msys.yml)
[![Windows MSVC](https://github.com/webcamoid/akvirtualcamera/actions/workflows/windows-vs.yml/badge.svg)](https://github.com/webcamoid/akvirtualcamera/actions/workflows/windows-vs.yml)
[![Build status](https://api.cirrus-ci.com/github/webcamoid/akvirtualcamera.svg)](https://cirrus-ci.com/github/webcamoid/akvirtualcamera)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/1cee2645a3604633a506a203fb8c3161)](https://www.codacy.com/gh/webcamoid/akvirtualcamera/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=webcamoid/akvirtualcamera&amp;utm_campaign=Badge_Grade)
[![Project Stats](https://www.openhub.net/p/akvirtualcamera/widgets/project_thin_badge.gif)](https://www.openhub.net/p/akvirtualcamera)

## Reporting Bugs

Report all issues in the [issues tracker](https://github.com/webcamoid/akvirtualcamera/issues).
