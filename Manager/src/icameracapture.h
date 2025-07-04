#ifndef ICAMERACAPTURE_H
#define ICAMERACAPTURE_H

#include <string>
#include <vector>
#include <functional>
#include "VCamUtils/src/videoframe.h" // Assuming VideoFrame is accessible

namespace AkVCam {

// Forward declaration
class VideoFrame;

// Callback function type for when a new frame is captured
using FrameCaptureCallback = std::function<void(const VideoFrame& frame)>;

class ICameraCapture {
public:
    virtual ~ICameraCapture() = default;

    // Lists available physical cameras (can be a static method or require an instance)
    // For now, let's assume it's part of the instance, or a separate utility.
    // For simplicity, the existing list_physical_cameras_impl can be used separately first.

    // Opens a camera device by its ID
    virtual bool open(const std::string& deviceId) = 0;

    // Closes the currently open camera device
    virtual void close() = 0;

    // Starts the video stream. Captured frames will be delivered via the callback.
    virtual bool startStream(FrameCaptureCallback callback) = 0;

    // Stops the video stream
    virtual void stopStream() = 0;

    // Checks if the camera stream is currently active
    virtual bool isStreaming() const = 0;

    // (Optional) Gets a list of supported video formats for the open camera
    // virtual std::vector<VideoFormat> getSupportedFormats() = 0;

    // (Optional) Sets a specific video format
    // virtual bool setFormat(const VideoFormat& format) = 0;

    // Returns the ID of the currently opened device, if any
    virtual std::string getOpenedDeviceId() const = 0;

    // Gets a list of supported video formats for the open camera
    // This might be called after open()
    virtual std::vector<VideoFormat> getSupportedFormats() = 0;

};

// Factory function to create a platform-specific camera capture instance
std::unique_ptr<ICameraCapture> create_platform_camera_capture();

} // namespace AkVCam

#endif // ICAMERACAPTURE_H
