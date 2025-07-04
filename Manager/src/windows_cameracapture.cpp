#include "icameracapture.h"
#include "VCamUtils/src/logger.h" // For AkLog

#ifdef _WIN32
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <shlwapi.h>
// Include other necessary headers

// Forward declaration for a potential callback class
class CaptureCallback;

namespace AkVCam {

class WindowsCameraCapture : public ICameraCapture {
public:
    WindowsCameraCapture();
    ~WindowsCameraCapture() override;

    bool open(const std::string& deviceId) override;
    void close() override;
    bool startStream(FrameCaptureCallback callback) override;
    void stopStream() override;
    bool isStreaming() const override;
    std::string getOpenedDeviceId() const override;
    std::vector<VideoFormat> getSupportedFormats() override;

private:
    IMFSourceReader* pSourceReader_ = nullptr;
    FrameCaptureCallback frame_callback_ = nullptr;
    bool streaming_ = false;
    std::string opened_device_id_;
    // Add other MF specific members, e.g., for handling callbacks, async operations
    // IMFMediaSource *pMediaSource_ = nullptr; // Alternative to SourceReader for more control
    // CaptureCallback* pCaptureCallback_ = nullptr; // Custom IMFSourceReaderCallback
};

WindowsCameraCapture::WindowsCameraCapture() {
    // It's good practice to initialize COM for the thread that will use MF.
    // However, list_physical_cameras_impl already does CoInitialize/Uninitialize.
    // If this class runs on a different thread or has a long lifetime,
    // it might need its own COM management. For now, assume COM is initialized
    // by the caller or globally for the process thread where these methods are invoked.
    // HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    // if (FAILED(hr)) {
    //     AkLogError() << "WindowsCameraCapture: Failed to initialize COM: " << std::hex << hr << std::endl;
    // }
    // MFStartup is also typically called once per process. Assuming it's handled.
}

WindowsCameraCapture::~WindowsCameraCapture() {
    close();
    // if (SUCCEEDED(CoUninitialize())) {} // Balance CoInitialize if done in constructor
}

bool WindowsCameraCapture::open(const std::string& deviceId) {
    if (streaming_) {
        AkLogError() << "WindowsCameraCapture::open: Cannot open while streaming." << std::endl;
        return false;
    }
    close(); // Ensure any previous state is cleared

    AkLogInfo() << "WindowsCameraCapture::open: Attempting to open device ID: " << deviceId << std::endl;

    HRESULT hr = S_OK;
    IMFAttributes* pAttributes = NULL;
    IMFMediaSource* pMediaSource = NULL; // Needed to create the source reader from symbolic link

    // Convert std::string deviceId (UTF-8) to WCHAR*
    std::wstring wDeviceId;
    int len = MultiByteToWideChar(CP_UTF8, 0, deviceId.c_str(), -1, NULL, 0);
    if (len > 0) {
        wDeviceId.resize(len -1); // -1 for null terminator
        MultiByteToWideChar(CP_UTF8, 0, deviceId.c_str(), -1, &wDeviceId[0], len);
    } else {
        AkLogError() << "WindowsCameraCapture::open: Failed to convert deviceId to WCHAR." << std::endl;
        return false;
    }

    // Create attributes for source reader.
    hr = MFCreateAttributes(&pAttributes, 1);
    if (FAILED(hr)) {
        AkLogError() << "WindowsCameraCapture::open: MFCreateAttributes failed: " << std::hex << hr << std::endl;
        goto done;
    }

    // TODO: Set up asynchronous callback if not using blocking ReadSample
    // hr = pAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, pCaptureCallback_);
    // if (FAILED(hr)) { ... }


    // Create a media source for the device.
    hr = MFCreateDeviceSource(wDeviceId.c_str(), &pMediaSource);
    if (FAILED(hr)) {
        AkLogError() << "WindowsCameraCapture::open: MFCreateDeviceSource for " << deviceId << " failed: " << std::hex << hr << std::endl;
        goto done;
    }

    // Create the source reader.
    hr = MFCreateSourceReaderFromMediaSource(pMediaSource, pAttributes, &pSourceReader_);
    if (FAILED(hr)) {
        AkLogError() << "WindowsCameraCapture::open: MFCreateSourceReaderFromMediaSource failed: " << std::hex << hr << std::endl;
        goto done;
    }

    // At this point, the device is "opened" in the sense that we have a source reader.
    // We might want to select a default media type here or in startStream.
    // For now, let's defer detailed format selection to getSupportedFormats/startStream.

    opened_device_id_ = deviceId;
    AkLogInfo() << "WindowsCameraCapture::open: Successfully created source reader for " << deviceId << std::endl;

done:
    if (pMediaSource) pMediaSource->Release();
    if (pAttributes) pAttributes->Release();
    if (FAILED(hr)) {
        close(); // Cleanup if anything failed
        return false;
    }
    return true;
}

void WindowsCameraCapture::close() {
    AkLogInfo() << "WindowsCameraCapture::close: Closing device " << opened_device_id_ << std::endl;
    stopStream(); // Ensure streaming is stopped first
    if (pSourceReader_) {
        pSourceReader_->Release();
        pSourceReader_ = nullptr;
    }
    opened_device_id_.clear();
    // if (pCaptureCallback_) { pCaptureCallback_->Release(); pCaptureCallback_ = nullptr; }
}

bool WindowsCameraCapture::startStream(FrameCaptureCallback callback) {
    if (!pSourceReader_) {
        AkLogError() << "WindowsCameraCapture::startStream: Source reader not initialized. Call open() first." << std::endl;
        return false;
    }
    if (streaming_) {
        AkLogWarn() << "WindowsCameraCapture::startStream: Already streaming." << std::endl;
        return true;
    }

    frame_callback_ = callback;
    AkLogInfo() << "WindowsCameraCapture::startStream: Starting stream for " << opened_device_id_ << std::endl;

    // TODO: Configure the media type for the stream.
    // This involves:
    // 1. Enumerating available types from pSourceReader_.
    // 2. Selecting one (e.g., a common RGB or YUY2 format).
    // 3. Calling pSourceReader_->SetCurrentMediaType() for the video stream.
    // This is critical. For now, it might pick a default, or fail if no default is suitable.
    // Example (simplified):
    // IMFMediaType *pCurrentType = nullptr;
    // hr = pSourceReader_->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &pCurrentType);
    // if(SUCCEEDED(hr)) { AkLogInfo() << "Current default type selected by MF"; pCurrentType->Release(); }
    // else { AkLogError() << "Could not get/set a media type"; return false; }


    // For synchronous ReadSample (blocking call in a loop):
    // Need a separate thread to call ReadSample repeatedly and invoke frame_callback_.
    // For asynchronous ReadSample (using IMFSourceReaderCallback):
    // The callback (pCaptureCallback_->OnReadSample) would handle frame processing.

    // Placeholder: Actual streaming loop or async callback setup is needed here.
    // For now, just set the flag. The real work happens when ReadSample is called.
    AkLogWarn() << "WindowsCameraCapture::startStream: Stream starting is placeholder. ReadSample loop/callback needed." << std::endl;
    streaming_ = true;
    return true;
}

void WindowsCameraCapture::stopStream() {
    if (!streaming_) {
        return;
    }
    AkLogInfo() << "WindowsCameraCapture::stopStream: Stopping stream for " << opened_device_id_ << std::endl;
    // If using a streaming thread, signal it to stop and join it.
    // If using async callbacks, this might just involve setting a flag
    // and ensuring no more ReadSample calls are made or callbacks processed.
    streaming_ = false;
    // pSourceReader_ might need to be flushed or reset if specific operations were pending.
}

bool WindowsCameraCapture::isStreaming() const {
    return streaming_;
}

std::string WindowsCameraCapture::getOpenedDeviceId() const {
    return opened_device_id_;
}

std::vector<VideoFormat> WindowsCameraCapture::getSupportedFormats() {
    std::vector<VideoFormat> formats;
    if (!pSourceReader_) {
        AkLogError() << "WindowsCameraCapture::getSupportedFormats: Source reader not initialized." << std::endl;
        return formats;
    }

    AkLogInfo() << "WindowsCameraCapture::getSupportedFormats: Enumerating formats for " << opened_device_id_ << std::endl;
    DWORD dwMediaTypeIndex = 0;
    HRESULT hr = S_OK;
    while (SUCCEEDED(hr)) {
        IMFMediaType* pMediaType = nullptr;
        hr = pSourceReader_->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, dwMediaTypeIndex, &pMediaType);
        if (hr == MF_E_NO_MORE_TYPES) {
            hr = S_OK; // End of list
            break;
        } else if (SUCCEEDED(hr) && pMediaType) {
            GUID majorType = { 0 };
            hr = pMediaType->GetMajorType(&majorType);
            if (SUCCEEDED(hr) && majorType == MFMediaType_Video) {
                GUID subtype = { 0 };
                UINT32 width = 0, height = 0;
                MFRatio frameRate = { 0, 0 };

                pMediaType->GetGUID(MF_MT_SUBTYPE, &subtype);
                MFGetAttributeSize(pMediaType, MF_MT_FRAME_SIZE, &width, &height);
                MFGetAttributeRatio(pMediaType, MF_MT_FRAME_RATE, (UINT32*)&frameRate.Numerator, (UINT32*)&frameRate.Denominator);

                FourCC fourcc = PixelFormatUnknown;
                // Map MF subtype GUIDs to AkVCam::FourCC
                if (subtype == MFVideoFormat_RGB32) fourcc = PixelFormatRGB32; // Or BGR32 if that's what AkVCam expects for RGB32
                else if (subtype == MFVideoFormat_RGB24) fourcc = PixelFormatRGB24;
                else if (subtype == MFVideoFormat_YUY2) fourcc = PixelFormatYUY2;
                else if (subtype == MFVideoFormat_NV12) fourcc = PixelFormatNV12;
                else if (subtype == MFVideoFormat_I420) fourcc = PixelFormatI420;
                // Add more common formats: MJPG, etc.
                else {
                    // AkLogDebug() << "WindowsCameraCapture::getSupportedFormats: Skipping unmapped MF subtype." << std::endl;
                }

                if (fourcc != PixelFormatUnknown && frameRate.Denominator != 0 && frameRate.Numerator != 0) {
                    formats.emplace_back(fourcc, width, height, {{frameRate.Numerator, frameRate.Denominator}});
                }
            }
            pMediaType->Release();
        }
        dwMediaTypeIndex++;
    }

    // Remove duplicates (e.g. if MF reports same format multiple times for some reason)
    std::sort(formats.begin(), formats.end());
    formats.erase(std::unique(formats.begin(), formats.end()), formats.end());

    AkLogInfo() << "WindowsCameraCapture::getSupportedFormats: Found " << formats.size() << " formats for " << opened_device_id_ << std::endl;
    return formats;
}


// Factory function needs to be updated in a common place or here conditionally
// This is a re-declaration, ensure it's compatible or move create_platform_camera_capture
// to a .cpp file that can see both declarations if they are in different files.
// The create_platform_camera_capture() factory function is expected to be defined
// in a way that the build system links the correct platform-specific version,
// e.g., macos_cameracapture.mm provides it for Apple, and this file (or another) for Windows.
// The previous #ifndef __APPLE__ block for create_platform_camera_capture is removed
// from here to avoid redefinition conflicts if macos_cameracapture.mm provides it.
// A single, properly managed factory implementation is needed.
// For now, we rely on the one in macos_cameracapture.mm being guarded,
// and will add a Windows-specific part to it or a central factory.cpp later.

} // namespace AkVCam
#endif // _WIN32
