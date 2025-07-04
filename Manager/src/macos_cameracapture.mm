#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#include "icameracapture.h"
#include "VCamUtils/src/logger.h" // For AkLog
#include "VCamUtils/src/videoformat.h" // For VideoFormat, FourCC etc.

// Helper to convert CMSampleBuffer to AkVCam::VideoFrame
// This is a simplified placeholder. A real implementation needs careful handling
// of pixel formats, plane data, memory ownership, etc.

// Helper to convert OSType (FourCC) to a printable string for logging
static std::string FourCCString(OSType type) {
    char str[5] = {0};
    *(UInt32*)str = OSSwapHostToBigInt32(type); // Ensure correct byte order for display
    // Replace non-printable characters, though FourCCs are usually printable
    for(int i=0; i<4; ++i) {
        if (!isprint(str[i])) str[i] = '.';
    }
    return std::string(str);
}

static AkVCam::VideoFrame convertCMSampleBufferToVideoFrame(CMSampleBufferRef sampleBuffer) {
    if (!sampleBuffer) {
        return AkVCam::VideoFrame();
    }

    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    if (!imageBuffer) {
        return AkVCam::VideoFrame();
    }

    CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);

    int width = CVPixelBufferGetWidth(imageBuffer);
    int height = CVPixelBufferGetHeight(imageBuffer);
    OSType pixelFormat = CVPixelBufferGetPixelFormatType(imageBuffer);

    AkVCam::FourCC fourcc = AkVCam::PixelFormatUnknown;
    // More comprehensive FourCC mapping
    switch (pixelFormat) {
        case kCVPixelFormatType_32BGRA:
            fourcc = AkVCam::PixelFormatBGR32; // BGRA byte order
            break;
        case kCVPixelFormatType_32ABGR:
            fourcc = AkVCam::PixelFormatRGB32; // RGBA byte order (assuming AkVCam::PixelFormatRGB32 means RGBA) - Check VCamUtils definition
            break;
        case kCVPixelFormatType_24RGB:
            fourcc = AkVCam::PixelFormatRGB24;
            break;
        case kCVPixelFormatType_24BGR:
            fourcc = AkVCam::PixelFormatBGR24;
            break;
        case kCVPixelFormatType_422YpCbCr8: // YUYV or YUY2
            fourcc = AkVCam::PixelFormatYUY2;
            break;
        case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange: // NV12
        case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
            fourcc = AkVCam::PixelFormatNV12;
            break;
        case kCVPixelFormatType_420YpCbCr8Planar: // I420 / YUV420P (3 planes)
        case kCVPixelFormatType_420YpCbCr8PlanarFullRange:
            fourcc = AkVCam::PixelFormatI420; // Assuming I420 is the AkVCam equivalent
            break;
        // TODO: Add mappings for MJPEG (kCVPixelFormatType_JPEG) if needed, though it requires different handling (decompression)
        default:
            AkLogWarn() << "convertCMSampleBufferToVideoFrame: Unsupported CVPixelFormatType: " << FourCCString(pixelFormat) << std::endl;
            CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
            return AkVCam::VideoFrame();
    }

    AkVCam::VideoFormat vformat(fourcc, width, height);
    if (!vformat.isValid() || vformat.size() == 0) {
         AkLogError() << "convertCMSampleBufferToVideoFrame: Invalid VideoFormat created for " << width << "x" << height << " format " << AkVCam::VideoFormat::stringFromFourcc(fourcc) << std::endl;
        CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
        return AkVCam::VideoFrame();
    }

    AkVCam::VideoFrame frame(vformat);

    // Simplified data copy - assumes contiguous data for formats like BGRA, RGB
    // For planar formats (like NV12, YUV420p), this would be more complex, needing to copy planes.

    uint8_t* destBuffer = frame.data().data();
    size_t destBufferSize = frame.data().size();

    if (fourcc == AkVCam::PixelFormatNV12) {
        if (CVPixelBufferGetPlaneCount(imageBuffer) >= 2) {
            uint8_t* yPlane = (uint8_t*)CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0);
            size_t yHeight = CVPixelBufferGetHeightOfPlane(imageBuffer, 0);
            size_t yBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(imageBuffer, 0);

            uint8_t* uvPlane = (uint8_t*)CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 1);
            size_t uvHeight = CVPixelBufferGetHeightOfPlane(imageBuffer, 1);
            size_t uvBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(imageBuffer, 1);

            // Copy Y plane (Luma)
            for (size_t i = 0; i < yHeight; ++i) {
                if ((i * width) + width > destBufferSize) { AkLogError() << "NV12 Y plane copy out of bounds"; break; }
                memcpy(destBuffer + (i * width), yPlane + (i * yBytesPerRow), width);
            }

            // Copy UV plane (Chroma)
            uint8_t* uvDest = destBuffer + (width * height); // UV data starts after Y data in NV12 format
            for (size_t i = 0; i < uvHeight; ++i) {
                 // NV12's UV plane width is the same as Y plane for data (it contains U and V interleaved), but height is half.
                 // Bytes per row might include padding.
                size_t uv_row_size_to_copy = width; // For NV12, UV plane effective width is same as Y
                if ((i * uv_row_size_to_copy) + uv_row_size_to_copy > (destBufferSize - (width*height)) ) {
                    AkLogError() << "NV12 UV plane copy out of bounds: i=" << i << ", uv_row_size_to_copy=" << uv_row_size_to_copy
                                 << ", available=" << (destBufferSize - (width*height));
                    break;
                }
                memcpy(uvDest + (i * uv_row_size_to_copy), uvPlane + (i * uvBytesPerRow), uv_row_size_to_copy);
            }
        } else {
            AkLogWarn() << "convertCMSampleBufferToVideoFrame: NV12 format expected 2 planes, got " << CVPixelBufferGetPlaneCount(imageBuffer) << ". Falling back to simple copy." << std::endl;
            memcpy(destBuffer, CVPixelBufferGetBaseAddress(imageBuffer), std::min(destBufferSize, CVPixelBufferGetDataSize(imageBuffer)));
        }
    } else if (fourcc == AkVCam::PixelFormatI420) {
        // I420 is tri-planar Y, U, V.
        if (CVPixelBufferGetPlaneCount(imageBuffer) >= 3) {
            // Y Plane
            uint8_t* yPlaneSrc = (uint8_t*)CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0);
            size_t yHeightSrc = CVPixelBufferGetHeightOfPlane(imageBuffer, 0);
            size_t yBytesPerRowSrc = CVPixelBufferGetBytesPerRowOfPlane(imageBuffer, 0);
            for (size_t i = 0; i < yHeightSrc; ++i) {
                if ((i * width) + width > destBufferSize) { AkLogError() << "I420 Y plane copy out of bounds"; break; }
                memcpy(destBuffer + (i * width), yPlaneSrc + (i * yBytesPerRowSrc), width);
            }

            // U Plane (Cb)
            uint8_t* uPlaneSrc = (uint8_t*)CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 1);
            size_t uHeightSrc = CVPixelBufferGetHeightOfPlane(imageBuffer, 1); // Should be height/2
            size_t uBytesPerRowSrc = CVPixelBufferGetBytesPerRowOfPlane(imageBuffer, 1); // Should be width/2
            uint8_t* uDest = destBuffer + (width * height);
            size_t uWidth = width / 2; // I420 U plane width
            for (size_t i = 0; i < uHeightSrc; ++i) {
                if ((i * uWidth) + uWidth > (destBufferSize - (width*height)) ) { AkLogError() << "I420 U plane copy out of bounds"; break; }
                memcpy(uDest + (i * uWidth), uPlaneSrc + (i * uBytesPerRowSrc), uWidth);
            }

            // V Plane (Cr)
            uint8_t* vPlaneSrc = (uint8_t*)CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 2);
            size_t vHeightSrc = CVPixelBufferGetHeightOfPlane(imageBuffer, 2); // Should be height/2
            size_t vBytesPerRowSrc = CVPixelBufferGetBytesPerRowOfPlane(imageBuffer, 2); // Should be width/2
            uint8_t* vDest = destBuffer + (width * height) + (uWidth * uHeightSrc); // V data after U data
            size_t vWidth = width / 2; // I420 V plane width
            for (size_t i = 0; i < vHeightSrc; ++i) {
                if ((i * vWidth) + vWidth > (destBufferSize - (width*height) - (uWidth*uHeightSrc)) ) { AkLogError() << "I420 V plane copy out of bounds"; break; }
                memcpy(vDest + (i * vWidth), vPlaneSrc + (i * vBytesPerRowSrc), vWidth);
            }
        } else {
            AkLogWarn() << "convertCMSampleBufferToVideoFrame: I420 format expected 3 planes, got " << CVPixelBufferGetPlaneCount(imageBuffer) << ". Falling back to simple copy." << std::endl;
            memcpy(destBuffer, CVPixelBufferGetBaseAddress(imageBuffer), std::min(destBufferSize, CVPixelBufferGetDataSize(imageBuffer)));
        }
    } else if (fourcc == AkVCam::PixelFormatYUY2) { // Packed YUV 4:2:2
        // YUY2 is packed, similar to RGB/BGRA in terms of being non-planar from CVPixelBuffer's perspective usually
        void* baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
        size_t sourceBufferSize = CVPixelBufferGetDataSize(imageBuffer);
        // YUY2 size is width * height * 2
        if (destBufferSize == sourceBufferSize && destBufferSize == (size_t)width * height * 2) {
            memcpy(destBuffer, baseAddress, sourceBufferSize);
        } else if (destBufferSize <= sourceBufferSize) { // If AkVCam::VideoFrame expects exact size
             AkLogWarn() << "convertCMSampleBufferToVideoFrame: YUY2 size mismatch or CVPixelBufferGetDataSize includes padding. Dest: " << destBufferSize << ", Source: " << sourceBufferSize << ", Expected YUY2: " << (width*height*2) << ". Copying destBufferSize." << std::endl;
            memcpy(destBuffer, baseAddress, destBufferSize);
        }
        else {
             AkLogWarn() << "convertCMSampleBufferToVideoFrame: YUY2 frame data size mismatch. Dest: " << destBufferSize << ", Source: " << sourceBufferSize << ". Copying min." << std::endl;
             memcpy(destBuffer, baseAddress, std::min(destBufferSize, sourceBufferSize));
        }
    }
    else if (!CVPixelBufferIsPlanar(imageBuffer)) { // Other Packed formats like BGRA, RGB24
        void* baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
        size_t sourceBufferSize = CVPixelBufferGetDataSize(imageBuffer);
        if (destBufferSize == sourceBufferSize) {
            memcpy(destBuffer, baseAddress, sourceBufferSize);
        } else {
             // This often happens if CVPixelBufferGetDataSize reports a size that includes padding,
             // while vformat.size() is the pure image data size.
             // We should copy based on expected image size (vformat.size()) if it's smaller or equal to sourceBufferSize.
            if (destBufferSize <= sourceBufferSize) {
                memcpy(destBuffer, baseAddress, destBufferSize);
            } else {
                 AkLogWarn() << "convertCMSampleBufferToVideoFrame: Frame data size mismatch. Dest: " << destBufferSize << ", Source: " << sourceBufferSize << ". Copying min." << std::endl;
                 memcpy(destBuffer, baseAddress, std::min(destBufferSize, sourceBufferSize));
            }
        }
    } else {
        // Other planar formats not yet handled
        AkLogWarn() << "convertCMSampleBufferToVideoFrame: Unhandled planar buffer format " << FourCCString(pixelFormat) << ". Using simple memcpy." << std::endl;
        memcpy(destBuffer, CVPixelBufferGetBaseAddress(imageBuffer), std::min(destBufferSize, CVPixelBufferGetDataSize(imageBuffer)));
    }

    CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
    return frame;
}


// AVCaptureVideoDataOutputSampleBufferDelegate
@interface AkCaptureDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property (nonatomic, assign) AkVCam::FrameCaptureCallback frameCallback;
@property (nonatomic, assign) dispatch_queue_t sampleBufferQueue;
- (instancetype)initWithCallback:(AkVCam::FrameCaptureCallback)callback;
@end

@implementation AkCaptureDelegate

- (instancetype)initWithCallback:(AkVCam::FrameCaptureCallback)callback {
    self = [super init];
    if (self) {
        _frameCallback = callback;
        // Create a serial dispatch queue for processing sample buffers.
        _sampleBufferQueue = dispatch_queue_create("akvirtualcamera.samplebufferqueue", DISPATCH_QUEUE_SERIAL);
    }
    return self;
}

- (void)captureOutput:(AVCaptureOutput *)output didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    if (self.frameCallback) {
        @autoreleasepool {
            //AkLogDebug() << "AkCaptureDelegate: didOutputSampleBuffer" << std::endl;
            AkVCam::VideoFrame frame = convertCMSampleBufferToVideoFrame(sampleBuffer);
            if (frame.isValid() && frame.format().size() > 0) {
                self.frameCallback(frame);
            } else {
                // AkLogWarn() << "AkCaptureDelegate: Dropping invalid frame from conversion." << std::endl;
            }
        }
    }
}

- (void)captureOutput:(AVCaptureOutput *)output didDropSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    // AkLogWarn() << "AkCaptureDelegate: didDropSampleBuffer" << std::endl;
    // Handle dropped frames if necessary
}
@end


namespace AkVCam {

class MacOSCameraCapture : public ICameraCapture {
public:
    MacOSCameraCapture() : captureSession(nil), videoDeviceInput(nil), videoDataOutput(nil), captureDelegate(nil), streaming(false) {}

    ~MacOSCameraCapture() override {
        close(); // Ensure resources are released
    }

    bool open(const std::string& deviceId) override {
        if (streaming) {
            AkLogError() << "MacOSCameraCapture::open: Cannot open camera while streaming." << std::endl;
            return false;
        }
        close(); // Close any existing session

        @autoreleasepool {
            NSString* nsDeviceId = [NSString stringWithUTF8String:deviceId.c_str()];
            AVCaptureDevice* device = [AVCaptureDevice deviceWithUniqueID:nsDeviceId];

            if (!device) {
                AkLogError() << "MacOSCameraCapture::open: Device with ID '" << deviceId << "' not found." << std::endl;
                return false;
            }

            NSError* error = nil;
            videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
            if (error || !videoDeviceInput) {
                AkLogError() << "MacOSCameraCapture::open: Error creating device input: " << (error ? [[error localizedDescription] UTF8String] : "Unknown error") << std::endl;
                videoDeviceInput = nil;
                return false;
            }

            captureSession = [[AVCaptureSession alloc] init];
            // TODO: Configure sessionPreset for desired resolution/quality if needed
            // [captureSession setSessionPreset:AVCaptureSessionPresetMedium];

            if ([captureSession canAddInput:videoDeviceInput]) {
                [captureSession addInput:videoDeviceInput];
            } else {
                AkLogError() << "MacOSCameraCapture::open: Cannot add device input to session." << std::endl;
                close();
                return false;
            }

            videoDataOutput = [[AVCaptureVideoDataOutput alloc] init];
            // Specify the pixel format. BGRA is often well-supported.
            // Or query device for supported formats and choose one.
            // NSDictionary* videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_32BGRA)};
            // [videoDataOutput setVideoSettings:videoSettings];
            videoDataOutput.alwaysDiscardsLateVideoFrames = YES; // Important for real-time

            if ([captureSession canAddOutput:videoDataOutput]) {
                [captureSession addOutput:videoDataOutput];
            } else {
                AkLogError() << "MacOSCameraCapture::open: Cannot add video data output to session." << std::endl;
                close();
                return false;
            }

            openedDeviceId = deviceId;
            AkLogInfo() << "MacOSCameraCapture::open: Successfully opened device: " << deviceId << std::endl;
            return true;
        } // @autoreleasepool
    }

    void close() override {
        if (streaming) {
            stopStream();
        }
        @autoreleasepool {
            if (captureSession) {
                if (videoDeviceInput && [captureSession.inputs containsObject:videoDeviceInput]) {
                    [captureSession removeInput:videoDeviceInput];
                }
                if (videoDataOutput && [captureSession.outputs containsObject:videoDataOutput]) {
                    [captureSession removeOutput:videoDataOutput];
                }
                videoDeviceInput = nil;
                videoDataOutput = nil;
                captureSession = nil;
            }
            captureDelegate = nil; // Releases the delegate
            openedDeviceId.clear();
            AkLogInfo() << "MacOSCameraCapture::close: Camera closed." << std::endl;
        }
    }

    bool startStream(FrameCaptureCallback callback) override {
        if (!captureSession || !videoDeviceInput || !videoDataOutput) {
            AkLogError() << "MacOSCameraCapture::startStream: Camera not opened or not configured correctly." << std::endl;
            return false;
        }
        if (streaming) {
            AkLogWarn() << "MacOSCameraCapture::startStream: Stream already started." << std::endl;
            return true;
        }

        @autoreleasepool {
            if (!captureDelegate) {
                 captureDelegate = [[AkCaptureDelegate alloc] initWithCallback:callback];
            } else {
                // Update callback if it changed, though typically it's set once.
                captureDelegate.frameCallback = callback;
            }
            [videoDataOutput setSampleBufferDelegate:captureDelegate queue:captureDelegate.sampleBufferQueue];

            [captureSession startRunning];
            streaming = true;
            AkLogInfo() << "MacOSCameraCapture::startStream: Stream started." << std::endl;
            return true;
        }
    }

    void stopStream() override {
        if (!streaming) {
            // AkLogInfo() << "MacOSCameraCapture::stopStream: Stream not active." << std::endl;
            return;
        }
        @autoreleasepool {
            if (captureSession && [captureSession isRunning]) {
                [captureSession stopRunning];
            }
            if (videoDataOutput) {
                 [videoDataOutput setSampleBufferDelegate:nil queue:nil]; // Remove delegate
            }
            streaming = false;
            AkLogInfo() << "MacOSCameraCapture::stopStream: Stream stopped." << std::endl;
        }
    }

    bool isStreaming() const override {
        return streaming;
    }

    std::string getOpenedDeviceId() const override {
        return openedDeviceId;
    }

    std::vector<AkVCam::VideoFormat> getSupportedFormats() override {
        std::vector<AkVCam::VideoFormat> supported_formats;
        if (!videoDeviceInput || !videoDeviceInput.device) {
            AkLogError() << "MacOSCameraCapture::getSupportedFormats: Device not open or input not available." << std::endl;
            return supported_formats;
        }

        @autoreleasepool {
            AVCaptureDevice* device = videoDeviceInput.device;
            for (AVCaptureDeviceFormat* format in device.formats) {
                CMVideoFormatDescriptionRef desc = format.formatDescription;
                FourCC fourcc = AkVCam::PixelFormatUnknown;
                OSType cvPixelFormat = CMVideoFormatDescriptionGetCodecType(desc);

                // Remap CVPixelFormatType to AkVCam::FourCC (similar to convertCMSampleBufferToVideoFrame)
                switch (cvPixelFormat) {
                    case kCVPixelFormatType_32BGRA: fourcc = AkVCam::PixelFormatBGR32; break;
                    case kCVPixelFormatType_32ABGR: fourcc = AkVCam::PixelFormatRGB32; break;
                    case kCVPixelFormatType_24RGB:  fourcc = AkVCam::PixelFormatRGB24; break;
                    case kCVPixelFormatType_24BGR:  fourcc = AkVCam::PixelFormatBGR24; break;
                    case kCVPixelFormatType_422YpCbCr8: fourcc = AkVCam::PixelFormatYUY2; break;
                    case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
                    case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
                        fourcc = AkVCam::PixelFormatNV12; break;
                    case kCVPixelFormatType_420YpCbCr8Planar:
                    case kCVPixelFormatType_420YpCbCr8PlanarFullRange:
                        fourcc = AkVCam::PixelFormatI420; break;
                    default:
                        // AkLogDebug() << "MacOSCameraCapture::getSupportedFormats: Skipping unsupported CVPixelFormatType: " << FourCCString(cvPixelFormat) << std::endl;
                        continue; // Skip if not mapped
                }

                if (fourcc != AkVCam::PixelFormatUnknown) {
                    CMVideoDimensions dimensions = CMVideoFormatDescriptionGetDimensions(desc);
                    // Frame rates
                    std::vector<AkVCam::Fraction> frameRates;
                    for (AVFrameRateRange* range in format.videoSupportedFrameRateRanges) {
                        // For simplicity, we can take min, max, or a common rate.
                        // Here, let's try to add a few common ones or just the max.
                        // AVFoundation provides ranges; VCamUtils might expect discrete rates.
                        // This part might need more sophisticated mapping if specific discrete rates are needed.
                        if (range.maxFrameRate >= 30.0) frameRates.push_back({30, 1});
                        else if (range.maxFrameRate >= 25.0) frameRates.push_back({25,1});
                        else if (range.maxFrameRate > 0) frameRates.push_back({(int)(range.maxFrameRate), 1});
                        // To be more thorough, iterate through common FPS values and check if they fall in any range.
                    }
                    if (frameRates.empty()){ // Add a default if none found from ranges, e.g., 30fps
                        frameRates.push_back({30,1});
                    }
                    // Remove duplicate frame rates if any added
                    std::sort(frameRates.begin(), frameRates.end());
                    frameRates.erase(std::unique(frameRates.begin(), frameRates.end()), frameRates.end());


                    if (!frameRates.empty()) {
                         supported_formats.emplace_back(fourcc, dimensions.width, dimensions.height, frameRates);
                    }
                }
            }
        }
        // Remove duplicate formats (e.g. same resolution/fourcc but from different AVFrameRateRange yielding same discrete FPS)
        std::sort(supported_formats.begin(), supported_formats.end());
        supported_formats.erase(std::unique(supported_formats.begin(), supported_formats.end()), supported_formats.end());

        AkLogInfo() << "MacOSCameraCapture::getSupportedFormats: Found " << supported_formats.size() << " formats for device " << openedDeviceId << std::endl;
        return supported_formats;
    }

private:
    AVCaptureSession* captureSession;
    AVCaptureDeviceInput* videoDeviceInput;
    AVCaptureVideoDataOutput* videoDataOutput;
    AkCaptureDelegate* captureDelegate;
    bool streaming;
    std::string openedDeviceId;
};


// Factory function implementation
std::unique_ptr<ICameraCapture> create_platform_camera_capture() {
#ifdef __APPLE__
    return std::make_unique<MacOSCameraCapture>();
#elif _WIN32
    // Forward declare WindowsCameraCapture if its definition is in another file and not yet seen
    // class WindowsCameraCapture; // This would require WindowsCameraCapture to be defined for the linker
    // For now, this will only compile if WindowsCameraCapture is defined *before* this point
    // when compiling for Windows. This implies windows_cameracapture.cpp should be compiled first or
    // the class definition needs to be available.
    // To make this work cleanly, WindowsCameraCapture definition should be available.
    // Assuming windows_cameracapture.cpp defines it and is part of the same target.
    return std::make_unique<WindowsCameraCapture>();
#else
    AkLogError() << "create_platform_camera_capture: Platform not supported for camera capture." << std::endl;
    return nullptr;
#endif
}

} // namespace AkVCam
