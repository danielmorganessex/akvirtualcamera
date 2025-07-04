/* akvirtualcamera, virtual camera for Mac and Windows.
 * Copyright (C) 2020  Gonzalo Exequiel Pedone
 *
 * akvirtualcamera is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * akvirtualcamera is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with akvirtualcamera. If not, see <http://www.gnu.org/licenses/>.
 *
 * Web-Site: http://webcamoid.github.io/
 */

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <codecvt>
#include <csignal>
#include <cstring>
#include <iostream>
#include <functional>
#include <locale>
#include <sstream>
#include <thread>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "cmdparser.h"
#include "VCamUtils/src/ipcbridge.h"
#include "VCamUtils/src/settings.h"
#include "VCamUtils/src/videoformat.h"
#include "VCamUtils/src/videoframe.h"
#include "VCamUtils/src/logger.h"
#include "icameracapture.h" // Include the new interface

#define COMMONS_PROJECT_COMMIT_URL "https://github.com/webcamoid/akvirtualcamera/commit"

#define AKVCAM_BIND_FUNC(member) \
    std::bind(&member, this->d, std::placeholders::_1, std::placeholders::_2)

namespace AkVCam {
    using StringMatrix = std::vector<StringVector>;
    using VideoFormatMatrix = std::vector<std::vector<VideoFormat>>;

    struct CmdParserFlags
    {
        StringVector flags;
        std::string value;
        std::string helpString;
    };

    struct CmdParserCommand
    {
        std::string command;
        std::string arguments;
        std::string helpString;
        ProgramOptionsFunc func;
        std::vector<CmdParserFlags> flags;
        bool advanced {false};

        CmdParserCommand();
        CmdParserCommand(const std::string &command,
                         const std::string &arguments,
                         const std::string &helpString,
                         const ProgramOptionsFunc &func,
                         const std::vector<CmdParserFlags> &flags,
                         bool advanced);
    };

    // Placeholder implementation for listing physical cameras
    // This needs to be implemented with platform-specific code.
#ifdef __APPLE__
#include <AVFoundation/AVFoundation.h>
#endif

    std::vector<AkVCam::PhysicalCamera> AkVCam::list_physical_cameras_impl() {
        std::vector<AkVCam::PhysicalCamera> cameras;
    AkLogInfo() << "list_physical_cameras_impl: Starting camera enumeration." << std::endl;

#ifdef __APPLE__
    @autoreleasepool {
        NSArray<AVCaptureDevice *> *devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
        if (devices == nil || [devices count] == 0) {
            AkLogWarn() << "list_physical_cameras_impl: No video devices found on macOS." << std::endl;
            return cameras;
        }

        for (AVCaptureDevice *device in devices) {
            PhysicalCamera cam;
            cam.id = std::string([[device uniqueID] UTF8String]);
            cam.name = std::string([[device localizedName] UTF8String]);
            // You could also get modelID, manufacturer, etc. if needed
            // cam.model = std::string([[device modelID] UTF8String]);
            // cam.manufacturer = std::string([[device manufacturer] UTF8String]);
            cameras.push_back(cam);
            AkLogInfo() << "list_physical_cameras_impl: Found macOS camera: ID=" << cam.id << ", Name=" << cam.name << std::endl;
        }
    }
#elif _WIN32
    // Windows implementation using Media Foundation
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <shlwapi.h> // For StrRetToBuf
#include <propsys.h> // For IPropertyStore (though MFGetAttributeString might be enough)

#pragma comment(lib, "mfplat")
#pragma comment(lib, "mfreadwrite")
#pragma comment(lib, "mfuuid")
#pragma comment(lib, "shlwapi")
#pragma comment(lib, "propsys")


    HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    if (SUCCEEDED(hr)) {
        hr = MFStartup(MF_VERSION, MFSTARTUP_FULL);
        if (SUCCEEDED(hr)) {
            IMFAttributes* pAttributes = NULL;
            hr = MFCreateAttributes(&pAttributes, 1);
            if (SUCCEEDED(hr)) {
                hr = pAttributes->SetGUID(
                    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
                );
            }

            IMFActivate** ppDevices = NULL;
            UINT32 count = 0;
            if (SUCCEEDED(hr)) {
                hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
            }

            if (SUCCEEDED(hr)) {
                for (UINT32 i = 0; i < count; i++) {
                    PhysicalCamera cam;
                    WCHAR* pszFriendlyName = NULL;
                    WCHAR* pszSymbolicLink = NULL; // Unique ID

                    hr = ppDevices[i]->GetAllocatedString(
                        MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                        &pszFriendlyName,
                        NULL
                    );
                    if (FAILED(hr)) {
                        AkLogWarn() << "list_physical_cameras_impl: Failed to get friendly name for device " << i << std::endl;
                        pszFriendlyName = L"Unknown Camera"; // Fallback
                    }

                    hr = ppDevices[i]->GetAllocatedString(
                        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
                        &pszSymbolicLink,
                        NULL
                    );
                     if (FAILED(hr)) {
                        AkLogWarn() << "list_physical_cameras_impl: Failed to get symbolic link for device " << i << std::endl;
                        // Create a fallback ID if symbolic link fails, though it's critical
                        std::wstring fallback_id = L"WIN_CAM_FALLBACK_ID_" + std::to_wstring(i);
                        pszSymbolicLink = (WCHAR*)CoTaskMemAlloc((fallback_id.length() + 1) * sizeof(WCHAR));
                        if(pszSymbolicLink) wcscpy_s(pszSymbolicLink, fallback_id.length() + 1, fallback_id.c_str()); else continue;
                    }

                    char friendlyNameStr[256] = {0};
                    WideCharToMultiByte(CP_UTF8, 0, pszFriendlyName, -1, friendlyNameStr, sizeof(friendlyNameStr) -1, NULL, NULL);
                    cam.name = friendlyNameStr;

                    char symbolicLinkStr[512] = {0}; // Symbolic links can be long
                    WideCharToMultiByte(CP_UTF8, 0, pszSymbolicLink, -1, symbolicLinkStr, sizeof(symbolicLinkStr) -1, NULL, NULL);
                    cam.id = symbolicLinkStr;

                    cameras.push_back(cam);
                    AkLogInfo() << "list_physical_cameras_impl: Found Windows camera: ID=" << cam.id << ", Name=" << cam.name << std::endl;

                    CoTaskMemFree(pszFriendlyName);
                    CoTaskMemFree(pszSymbolicLink);
                    ppDevices[i]->Release();
                }
                CoTaskMemFree(ppDevices);
            } else {
                 AkLogError() << "list_physical_cameras_impl: MFEnumDeviceSources failed with HRESULT: " << std::hex << hr << std::endl;
            }

            if (pAttributes) pAttributes->Release();
            MFShutdown();
        } else {
             AkLogError() << "list_physical_cameras_impl: MFStartup failed with HRESULT: " << std::hex << hr << std::endl;
        }
        CoUninitialize();
    } else {
        AkLogError() << "list_physical_cameras_impl: CoInitializeEx failed with HRESULT: " << std::hex << hr << std::endl;
    }
#else
    // Linux or other platforms - returning dummy data
    cameras.push_back({"other_cam_id_01", "Other OS Dummy Webcam 1"});
    AkLogWarn() << "list_physical_cameras_impl: Non-macOS/Windows platform, returning dummy data." << std::endl;
#endif

    if (cameras.empty() && !defined(__APPLE__) && !defined(_WIN32)) {
         // Fallback dummy data if no platform-specific code was hit and list is empty
        cameras.push_back({"dummy_cam_id_01", "Fallback Dummy Webcam"});
        AkLogWarn() << "list_physical_cameras_impl: No platform-specific implementation hit, returning fallback dummy data." << std::endl;
    }

    if (cameras.empty()) {
        AkLogWarn() << "list_physical_cameras_impl: No cameras found after enumeration attempts." << std::endl;
    }

        return cameras;
    }

    class CmdParserPrivate
    {
        public:
            std::vector<CmdParserCommand> m_commands;
            IpcBridge m_ipcBridge;
            bool m_parseable {false};
            bool m_force {false};

            // Map to store physical camera ID to its split configuration
            std::map<std::string, WebcamSplitConfig> m_active_splits;
            // Instance for capturing from the physical camera
            std::unique_ptr<ICameraCapture> m_camera_capture;
            // Store the ID of the physical camera currently being captured for splitting
            std::string m_active_capture_physical_id;

            static const std::map<ControlType, std::string> &typeStrMap();
            void printFlags(const std::vector<CmdParserFlags> &cmdFlags,
                            size_t indent);
            size_t maxCommandLength(bool showAdvancedHelp);
            size_t maxArgumentsLength(bool showAdvancedHelp);
            size_t maxFlagsLength(const std::vector<CmdParserFlags> &flags);
            size_t maxFlagsValueLength(const std::vector<CmdParserFlags> &flags);
            size_t maxColumnLength(const StringVector &table,
                                   size_t width,
                                   size_t column);
            std::vector<size_t> maxColumnsLength(const StringVector &table,
                                                 size_t width);
            void drawTableHLine(const std::vector<size_t> &columnsLength,
                                bool toStdErr=false);
            void drawTable(const StringVector &table,
                           size_t width,
                           bool toStdErr=false);
            CmdParserCommand *parserCommand(const std::string &command);
            const CmdParserFlags *parserFlag(const std::vector<CmdParserFlags> &cmdFlags,
                                             const std::string &flag);
            bool containsFlag(const StringMap &flags,
                              const std::string &command,
                              const std::string &flagAlias);
            std::string flagValue(const StringMap &flags,
                                  const std::string &command,
                                  const std::string &flagAlias);
            int defaultHandler(const StringMap &flags,
                               const StringVector &args);
            int showHelp(const StringMap &flags, const StringVector &args);
            int showDevices(const StringMap &flags, const StringVector &args);
            int addDevice(const StringMap &flags, const StringVector &args);
            int removeDevice(const StringMap &flags, const StringVector &args);
            int removeDevices(const StringMap &flags, const StringVector &args);
            int showDeviceDescription(const StringMap &flags,
                                      const StringVector &args);
            int setDeviceDescription(const StringMap &flags,
                                     const StringVector &args);
            int showSupportedFormats(const StringMap &flags,
                                     const StringVector &args);
            int showDefaultFormat(const StringMap &flags,
                                  const StringVector &args);
            int showFormats(const StringMap &flags, const StringVector &args);
            int addFormat(const StringMap &flags, const StringVector &args);
            int removeFormat(const StringMap &flags, const StringVector &args);
            int removeFormats(const StringMap &flags, const StringVector &args);
            int update(const StringMap &flags, const StringVector &args);
            int loadSettings(const StringMap &flags, const StringVector &args);
            int stream(const StringMap &flags, const StringVector &args);
            int listenEvents(const StringMap &flags, const StringVector &args);
            int showControls(const StringMap &flags, const StringVector &args);
            int readControl(const StringMap &flags, const StringVector &args);
            int writeControls(const StringMap &flags, const StringVector &args);
            int picture(const StringMap &flags, const StringVector &args);
            int setPicture(const StringMap &flags, const StringVector &args);
            int logLevel(const StringMap &flags, const StringVector &args);
            int setLogLevel(const StringMap &flags, const StringVector &args);
            int showClients(const StringMap &flags, const StringVector &args);
            int dumpInfo(const StringMap &flags, const StringVector &args);
            int hacks(const StringMap &flags, const StringVector &args);
            int hackInfo(const StringMap &flags, const StringVector &args);
            int hack(const StringMap &flags, const StringVector &args);
            void loadGenerals(Settings &settings);
            VideoFormatMatrix readFormats(Settings &settings);
            std::vector<VideoFormat> readFormat(Settings &settings);
            int listPhysicalCameras(const StringMap &flags, const StringVector &args);
            int splitWebcam(const StringMap &flags, const StringVector &args);         // New handler
            int startSplit(const StringMap &flags, const StringVector &args);          // New handler
            int stopSplit(const StringMap &flags, const StringVector &args);           // New handler
            int removeSplit(const StringMap &flags, const StringVector &args);         // New handler
            StringMatrix matrixCombine(const StringMatrix &matrix);
            void matrixCombineP(const StringMatrix &matrix,
                                size_t index,
                                StringVector combined,
                                StringMatrix &combinations);
            void createDevices(Settings &settings,
                               const VideoFormatMatrix &availableFormats);
            void createDevice(Settings &settings,
                              const VideoFormatMatrix &availableFormats);
            std::vector<VideoFormat> readDeviceFormats(Settings &settings,
                                                       const VideoFormatMatrix &availableFormats);
    };

    std::string operator *(const std::string &str, size_t n);
    std::string operator *(size_t n, const std::string &str);
}

AkVCam::CmdParser::CmdParser()
{
    this->d = new CmdParserPrivate();
    auto logFile = this->d->m_ipcBridge.logPath("AkVCamManager");
    AkLogInfo() << "Sending debug output to " << logFile << std::endl;
    AkVCam::Logger::setLogFile(logFile);

    this->d->m_commands.push_back({});
    this->setDefaultFuntion(AKVCAM_BIND_FUNC(CmdParserPrivate::defaultHandler));
    this->addFlags("",
                   {"-h", "--help"},
                   "Show help.");
    this->addFlags("",
                   {"--help-all"},
                   "Show advanced help.");
    this->addFlags("",
                   {"-v", "--version"},
                   "Show program version.");
    this->addFlags("",
                   {"-p", "--parseable"},
                   "Show parseable output.");
    this->addFlags("",
                   {"-f", "--force"},
                   "Force command.");
    this->addFlags("",
                   {"--build-info"},
                   "Show build information.");
    this->addCommand("devices",
                     "",
                     "List devices.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::showDevices));
    this->addCommand("add-device",
                     "DESCRIPTION",
                     "Add a new device.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::addDevice));
    this->addFlags("add-device",
                   {"-i", "--id"},
                   "DEVICEID",
                   "Create device as DEVICEID.");
    this->addCommand("remove-device",
                     "DEVICE",
                     "Remove a device.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::removeDevice));
    this->addCommand("remove-devices",
                     "",
                     "Remove all devices.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::removeDevices));
    this->addCommand("description",
                     "DEVICE",
                     "Show device description.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::showDeviceDescription));
    this->addCommand("set-description",
                     "DEVICE DESCRIPTION",
                     "Set device description.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::setDeviceDescription));
    this->addCommand("supported-formats",
                     "",
                     "Show supported formats.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::showSupportedFormats));
    this->addFlags("supported-formats",
                   {"-i", "--input"},
                   "Show supported input formats.");
    this->addFlags("supported-formats",
                   {"-o", "--output"},
                   "Show supported output formats.");
    this->addCommand("default-format",
                     "",
                     "Default device format.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::showDefaultFormat));
    this->addFlags("default-format",
                   {"-i", "--input"},
                   "Default input format.");
    this->addFlags("default-format",
                   {"-o", "--output"},
                   "Default output format.");
    this->addCommand("formats",
                     "DEVICE",
                     "Show device formats.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::showFormats));
    this->addCommand("add-format",
                     "DEVICE FORMAT WIDTH HEIGHT FPS",
                     "Add a new device format.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::addFormat));
    this->addFlags("add-format",
                   {"-i", "--index"},
                   "INDEX",
                   "Add format at INDEX.");
    this->addCommand("remove-format",
                     "DEVICE INDEX",
                     "Remove device format.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::removeFormat));
    this->addCommand("remove-formats",
                     "DEVICE",
                     "Remove all device formats.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::removeFormats));
    this->addCommand("update",
                     "",
                     "Update devices.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::update));
    this->addCommand("load",
                     "SETTINGS.INI",
                     "Create devices from a setting file.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::loadSettings));
    this->addCommand("stream",
                     "DEVICE FORMAT WIDTH HEIGHT",
                     "Read frames from stdin and send them to the device.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::stream));
    this->addFlags("stream",
                   {"-f", "--fps"},
                   "FPS",
                   "Read stream input at a constant frame rate.");
    this->addCommand("listen-events",
                     "",
                     "Keep the manager running and listening to global events.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::listenEvents));
    this->addCommand("controls",
                     "DEVICE",
                     "Show device controls.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::showControls));
    this->addCommand("get-control",
                     "DEVICE CONTROL",
                     "Read device control.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::readControl));
    this->addFlags("get-control",
                   {"-c", "--description"},
                   "Show control description.");
    this->addFlags("get-control",
                   {"-t", "--type"},
                   "Show control type.");
    this->addFlags("get-control",
                   {"-m", "--min"},
                   "Show minimum value for the control.");
    this->addFlags("get-control",
                   {"-M", "--max"},
                   "Show maximum value for the control.");
    this->addFlags("get-control",
                   {"-s", "--step"},
                   "Show increment/decrement step for the control.");
    this->addFlags("get-control",
                   {"-d", "--default"},
                   "Show default value for the control.");
    this->addFlags("get-control",
                   {"-l", "--menu"},
                   "Show options of a memu type control.");
    this->addCommand("set-controls",
                     "DEVICE CONTROL_1=VALUE CONTROL_2=VALUE...",
                     "Write device controls values.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::writeControls));
    this->addCommand("picture",
                     "",
                     "Placeholder picture to show when no streaming.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::picture));
    this->addCommand("set-picture",
                     "FILE",
                     "Set placeholder picture.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::setPicture));
    this->addCommand("loglevel",
                     "",
                     "Show current debugging level.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::logLevel));
    this->addCommand("set-loglevel",
                     "LEVEL",
                     "Set debugging level.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::setLogLevel));
    this->addCommand("clients",
                     "",
                     "Show clients using the camera.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::showClients));
    this->addCommand("dump",
                     "",
                     "Show all information in a parseable XML format.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::dumpInfo));
    this->addCommand("hacks",
                     "",
                     "List system hacks to make the virtual camera work.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::hacks),
                     true);
    this->addCommand("hack-info",
                     "HACK",
                     "Show hack information.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::hackInfo),
                     true);
    this->addFlags("hack-info",
                   {"-s", "--issafe"},
                   "Is hack safe?");
    this->addFlags("hack-info",
                   {"-c", "--description"},
                   "Show hack description.");
    this->addCommand("hack",
                     "HACK PARAMS...",
                     "Apply system hack.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::hack),
                     true);
    this->addFlags("hack",
                   {"-y", "--yes"},
                   "Accept all risks and continue anyway.");
    this->addCommand("list-physical-cameras",
                     "",
                     "List available physical cameras.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::listPhysicalCameras));
    this->addCommand("split-webcam",
                     "<physical_cam_id> <num_splits> [base_name_prefix]",
                     "Configure a physical camera to be split into multiple virtual cameras.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::splitWebcam));
    this->addCommand("start-split",
                     "<physical_cam_id>",
                     "Start streaming from a physical camera to its configured virtual splits.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::startSplit));
    this->addCommand("stop-split",
                     "<physical_cam_id>",
                     "Stop streaming from a physical camera to its virtual splits.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::stopSplit));
    this->addCommand("remove-split",
                     "<physical_cam_id>",
                     "Remove a webcam split configuration and its virtual cameras.",
                     AKVCAM_BIND_FUNC(CmdParserPrivate::removeSplit));
}

AkVCam::CmdParser::~CmdParser()
{
    delete this->d;
}

int AkVCam::CmdParser::parse(int argc, char **argv)
{
    auto program = basename(argv[0]);
    auto command = &this->d->m_commands[0];
    StringMap flags;
    StringVector arguments {program};

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        char *p = nullptr;
        strtod(arg.c_str(), &p);

        if (arg[0] == '-' && p && strnlen(p, 1024) != 0) {
            auto flag = this->d->parserFlag(command->flags, arg);

            if (!flag) {
                if (command->command.empty())
                    std::cout << "Invalid option '"
                              << arg
                              << "'"
                              << std::endl;
                else
                    std::cout << "Invalid option '"
                              << arg << "' for '"
                              << command->command
                              << "'"
                              << std::endl;

                return -EINVAL;
            }

            std::string value;

            if (!flag->value.empty()) {
                auto next = i + 1;

                if (next < argc) {
                    value = argv[next];
                    i++;
                }
            }

            flags[arg] = value;
        } else {
            if (command->command.empty()) {
                if (!flags.empty()) {
                    auto result = command->func(flags, {program});

                    if (result < 0)
                        return result;

                    flags.clear();
                }

                auto cmd = this->d->parserCommand(arg);

                if (cmd) {
                    command = cmd;
                    flags.clear();
                } else {
                    std::cout << "Unknown command '" << arg << "'" << std::endl;

                    return -EINVAL;
                }
            } else {
                arguments.push_back(arg);
            }
        }
    }

    if (!this->d->m_force && this->d->m_ipcBridge.isBusyFor(command->command)) {
        std::cerr << "This operation is not permitted." << std::endl;
        std::cerr << "The virtual camera is in use. Stop or close the virtual "
                  << "camera clients and try again." << std::endl;
        std::cerr << std::endl;
        auto clients = this->d->m_ipcBridge.clientsPids();

        if (!clients.empty()) {
            std::vector<std::string> table {
                "Pid",
                "Executable"
            };
            auto columns = table.size();

            for (auto &pid: clients) {
                table.push_back(std::to_string(pid));
                table.push_back(this->d->m_ipcBridge.clientExe(pid));
            }

            this->d->drawTable(table, columns, true);
        }

        return -EBUSY;
    }

    if (this->d->m_ipcBridge.needsRoot(command->command)
        || (command->command == "hack"
            && arguments.size() >= 2
            && this->d->m_ipcBridge.hackNeedsRoot(arguments[1]))) {
        std::cerr << "You must run this command with administrator privileges." << std::endl;

        return -EPERM;
    }

    return command->func(flags, arguments);
}

void AkVCam::CmdParser::setDefaultFuntion(const ProgramOptionsFunc &func)
{
    this->d->m_commands[0].func = func;
}

void AkVCam::CmdParser::addCommand(const std::string &command,
                                   const std::string &arguments,
                                   const std::string &helpString,
                                   const ProgramOptionsFunc &func,
                                   bool advanced)
{
    auto it =
            std::find_if(this->d->m_commands.begin(),
                         this->d->m_commands.end(),
                         [&command] (const CmdParserCommand &cmd) -> bool {
        return cmd.command == command;
    });

    if (it == this->d->m_commands.end()) {
        this->d->m_commands.push_back({command,
                                       arguments,
                                       helpString,
                                       func,
                                       {},
                                       advanced});
    } else {
        it->command = command;
        it->arguments = arguments;
        it->helpString = helpString;
        it->func = func;
        it->advanced = advanced;
    }
}

void AkVCam::CmdParser::addFlags(const std::string &command,
                                 const StringVector &flags,
                                 const std::string &value,
                                 const std::string &helpString)
{
    auto it =
            std::find_if(this->d->m_commands.begin(),
                         this->d->m_commands.end(),
                         [&command] (const CmdParserCommand &cmd) -> bool {
        return cmd.command == command;
    });

    if (it == this->d->m_commands.end())
        return;

    it->flags.push_back({flags, value, helpString});
}

void AkVCam::CmdParser::addFlags(const std::string &command,
                                 const StringVector &flags,
                                 const std::string &helpString)
{
    this->addFlags(command, flags, "", helpString);
}

const std::map<AkVCam::ControlType, std::string> &AkVCam::CmdParserPrivate::typeStrMap()
{
    static const std::map<ControlType, std::string> typeStr {
        {ControlTypeInteger, "Integer"},
        {ControlTypeBoolean, "Boolean"},
        {ControlTypeMenu   , "Menu"   },
    };

    return typeStr;
}

void AkVCam::CmdParserPrivate::printFlags(const std::vector<CmdParserFlags> &cmdFlags,
                                          size_t indent)
{
    std::vector<char> spaces(indent, ' ');
    auto maxFlagsLen = this->maxFlagsLength(cmdFlags);
    auto maxFlagsValueLen = this->maxFlagsValueLength(cmdFlags);

    for (auto &flag: cmdFlags) {
        auto allFlags = join(flag.flags, ", ");
        std::cout << std::string(spaces.data(), indent)
                  << fill(allFlags, maxFlagsLen);

        if (maxFlagsValueLen > 0)
            std::cout << " " << fill(flag.value, maxFlagsValueLen);

        std::cout << "    "
                  << flag.helpString
                  << std::endl;
    }
}

size_t AkVCam::CmdParserPrivate::maxCommandLength(bool showAdvancedHelp)
{
    size_t length = 0;

    for (auto &cmd: this->m_commands)
        if (!cmd.advanced || showAdvancedHelp)
            length = std::max(cmd.command.size(), length);

    return length;
}

size_t AkVCam::CmdParserPrivate::maxArgumentsLength(bool showAdvancedHelp)
{
    size_t length = 0;

    for (auto &cmd: this->m_commands)
        if (!cmd.advanced || showAdvancedHelp)
            length = std::max(cmd.arguments.size(), length);

    return length;
}

size_t AkVCam::CmdParserPrivate::maxFlagsLength(const std::vector<CmdParserFlags> &flags)
{
    size_t length = 0;

    for (auto &flag: flags)
        length = std::max(join(flag.flags, ", ").size(), length);

    return length;
}

size_t AkVCam::CmdParserPrivate::maxFlagsValueLength(const std::vector<CmdParserFlags> &flags)
{
    size_t length = 0;

    for (auto &flag: flags)
        length = std::max(flag.value.size(), length);

    return length;
}

size_t AkVCam::CmdParserPrivate::maxColumnLength(const AkVCam::StringVector &table,
                                                 size_t width,
                                                 size_t column)
{
    size_t length = 0;
    size_t height = table.size() / width;

    for (size_t y = 0; y < height; y++) {
        auto &str = table[y * width + column];
        length = std::max(str.size(), length);
    }

    return length;
}

std::vector<size_t> AkVCam::CmdParserPrivate::maxColumnsLength(const AkVCam::StringVector &table,
                                                               size_t width)
{
    std::vector<size_t> lengths;

    for (size_t x = 0; x < width; x++)
        lengths.push_back(this->maxColumnLength(table, width, x));

    return lengths;
}

void AkVCam::CmdParserPrivate::drawTableHLine(const std::vector<size_t> &columnsLength,
                                              bool toStdErr)
{
    std::ostream *out = &std::cout;

    if (toStdErr)
        out = &std::cerr;

    *out << '+';

    for (auto &len: columnsLength)
        *out << std::string("-") * (len + 2) << '+';

    *out << std::endl;
}

void AkVCam::CmdParserPrivate::drawTable(const AkVCam::StringVector &table,
                                         size_t width,
                                         bool toStdErr)
{
    size_t height = table.size() / width;
    auto columnsLength = this->maxColumnsLength(table, width);
    this->drawTableHLine(columnsLength, toStdErr);
    std::ostream *out = &std::cout;

    if (toStdErr)
        out = &std::cerr;

    for (size_t y = 0; y < height; y++) {
        *out << "|";

        for (size_t x = 0; x < width; x++) {
            auto &element = table[x + y * width];
            *out << " " << fill(element, columnsLength[x]) << " |";
        }

        *out << std::endl;

        if (y == 0 && height > 1)
            this->drawTableHLine(columnsLength, toStdErr);
    }

    this->drawTableHLine(columnsLength, toStdErr);
}

AkVCam::CmdParserCommand *AkVCam::CmdParserPrivate::parserCommand(const std::string &command)
{
    for (auto &cmd: this->m_commands)
        if (cmd.command == command)
            return &cmd;

    return nullptr;
}

const AkVCam::CmdParserFlags *AkVCam::CmdParserPrivate::parserFlag(const std::vector<CmdParserFlags> &cmdFlags,
                                                                   const std::string &flag)
{
    for (auto &flags: cmdFlags)
        for (auto &f: flags.flags)
            if (f == flag)
                return &flags;

    return nullptr;
}

bool AkVCam::CmdParserPrivate::containsFlag(const StringMap &flags,
                                            const std::string &command,
                                            const std::string &flagAlias)
{
    for (auto &cmd: this->m_commands)
        if (cmd.command == command) {
            for (auto &flag: cmd.flags) {
                auto it = std::find(flag.flags.begin(),
                                    flag.flags.end(),
                                    flagAlias);

                if (it != flag.flags.end()) {
                    for (auto &f1: flags)
                        for (auto &f2: flag.flags)
                            if (f1.first == f2)
                                return true;

                    return false;
                }
            }

            return false;
        }

    return false;
}

std::string AkVCam::CmdParserPrivate::flagValue(const AkVCam::StringMap &flags,
                                                const std::string &command,
                                                const std::string &flagAlias)
{
    for (auto &cmd: this->m_commands)
        if (cmd.command == command) {
            for (auto &flag: cmd.flags) {
                auto it = std::find(flag.flags.begin(),
                                    flag.flags.end(),
                                    flagAlias);

                if (it != flag.flags.end()) {
                    for (auto &f1: flags)
                        for (auto &f2: flag.flags)
                            if (f1.first == f2)
                                return f1.second;

                    return {};
                }
            }

            return {};
        }

    return {};
}

int AkVCam::CmdParserPrivate::defaultHandler(const StringMap &flags,
                                             const StringVector &args)
{
    if (flags.empty()
        || this->containsFlag(flags, "", "-h")
        || this->containsFlag(flags, "", "--help-all")) {
        return this->showHelp(flags, args);
    }

    if (this->containsFlag(flags, "", "-v")) {
        std::cout << COMMONS_VERSION << std::endl;

        return 0;
    }

    if (this->containsFlag(flags, "", "--build-info")) {
#ifdef GIT_COMMIT_HASH
        std::string commitHash = GIT_COMMIT_HASH;
        std::string commitUrl = COMMONS_PROJECT_COMMIT_URL "/" GIT_COMMIT_HASH;

        if (commitHash.empty())
            commitHash = "Unknown";

        if (commitUrl.empty())
            commitUrl = "Unknown";
#else
        std::string commitHash;
        std::string commitUrl;
#endif

        std::cout << "Commit hash: " << commitHash << std::endl;
        std::cout << "Commit URL: " << commitUrl << std::endl;

        return 0;
    }

    if (this->containsFlag(flags, "", "-p"))
        this->m_parseable = true;

    if (this->containsFlag(flags, "", "-f"))
        this->m_force = true;

    return 0;
}

int AkVCam::CmdParserPrivate::showHelp(const StringMap &flags,
                                       const StringVector &args)
{
    UNUSED(flags);

    std::cout << args[0]
              << " [OPTIONS...] COMMAND [COMMAND_OPTIONS...] ..."
              << std::endl;
    std::cout << std::endl;
    std::cout << "AkVirtualCamera virtual device manager." << std::endl;
    std::cout << std::endl;
    std::cout << "General Options:" << std::endl;
    std::cout << std::endl;
    this->printFlags(this->m_commands[0].flags, 4);
    std::cout << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << std::endl;

    bool showAdvancedHelp = this->containsFlag(flags, "", "--help-all");
    auto maxCmdLen = this->maxCommandLength(showAdvancedHelp);
    auto maxArgsLen = this->maxArgumentsLength(showAdvancedHelp);

    for (auto &cmd: this->m_commands) {
        if (cmd.command.empty()
            || (cmd.advanced && !showAdvancedHelp))
            continue;

        std::cout << "    "
                  << fill(cmd.command, maxCmdLen)
                  << " "
                  << fill(cmd.arguments, maxArgsLen)
                  << "    "
                  << cmd.helpString << std::endl;

        if (!cmd.flags.empty())
            std::cout << std::endl;

        this->printFlags(cmd.flags, 8);

        if (!cmd.flags.empty())
            std::cout << std::endl;
    }

    return 0;
}

int AkVCam::CmdParserPrivate::showDevices(const StringMap &flags,
                                          const StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);

    auto devices = this->m_ipcBridge.devices();

    if (devices.empty())
        return 0;

    std::sort(devices.begin(), devices.end());

    if (this->m_parseable) {
        for (auto &device: devices)
            std::cout << device << std::endl;
    } else {
        std::vector<std::string> table {
            "Device",
            "Description"
        };
        auto columns = table.size();

        for (auto &device: devices) {
            table.push_back(device);
            table.push_back(this->m_ipcBridge.description(device));
        }

        this->drawTable(table, columns);
    }

    return 0;
}

int AkVCam::CmdParserPrivate::addDevice(const StringMap &flags,
                                        const StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Device description not provided." << std::endl;

        return -EINVAL;
    }

    auto deviceId = this->flagValue(flags, "add-device", "-i");
    deviceId = this->m_ipcBridge.addDevice(args[1], deviceId);

    if (deviceId.empty()) {
        std::cerr << "Failed to create device." << std::endl;

        return -EIO;
    }

    if (this->m_parseable)
        std::cout << deviceId << std::endl;
    else
        std::cout << "Device created as " << deviceId << std::endl;

    return 0;
}

int AkVCam::CmdParserPrivate::removeDevice(const StringMap &flags,
                                           const StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Device not provided." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto it = std::find(devices.begin(), devices.end(), deviceId);

    if (it == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    this->m_ipcBridge.removeDevice(args[1]);

    return 0;
}

int AkVCam::CmdParserPrivate::removeDevices(const AkVCam::StringMap &flags,
                                            const AkVCam::StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);

    for (auto &device: this->m_ipcBridge.devices())
        this->m_ipcBridge.removeDevice(device);

    return 0;
}

int AkVCam::CmdParserPrivate::showDeviceDescription(const StringMap &flags,
                                                    const StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Device not provided." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto it = std::find(devices.begin(), devices.end(), deviceId);

    if (it == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    std::cout << this->m_ipcBridge.description(args[1]) << std::endl;

    return 0;
}

int AkVCam::CmdParserPrivate::setDeviceDescription(const AkVCam::StringMap &flags,
                                                   const AkVCam::StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 3) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    this->m_ipcBridge.setDescription(deviceId, args[2]);

    return 0;
}

int AkVCam::CmdParserPrivate::showSupportedFormats(const StringMap &flags,
                                                   const StringVector &args)
{
    UNUSED(args);

    auto type =
            this->containsFlag(flags, "supported-formats", "-i")?
                IpcBridge::StreamType_Input:
                IpcBridge::StreamType_Output;
    auto formats = this->m_ipcBridge.supportedPixelFormats(type);

    if (!this->m_parseable) {
        if (type == IpcBridge::StreamType_Input)
            std::cout << "Input formats:" << std::endl;
        else
            std::cout << "Output formats:" << std::endl;

        std::cout << std::endl;
    }

    for (auto &format: formats)
        std::cout << VideoFormat::stringFromFourcc(format) << std::endl;

    return 0;
}

int AkVCam::CmdParserPrivate::showDefaultFormat(const AkVCam::StringMap &flags,
                                                const AkVCam::StringVector &args)
{
    UNUSED(args);

    auto type =
            this->containsFlag(flags, "default-format", "-i")?
                IpcBridge::StreamType_Input:
                IpcBridge::StreamType_Output;
    auto format = this->m_ipcBridge.defaultPixelFormat(type);
    std::cout << VideoFormat::stringFromFourcc(format) << std::endl;

    return 0;
}

int AkVCam::CmdParserPrivate::showFormats(const StringMap &flags,
                                          const StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Device not provided." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto it = std::find(devices.begin(), devices.end(), deviceId);

    if (it == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    if (this->m_parseable) {
        for  (auto &format: this->m_ipcBridge.formats(args[1]))
            std::cout << VideoFormat::stringFromFourcc(format.fourcc())
                      << ' '
                      << format.width()
                      << ' '
                      << format.height()
                      << ' '
                      << format.minimumFrameRate().num()
                      << ' '
                      << format.minimumFrameRate().den()
                      << std::endl;
    } else {
        int i = 0;

        for  (auto &format: this->m_ipcBridge.formats(args[1])) {
            std::cout << i
                      << ": "
                      << VideoFormat::stringFromFourcc(format.fourcc())
                      << ' '
                      << format.width()
                      << 'x'
                      << format.height()
                      << ' '
                      << format.minimumFrameRate().num()
                      << '/'
                      << format.minimumFrameRate().den()
                      << " FPS"
                      << std::endl;
            i++;
        }
    }

    return 0;
}

int AkVCam::CmdParserPrivate::addFormat(const StringMap &flags,
                                        const StringVector &args)
{
    if (args.size() < 6) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    auto format = VideoFormat::fourccFromString(args[2]);

    if (!format) {
        std::cerr << "Invalid pixel format." << std::endl;

        return -EINVAL;
    }

    auto formats =
            this->m_ipcBridge.supportedPixelFormats(IpcBridge::StreamType_Output);
    auto fit = std::find(formats.begin(), formats.end(), format);

    if (fit == formats.end()) {
        std::cerr << "Format not supported." << std::endl;

        return -EINVAL;
    }

    char *p = nullptr;
    auto width = strtoul(args[3].c_str(), &p, 10);

    if (*p) {
        std::cerr << "Width must be an unsigned integer." << std::endl;

        return -EINVAL;
    }

    p = nullptr;
    auto height = strtoul(args[4].c_str(), &p, 10);

    if (*p) {
        std::cerr << "Height must be an unsigned integer." << std::endl;

        return -EINVAL;
    }

    Fraction fps(args[5]);

    if (fps.num() < 1 || fps.den() < 1) {
        std::cerr << "Invalid frame rate." << std::endl;

        return -EINVAL;
    }

    auto indexStr = this->flagValue(flags, "add-format", "-i");
    int index = -1;

    if (!indexStr.empty()) {
        p = nullptr;
        index = int(strtoul(indexStr.c_str(), &p, 10));

        if (*p) {
            std::cerr << "Index must be an unsigned integer." << std::endl;

            return -EINVAL;
        }
    }

    VideoFormat fmt(format, int(width), int(height), {fps});
    this->m_ipcBridge.addFormat(deviceId, fmt, index);

    return 0;
}

int AkVCam::CmdParserPrivate::removeFormat(const StringMap &flags,
                                           const StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 3) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    char *p = nullptr;
    auto index = strtoul(args[2].c_str(), &p, 10);

    if (*p) {
        std::cerr << "Index must be an unsigned integer." << std::endl;

        return -EINVAL;
    }

    auto formats = this->m_ipcBridge.formats(deviceId);

    if (index >= formats.size()) {
        std::cerr << "Index is out of range." << std::endl;

        return -ERANGE;
    }

    this->m_ipcBridge.removeFormat(deviceId, int(index));

    return 0;
}

int AkVCam::CmdParserPrivate::removeFormats(const AkVCam::StringMap &flags,
                                           const AkVCam::StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    this->m_ipcBridge.setFormats(deviceId, {});

    return 0;
}

int AkVCam::CmdParserPrivate::update(const StringMap &flags,
                                     const StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);
    this->m_ipcBridge.updateDevices();

    return 0;
}

int AkVCam::CmdParserPrivate::loadSettings(const AkVCam::StringMap &flags,
                                           const AkVCam::StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Settings file not provided." << std::endl;

        return -EINVAL;
    }

    Settings settings;

    if (!settings.load(args[1])) {
        std::cerr << "Settings file not valid." << std::endl;

        return -EIO;
    }

    this->loadGenerals(settings);
    auto devices = this->m_ipcBridge.devices();

    for (auto &device: devices)
        this->m_ipcBridge.removeDevice(device);

    this->createDevices(settings, this->readFormats(settings));

    return 0;
}

int AkVCam::CmdParserPrivate::stream(const AkVCam::StringMap &flags,
                                     const AkVCam::StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 5) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    auto format = VideoFormat::fourccFromString(args[2]);

    if (!format) {
        std::cerr << "Invalid pixel format." << std::endl;

        return -EINVAL;
    }

    auto formats =
            this->m_ipcBridge.supportedPixelFormats(IpcBridge::StreamType_Output);
    auto fit = std::find(formats.begin(), formats.end(), format);

    if (fit == formats.end()) {
        std::cerr << "Format not supported." << std::endl;

        return -EINVAL;
    }

    char *p = nullptr;
    auto width = strtoul(args[3].c_str(), &p, 10);

    if (*p) {
        std::cerr << "Width must be an unsigned integer." << std::endl;

        return -EINVAL;
    }

    p = nullptr;
    auto height = strtoul(args[4].c_str(), &p, 10);

    if (*p) {
        std::cerr << "Height must be an unsigned integer." << std::endl;

        return -EINVAL;
    }

    auto fpsStr = this->flagValue(flags, "stream", "-f");
    double fps = std::numeric_limits<double>::quiet_NaN();

    if (!fpsStr.empty()) {
        p = nullptr;
        fps = int(strtod(fpsStr.c_str(), &p));

        if (*p) {
            if (!Fraction::isFraction(fpsStr)) {
                std::cerr << "The framerate must be a number or a fraction." << std::endl;

                return -EINVAL;
            }

            fps = Fraction(fpsStr).value();
        }

        if (fps <= 0 || std::isinf(fps)) {
            std::cerr << "The framerate is out of range." << std::endl;

            return -ERANGE;
        }
    }

    VideoFormat fmt(format, int(width), int(height), {{30, 1}});

    if (!this->m_ipcBridge.deviceStart(IpcBridge::StreamType_Output, deviceId)) {
        std::cerr << "Can't start stream." << std::endl;

        return -EIO;
    }

    static bool exit = false;
    auto signalHandler = [] (int) {
        exit = true;
    };
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

#ifndef _WIN32
    signal(SIGPIPE, [] (int) {
    });
#endif

    AkVCam::VideoFrame frame(fmt);
    size_t bufferSize = 0;

#ifdef _WIN32
    // Set std::cin in binary mode.
    _setmode(_fileno(stdin), _O_BINARY);
#endif

    auto clock = [] (const std::chrono::time_point<std::chrono::high_resolution_clock> &since) -> double {
        return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - since).count();
    };

    const double minThreshold = 0.04;
    const double maxThreshold = 0.1;
    const double framedupThreshold = 0.1;
    const double nosyncThreshold = 10.0;

    double lastPts = 0.0;
    auto t0 = std::chrono::high_resolution_clock::now();
    double drift = 0;
    uint64_t i = 0;

    do {
        std::cin.read(reinterpret_cast<char *>(frame.data().data()
                                               + bufferSize),
                      std::streamsize(frame.data().size() - bufferSize));
        bufferSize += size_t(std::cin.gcount());

        if (bufferSize == frame.data().size()) {
            if (fpsStr.empty()) {
                this->m_ipcBridge.write(deviceId, frame);
            } else {
                double pts = double(i) / fps;

                for (;;) {
                    double clock_pts = clock(t0) + drift;
                    double diff = pts - clock_pts;
                    double delay = pts - lastPts;
                    double syncThreshold =
                            std::max(minThreshold,
                                     std::min(delay, maxThreshold));

                    if (!std::isnan(diff)
                        && std::abs(diff) < nosyncThreshold
                        && delay < framedupThreshold) {
                        if (diff <= -syncThreshold) {
                            lastPts = pts;

                            break;
                        }

                        if (diff > syncThreshold) {
                            std::this_thread::sleep_for(std::chrono::duration<double>(diff - syncThreshold));

                            continue;
                        }
                    } else {
                        drift = clock(t0) - pts;
                    }

                    this->m_ipcBridge.write(deviceId, frame);
                    lastPts = pts;

                    break;
                }

                i++;
            }

            bufferSize = 0;
        }
    } while (!std::cin.eof() && !exit);

    this->m_ipcBridge.deviceStop(deviceId);

    return 0;
}

int AkVCam::CmdParserPrivate::listenEvents(const AkVCam::StringMap &flags,
                                           const AkVCam::StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);

    auto devicesChanged = [] (void *, const std::vector<std::string> &) {
        std::cout << "DevicesUpdated" << std::endl;
    };
    auto pictureChanged = [] (void *, const std::string &) {
        std::cout << "PictureUpdated" << std::endl;
    };

    this->m_ipcBridge.connectDevicesChanged(this, devicesChanged);
    this->m_ipcBridge.connectPictureChanged(this, pictureChanged);

    static bool exit = false;
    auto signalHandler = [] (int) {
        exit = true;
    };
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    while (!exit)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return 0;
}

int AkVCam::CmdParserPrivate::showControls(const StringMap &flags,
                                           const StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Device not provided." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    if (this->m_parseable) {
        for (auto &control: this->m_ipcBridge.controls(deviceId))
            std::cout << control.id << std::endl;
    } else {
        auto typeStr = typeStrMap();

        std::vector<std::string> table {
            "Control",
            "Description",
            "Type",
            "Minimum",
            "Maximum",
            "Step",
            "Default",
            "Value"
        };
        auto columns = table.size();

        for (auto &control: this->m_ipcBridge.controls(deviceId)) {
            table.push_back(control.id);
            table.push_back(control.description);
            table.push_back(typeStr[control.type]);
            table.push_back(std::to_string(control.minimum));
            table.push_back(std::to_string(control.maximum));
            table.push_back(std::to_string(control.step));
            table.push_back(std::to_string(control.defaultValue));
            table.push_back(std::to_string(control.value));
        }

        this->drawTable(table, columns);
    }

    return 0;
}

int AkVCam::CmdParserPrivate::readControl(const StringMap &flags,
                                          const StringVector &args)
{
    if (args.size() < 3) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    for (auto &control: this->m_ipcBridge.controls(deviceId))
        if (control.id == args[2]) {
            if (flags.empty()) {
                std::cout << control.value << std::endl;
            } else {
                if (this->containsFlag(flags, "get-control", "-c")) {
                    std::cout << control.description << std::endl;
                }

                if (this->containsFlag(flags, "get-control", "-t")) {
                    auto typeStr = typeStrMap();
                    std::cout << typeStr[control.type] << std::endl;
                }

                if (this->containsFlag(flags, "get-control", "-m")) {
                    std::cout << control.minimum << std::endl;
                }

                if (this->containsFlag(flags, "get-control", "-M")) {
                    std::cout << control.maximum << std::endl;
                }

                if (this->containsFlag(flags, "get-control", "-s")) {
                    std::cout << control.step << std::endl;
                }

                if (this->containsFlag(flags, "get-control", "-d")) {
                    std::cout << control.defaultValue << std::endl;
                }

                if (this->containsFlag(flags, "get-control", "-l")) {
                    for (size_t i = 0; i < control.menu.size(); i++)
                        if (this->m_parseable)
                            std::cout << control.menu[i] << std::endl;
                        else
                            std::cout << i
                                      << ": "
                                      << control.menu[i]
                                      << std::endl;
                }
            }

            return 0;
        }

    std::cerr << "'" << args[2] << "' control not available." << std::endl;

    return -ENOSYS;
}

int AkVCam::CmdParserPrivate::writeControls(const StringMap &flags,
                                            const StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 3) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto deviceId = args[1];
    auto devices = this->m_ipcBridge.devices();
    auto dit = std::find(devices.begin(), devices.end(), deviceId);

    if (dit == devices.end()) {
        std::cerr << "'" << deviceId << "' doesn't exists." << std::endl;

        return -ENODEV;
    }

    std::map<std::string, int> controls;

    for (size_t i = 2; i < args.size(); i++) {
        if (args[i].find('=') == std::string::npos) {
            std::cerr << "Argumment "
                      << i
                      << " is not in the form KEY=VALUE."
                      << std::endl;

            return -EINVAL;
        }

        auto pair = splitOnce(args[i], "=");

        if (pair.first.empty()) {
            std::cerr << "Key for argumment "
                      << i
                      << " is emty."
                      << std::endl;

            return -EINVAL;
        }

        auto key = trimmed(pair.first);
        auto value = trimmed(pair.second);
        bool found = false;

        for (auto &control: this->m_ipcBridge.controls(deviceId))
                if (control.id == key) {
                    switch (control.type) {
                    case ControlTypeInteger: {
                        char *p = nullptr;
                        auto val = strtol(value.c_str(), &p, 10);

                        if (*p) {
                            std::cerr << "Value at argument "
                                      << i
                                      << " must be an integer."
                                      << std::endl;

                            return -EINVAL;
                        }

                        controls[key] = val;

                        break;
                    }

                    case ControlTypeBoolean: {
                        std::locale loc;
                        std::transform(value.begin(),
                                       value.end(),
                                       value.begin(),
                                       [&loc](char c) {
                            return std::tolower(c, loc);
                        });

                        if (value == "0" || value == "false") {
                            controls[key] = 0;
                        } else if (value == "1" || value == "true") {
                            controls[key] = 1;
                        } else {
                            std::cerr << "Value at argument "
                                      << i
                                      << " must be a boolean."
                                      << std::endl;

                            return -EINVAL;
                        }

                        break;
                    }

                    case ControlTypeMenu: {
                        char *p = nullptr;
                        auto val = strtoul(value.c_str(), &p, 10);

                        if (*p) {
                            auto it = std::find(control.menu.begin(),
                                                control.menu.end(),
                                                value);

                            if (it == control.menu.end()) {
                                std::cerr << "Value at argument "
                                          << i
                                          << " is not valid."
                                          << std::endl;

                                return -EINVAL;
                            }

                            controls[key] = int(it - control.menu.begin());
                        } else {
                            if (val >= control.menu.size()) {
                                std::cerr << "Value at argument "
                                          << i
                                          << " is out of range."
                                          << std::endl;

                                return -ERANGE;
                            }

                            controls[key] = int(val);
                        }

                        break;
                    }

                    default:
                        break;
                    }

                    found = true;

                    break;
                }

        if (!found) {
            std::cerr << "No such '"
                      << key
                      << "' control in argument "
                      << i
                      << "."
                      << std::endl;

            return -ENOSYS;
        }
    }

    this->m_ipcBridge.setControls(deviceId, controls);

    return 0;
}

int AkVCam::CmdParserPrivate::picture(const AkVCam::StringMap &flags,
                                      const AkVCam::StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);

    std::cout << this->m_ipcBridge.picture() << std::endl;

    return 0;
}

int AkVCam::CmdParserPrivate::setPicture(const AkVCam::StringMap &flags,
                                         const AkVCam::StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    this->m_ipcBridge.setPicture(args[1]);

    return 0;
}

int AkVCam::CmdParserPrivate::logLevel(const AkVCam::StringMap &flags,
                                       const AkVCam::StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);

    auto level = this->m_ipcBridge.logLevel();

    if (this->m_parseable)
        std::cout << level << std::endl;
    else
        std::cout << AkVCam::Logger::levelToString(level) << std::endl;

    return 0;
}

int AkVCam::CmdParserPrivate::setLogLevel(const AkVCam::StringMap &flags,
                                          const AkVCam::StringVector &args)
{
    UNUSED(flags);

    if (args.size() < 2) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto levelStr = args[1];
    char *p = nullptr;
    auto level = strtol(levelStr.c_str(), &p, 10);

    if (*p)
        level = AkVCam::Logger::levelFromString(levelStr);

    this->m_ipcBridge.setLogLevel(level);

    return 0;
}

int AkVCam::CmdParserPrivate::showClients(const StringMap &flags,
                                          const StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);
    auto clients = this->m_ipcBridge.clientsPids();

    if (clients.empty())
        return 0;

    if (this->m_parseable) {
        for (auto &pid: clients)
            std::cout << pid
                      << " "
                      << this->m_ipcBridge.clientExe(pid)
                      << std::endl;
    } else {
        std::vector<std::string> table {
            "Pid",
            "Executable"
        };
        auto columns = table.size();

        for (auto &pid: clients) {
            table.push_back(std::to_string(pid));
            table.push_back(this->m_ipcBridge.clientExe(pid));
        }

        this->drawTable(table, columns);
    }

    return 0;
}

int AkVCam::CmdParserPrivate::dumpInfo(const AkVCam::StringMap &flags,
                                       const AkVCam::StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);
    static const auto indent = 4 * std::string(" ");

    std::cout << "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" << std::endl;
    std::cout << "<info>" << std::endl;
    std::cout << indent << "<devices>" << std::endl;

    auto devices = this->m_ipcBridge.devices();

    for (auto &device: devices) {
        std::cout << 2 * indent << "<device>" << std::endl;
        std::cout << 3 * indent << "<id>" << device << "</id>" << std::endl;
        std::cout << 3 * indent
                  << "<description>"
                  << this->m_ipcBridge.description(device)
                  << "</description>"
                  << std::endl;
        std::cout << 3 * indent << "<formats>" << std::endl;

        for  (auto &format: this->m_ipcBridge.formats(device)) {
            std::cout << 4 * indent << "<format>" << std::endl;
            std::cout << 5 * indent
                      << "<pixel-format>"
                      << VideoFormat::stringFromFourcc(format.fourcc())
                      << "</pixel-format>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<width>"
                      << format.width()
                      << "</width>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<height>"
                      << format.height()
                      << "</height>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<fps>"
                      << format.minimumFrameRate().toString()
                      << "</fps>"
                      << std::endl;
            std::cout << 4 * indent << "</format>" << std::endl;
        }

        std::cout << 3 * indent << "</formats>" << std::endl;
        std::cout << 3 * indent << "<controls>" << std::endl;

        for (auto &control: this->m_ipcBridge.controls(device)) {
            std::cout << 4 * indent << "<control>" << std::endl;
            std::cout << 5 * indent
                      << "<id>"
                      << control.id
                      << "</id>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<description>"
                      << control.description
                      << "</description>"
                      << std::endl;
            auto typeStr = typeStrMap();
            std::cout << 5 * indent
                      << "<type>"
                      << typeStr[control.type]
                      << "</type>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<minimum>"
                      << control.minimum
                      << "</minimum>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<maximum>"
                      << control.maximum
                      << "</maximum>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<step>"
                      << control.step
                      << "</step>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<default-value>"
                      << control.defaultValue
                      << "</default-value>"
                      << std::endl;
            std::cout << 5 * indent
                      << "<value>"
                      << control.value
                      << "</value>"
                      << std::endl;

            if (!control.menu.empty() && control.type == ControlTypeMenu) {
                std::cout << 5 * indent << "<menu>" << std::endl;

                for (auto &item: control.menu)
                    std::cout << 6 * indent
                              << "<item>"
                              << item
                              << "</item>"
                              << std::endl;

                std::cout << 5 * indent << "</menu>" << std::endl;
            }

            std::cout << 4 * indent << "</control>" << std::endl;
        }

        std::cout << 3 * indent << "</controls>" << std::endl;
        std::cout << 2 * indent << "</device>" << std::endl;
    }

    std::cout << indent << "</devices>" << std::endl;
    std::cout << indent << "<input-formats>" << std::endl;

    for (auto &format: this->m_ipcBridge.supportedPixelFormats(IpcBridge::StreamType_Input))
        std::cout << 2 * indent
                  << "<pixel-format>"
                  << VideoFormat::stringFromFourcc(format)
                  << "</pixel-format>"
                  << std::endl;

    std::cout << indent << "</input-formats>" << std::endl;

    auto defInputFormat =
            this->m_ipcBridge.defaultPixelFormat(IpcBridge::StreamType_Input);
    std::cout << indent
              << "<default-input-format>"
              << VideoFormat::stringFromFourcc(defInputFormat)
              << "</default-input-format>"
              << std::endl;

    std::cout << indent << "<output-formats>" << std::endl;

    for (auto &format: this->m_ipcBridge.supportedPixelFormats(IpcBridge::StreamType_Output))
        std::cout << 2 * indent
                  << "<pixel-format>"
                  << VideoFormat::stringFromFourcc(format)
                  << "</pixel-format>"
                  << std::endl;

    std::cout << indent << "</output-formats>" << std::endl;

    auto defOutputFormat =
            this->m_ipcBridge.defaultPixelFormat(IpcBridge::StreamType_Output);
    std::cout << indent
              << "<default-output-format>"
              << VideoFormat::stringFromFourcc(defOutputFormat)
              << "</default-output-format>"
              << std::endl;

    std::cout << indent << "<clients>" << std::endl;

   for (auto &pid: this->m_ipcBridge.clientsPids()) {
       std::cout << 2 * indent << "<client>" << std::endl;
       std::cout << 3 * indent << "<pid>" << pid << "</pid>" << std::endl;
       std::cout << 3 * indent
                 << "<exe>"
                 << this->m_ipcBridge.clientExe(pid)
                 << "</exe>"
                 << std::endl;
       std::cout << 2 * indent << "</client>" << std::endl;
   }

    std::cout << indent << "</clients>" << std::endl;
    std::cout << indent
              << "<picture>"
              << this->m_ipcBridge.picture()
              << "</picture>"
              << std::endl;
    std::cout << indent
              << "<loglevel>"
              << this->m_ipcBridge.logLevel()
              << "</loglevel>"
              << std::endl;
    std::cout << "</info>" << std::endl;

    return 0;
}

int AkVCam::CmdParserPrivate::hacks(const AkVCam::StringMap &flags,
                                    const AkVCam::StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);

    auto hacks = this->m_ipcBridge.hacks();

    if (hacks.empty())
        return 0;

    if (this->m_parseable) {
        for (auto &hack: hacks)
            std::cout << hack << std::endl;
    } else {
        std::cout << "Hacks are intended to fix common problems with the "
                     "virtual camera, and are intended to be used by developers "
                     "and advanced users only." << std::endl;
        std::cout << std::endl;
        std::cout << "WARNING: Unsafe hacks can brick your system, make it "
                     "unstable, or expose it to a serious security risk, "
                     "remember to make a backup of your files and system. You "
                     "are solely responsible of whatever happens for using "
                     "them. You been warned, don't come and cry later."
                  << std::endl;
        std::cout << std::endl;
        std::vector<std::string> table {
            "Hack",
            "Is safe?",
            "Description"
        };
        auto columns = table.size();

        for (auto &hack: hacks) {
            table.push_back(hack);
            table.push_back(this->m_ipcBridge.hackIsSafe(hack)? "Yes": "No");
            table.push_back(this->m_ipcBridge.hackDescription(hack));
        }

        this->drawTable(table, columns);
    }

    return 0;
}

int AkVCam::CmdParserPrivate::hackInfo(const AkVCam::StringMap &flags,
                                       const AkVCam::StringVector &args)
{
    if (args.size() < 2) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto hack = args[1];
    auto hacks = this->m_ipcBridge.hacks();
    auto dit = std::find(hacks.begin(), hacks.end(), hack);

    if (dit == hacks.end()) {
        std::cerr << "Unknown hack: " << hack << "." << std::endl;

        return -ENOSYS;
    }

    if (this->containsFlag(flags, "hack-info", "-c"))
        std::cout << this->m_ipcBridge.hackDescription(hack) << std::endl;

    if (this->containsFlag(flags, "hack-info", "-s")) {
        if (this->m_ipcBridge.hackIsSafe(hack))
            std::cout << "Yes" << std::endl;
        else
            std::cout << "No" << std::endl;
    }

    return 0;
}

int AkVCam::CmdParserPrivate::hack(const AkVCam::StringMap &flags,
                                   const AkVCam::StringVector &args)
{
    if (args.size() < 2) {
        std::cerr << "Not enough arguments." << std::endl;

        return -EINVAL;
    }

    auto hack = args[1];
    auto hacks = this->m_ipcBridge.hacks();
    auto dit = std::find(hacks.begin(), hacks.end(), hack);

    if (dit == hacks.end()) {
        std::cerr << "Unknown hack: " << hack << "." << std::endl;

        return -ENOSYS;
    }

    bool accepted = this->m_parseable | this->m_ipcBridge.hackIsSafe(hack);

    if (!accepted && !this->m_parseable) {
        std::cout << "WARNING: Applying this hack can brick your system, make "
                     "it unstable, or expose it to a serious security risk, "
                     "remember to make a backup of your files and system. "
                     "Agreeing to continue, you accept the full responsability "
                     "of whatever happens from now on."
                  << std::endl;
        std::cout << std::endl;

        if (this->containsFlag(flags, "hack", "-y")) {
            std::cout << "You agreed to continue from command line."
                      << std::endl;
            std::cout << std::endl;
            accepted = true;
        } else {
            std::cout << "If you agree to continue write YES: ";
            std::string answer;
            std::cin >> answer;
            std::cout << std::endl;
            accepted = answer == "YES";
        }
    }

    if (!accepted) {
        std::cerr << "Hack not applied." << std::endl;

        return -EIO;
    }

    StringVector hargs;

    for (size_t i = 2; i < args.size(); i++)
        hargs.push_back(args[i]);

    auto result = this->m_ipcBridge.execHack(hack, hargs);

    if (result == 0)
        std::cout << "Success" << std::endl;
    else
        std::cout << "Failed" << std::endl;

    return result;
}

int AkVCam::CmdParserPrivate::listPhysicalCameras(const StringMap &flags, const StringVector &args)
{
    UNUSED(flags);
    UNUSED(args);

    auto cameras = AkVCam::list_physical_cameras_impl();

    if (cameras.empty()) {
        if (!this->m_parseable) {
            std::cout << "No physical cameras found or error enumerating them." << std::endl;
        }
        return 0; // Not an error, could be no cameras connected
    }

    if (this->m_parseable) {
        for (const auto &cam : cameras) {
            std::cout << cam.id << "\t" << cam.name << std::endl;
        }
    } else {
        StringVector table_data;
        table_data.push_back("ID");
        table_data.push_back("Name");

        for (const auto &cam : cameras) {
            table_data.push_back(cam.id);
            table_data.push_back(cam.name);
        }
        this->drawTable(table_data, 2); // 2 columns
    }

    return 0;
}

int AkVCam::CmdParserPrivate::splitWebcam(const StringMap &flags, const StringVector &args) {
    UNUSED(flags);
    // args[0] is program name
    // args[1] is physical_cam_id
    // args[2] is num_splits
    // args[3] is optional base_name_prefix

    if (args.size() < 3) {
        std::cerr << "Usage: " << args[0] << " split-webcam <physical_cam_id> <num_splits> [base_name_prefix]" << std::endl;
        return -EINVAL;
    }

    const std::string& physical_cam_id = args[1];
    int num_splits = 0;
    try {
        num_splits = std::stoi(args[2]);
    } catch (const std::exception& e) {
        std::cerr << "Invalid number for num_splits: " << args[2] << std::endl;
        return -EINVAL;
    }

    if (num_splits <= 0) {
        std::cerr << "num_splits must be a positive integer." << std::endl;
        return -EINVAL;
    }

    std::string base_name_prefix = "SplitCam";
    if (args.size() >= 4) {
        base_name_prefix = args[3];
    }

    // Check if this physical camera is already configured for splitting
    if (m_active_splits.count(physical_cam_id)) {
        std::cerr << "Error: Physical camera '" << physical_cam_id << "' is already configured for splitting." << std::endl;
        std::cerr << "Remove the existing split configuration first using 'remove-split " << physical_cam_id << "'." << std::endl;
        return -EEXIST;
    }

    // Placeholder: Validate physical_cam_id by checking against list_physical_cameras_impl()
    auto physical_cameras = AkVCam::list_physical_cameras_impl();
    auto it_phys = std::find_if(physical_cameras.begin(), physical_cameras.end(),
        [&](const AkVCam::PhysicalCamera& cam){ return cam.id == physical_cam_id || cam.name == physical_cam_id; });

    if (it_phys == physical_cameras.end()) {
        std::cerr << "Error: Physical camera '" << physical_cam_id << "' not found." << std::endl;
        return -ENODEV;
    }
    const std::string actual_physical_cam_id = it_phys->id; // Use the ID for consistency
    const std::string physical_cam_name = it_phys->name;


    if (!this->m_parseable) {
        std::cout << "Configuring physical camera '" << physical_cam_name << "' (ID: " << actual_physical_cam_id << ") to be split into " << num_splits << " virtual cameras with base name '" << base_name_prefix << "'." << std::endl;
    }

    WebcamSplitConfig new_split_config;
    new_split_config.physical_camera_id = actual_physical_cam_id;
    new_split_config.physical_camera_name = physical_cam_name;
    new_split_config.num_splits = num_splits;
    new_split_config.base_name_prefix = base_name_prefix;
    new_split_config.is_active = false;

    for (int i = 0; i < num_splits; ++i) {
        std::string virtual_cam_name = base_name_prefix + " " + std::to_string(i + 1);
        // The addDevice method can generate an ID if one isn't provided.
        // Or, we can construct a unique ID here. For now, let addDevice handle it or generate one.
        std::string generated_virtual_id_base = actual_physical_cam_id + "_split_" + std::to_string(i);

        // For now, we'll simulate ID generation for planning. Actual ID comes from addDevice.
        std::string virtual_cam_id = "TEMP_VID_" + generated_virtual_id_base;

        if (!this->m_parseable) {
            std::cout << "  Registering virtual camera: " << virtual_cam_name << " (intended ID base: " << generated_virtual_id_base << ")" << std::endl;
        }
        // TODO: Call this->m_ipcBridge.addDevice(virtual_cam_name, generated_virtual_id_base);
        //       And retrieve the actual ID returned by addDevice.
        //       For now, using placeholder ID.
        // 실제 구현에서는 m_ipcBridge.addDevice를 호출하고 반환된 ID를 사용해야 합니다.
        // std::string actual_virtual_cam_id = this->m_ipcBridge.addDevice(virtual_cam_name, generated_virtual_id_base);
        // if (actual_virtual_cam_id.empty()) {
        //    std::cerr << "Error: Failed to create virtual camera device " << virtual_cam_name << std::endl;
        //    // Rollback: remove previously created virtual devices for this split
        //    for(const auto& vid : new_split_config.virtual_camera_ids) {
        //        this->m_ipcBridge.removeDevice(vid);
        //    }
        //    return -EIO;
        // }
        // new_split_config.virtual_camera_ids.push_back(actual_virtual_cam_id);
        //new_split_config.virtual_camera_ids.push_back(virtual_cam_id); // Using placeholder

        // Actually add the device via IPCBridge
        std::string actual_virtual_cam_id = this->m_ipcBridge.addDevice(virtual_cam_name, generated_virtual_id_base);
        if (actual_virtual_cam_id.empty()) {
            std::cerr << "Error: Failed to create virtual camera device '" << virtual_cam_name << "' (intended ID base: " << generated_virtual_id_base << ")." << std::endl;
            // Rollback: remove previously created virtual devices for this split
            if (!this->m_parseable) {
                std::cerr << "Rolling back previously added virtual cameras for this split attempt..." << std::endl;
            }
            for(const auto& vid : new_split_config.virtual_camera_ids) {
                this->m_ipcBridge.removeDevice(vid);
                 if (!this->m_parseable) {
                    std::cout << "  Removed device: " << vid << std::endl;
                }
            }
            return -EIO;
        }
        new_split_config.virtual_camera_ids.push_back(actual_virtual_cam_id);
        AkLogInfo() << "splitWebcam: Registered virtual camera '" << virtual_cam_name << "' with actual ID: " << actual_virtual_cam_id << std::endl;

        if (this->m_parseable) {
            std::cout << actual_virtual_cam_id << "\t" << virtual_cam_name << std::endl;
        } else {
            std::cout << "  Successfully registered virtual camera: " << virtual_cam_name << " (ID: " << actual_virtual_cam_id << ")" << std::endl;
        }
    }

    m_active_splits[actual_physical_cam_id] = new_split_config;

    // --- Set default formats for the new virtual cameras ---
    std::vector<VideoFormat> default_formats_to_set;
    // Create a temporary capture instance to get formats
    auto temp_capture = create_platform_camera_capture();
    if (temp_capture) {
        if (temp_capture->open(actual_physical_cam_id)) {
            auto physical_formats = temp_capture->getSupportedFormats();
            if (!physical_formats.empty()) {
                // Try to find a common/preferred format, e.g., 640x480 YUY2 or NV12, or just pick the first one
                // For simplicity, let's pick the first one that is reasonably common if possible.
                // A more sophisticated selection could be implemented here.
                auto preferred_it = std::find_if(physical_formats.begin(), physical_formats.end(), [](const VideoFormat& vf){
                    return (vf.width() == 640 && vf.height() == 480 && (vf.fourcc() == PixelFormatYUY2 || vf.fourcc() == PixelFormatNV12)) ||
                           (vf.width() == 1280 && vf.height() == 720 && (vf.fourcc() == PixelFormatYUY2 || vf.fourcc() == PixelFormatNV12));
                });
                if (preferred_it != physical_formats.end()) {
                    default_formats_to_set.push_back(*preferred_it);
                } else {
                    default_formats_to_set.push_back(physical_formats.front()); // Fallback to the first format
                }
                AkLogInfo() << "splitWebcam: Using format " << default_formats_to_set.front().toString() << " for virtual cameras." << std::endl;
            } else {
                AkLogWarn() << "splitWebcam: Physical camera reported no supported formats. Virtual cameras might not work." << std::endl;
            }
            temp_capture->close();
        } else {
            AkLogWarn() << "splitWebcam: Could not open physical camera temporarily to get formats." << std::endl;
        }
    } else {
        AkLogWarn() << "splitWebcam: Could not create temp camera capture instance to get formats." << std::endl;
    }

    if (default_formats_to_set.empty()) {
        // Fallback to a very common default if physical camera formats couldn't be queried
        AkLogWarn() << "splitWebcam: Using hardcoded fallback format for virtual cameras." << std::endl;
        default_formats_to_set.push_back(VideoFormat(PixelFormatYUY2, 640, 480, {{30,1}}));
    }

    for (const auto& v_id : new_split_config.virtual_camera_ids) {
        if (!this->m_ipcBridge.setFormats(v_id, default_formats_to_set)) {
            AkLogWarn() << "splitWebcam: Failed to set formats for virtual camera " << v_id << std::endl;
            // This is not ideal, the virtual camera might not be usable by some apps.
            // Consider if this should be a critical error leading to rollback. For now, just a warning.
        } else {
            if (!this->m_parseable) {
                std::cout << "  Set format for " << v_id << " to " << default_formats_to_set.front().toString() << std::endl;
            }
        }
    }
    // --- End set default formats ---


    if (!this->m_parseable) {
        std::cout << "Webcam split configured. Use 'start-split " << actual_physical_cam_id << "' to begin streaming." << std::endl;
    }

    return 0;
}

int AkVCam::CmdParserPrivate::startSplit(const StringMap &flags, const StringVector &args) {
    UNUSED(flags);
    if (args.size() < 2) {
        std::cerr << "Usage: " << args[0] << " start-split <physical_cam_id>" << std::endl;
        return -EINVAL;
    }
    const std::string& physical_cam_id_arg = args[1];

    auto it = m_active_splits.find(physical_cam_id_arg);
    if (it == m_active_splits.end()) {
        // Also check if user provided name instead of ID
        auto it_by_name = std::find_if(m_active_splits.begin(), m_active_splits.end(),
            [&](const auto& pair){ return pair.second.physical_camera_name == physical_cam_id_arg; });
        if (it_by_name != m_active_splits.end()) {
            it = it_by_name;
        } else {
            std::cerr << "Error: No split configuration found for physical camera '" << physical_cam_id_arg << "'." << std::endl;
            std::cerr << "Use 'split-webcam' command first." << std::endl;
            return -ENOENT;
        }
    }

    WebcamSplitConfig& config = it->second;

    if (config.is_active) {
        std::cerr << "Error: Split for physical camera '" << config.physical_camera_name << "' (ID: " << config.physical_camera_id << ") is already active." << std::endl;
        return -EALREADY;
    }

    if (!this->m_parseable) {
        std::cout << "Starting split for physical camera '" << config.physical_camera_name << "' (ID: " << config.physical_camera_id << ")..." << std::endl;
        std::cout << "  (Placeholder: Actual camera capture and frame distribution not yet implemented)" << std::endl;
        std::cout << "  Frames would be sent to:" << std::endl;
        for(const auto& vid : config.virtual_camera_ids) {
            std::cout << "    - " << vid << std::endl;
        }
    }

    // TODO: Implement actual physical camera capture initiation here.
    // This will involve platform-specific code.

    if (m_camera_capture && m_camera_capture->isStreaming()) {
        // This case implies another split is already active with the m_camera_capture instance
        if (m_camera_capture->getOpenedDeviceId() != config.physical_camera_id) {
             AkLogError() << "startSplit: Another physical camera '" << m_camera_capture->getOpenedDeviceId() << "' is already active for capture." << std::endl;
             std::cerr << "Error: Another physical camera capture (" << m_camera_capture->getOpenedDeviceId() <<") is already active. Stop it first." << std::endl;
             return -EBUSY; // Or another appropriate error
        }
        // If it's the same camera, it might already be streaming, which is covered by config.is_active
    }

    if (!m_camera_capture) {
        m_camera_capture = create_platform_camera_capture();
    }

    if (!m_camera_capture) {
        AkLogError() << "startSplit: Failed to create platform camera capture instance." << std::endl;
        std::cerr << "Error: Could not initialize camera capture for this platform." << std::endl;
        return -EIO;
    }

    // Check if the camera is already opened by this instance, but not for this specific split's physical_camera_id
    // This is a bit redundant if m_active_capture_physical_id is managed well.
    if (m_camera_capture->getOpenedDeviceId() != "" && m_camera_capture->getOpenedDeviceId() != config.physical_camera_id) {
         AkLogError() << "startSplit: m_camera_capture is already open for a different device ID: " << m_camera_capture->getOpenedDeviceId() << std::endl;
         // This state should ideally not be reached if m_active_capture_physical_id is correctly managed.
         // Close it before proceeding, or return an error. For now, let's try closing.
         m_camera_capture->close();
    }


    if (m_camera_capture->getOpenedDeviceId() != config.physical_camera_id) { // Only open if not already open for this device
        if (!m_camera_capture->open(config.physical_camera_id)) {
            AkLogError() << "startSplit: Failed to open physical camera ID: " << config.physical_camera_id << std::endl;
            std::cerr << "Error: Could not open physical camera '" << config.physical_camera_name << "' (ID: " << config.physical_camera_id << ")." << std::endl;
            m_camera_capture.reset(); // Release the instance if open failed
            return -EIO;
        }
    }

    // Define the frame callback
    auto frame_callback = [this, physical_id = config.physical_camera_id](const VideoFrame& frame) {
        // Ensure we are still supposed to be splitting for this physical_id
        auto it_cb = this->m_active_splits.find(physical_id);
        if (it_cb != this->m_active_splits.end() && it_cb->second.is_active) {
            if (frame.isValid() && frame.format().size() > 0) {
                //AkLogDebug() << "Frame received from " << physical_id << ", distributing to " << it_cb->second.virtual_camera_ids.size() << " virtual cams." << std::endl;
                for (const auto& virtual_cam_id : it_cb->second.virtual_camera_ids) {
                    this->m_ipcBridge.write(virtual_cam_id, frame);
                }
            } else {
                //AkLogWarn() << "Frame callback: Received invalid frame for " << physical_id << std::endl;
            }
        } else {
             //AkLogInfo() << "Frame callback: Split for " << physical_id << " is no longer active or found, stopping distribution." << std::endl;
             // This could be a point to auto-stop the capture if no active splits depend on it.
             // For now, the explicit stopSplit command handles termination.
        }
    };

    if (!m_camera_capture->startStream(frame_callback)) {
        AkLogError() << "startSplit: Failed to start stream for physical camera ID: " << config.physical_camera_id << std::endl;
        std::cerr << "Error: Could not start stream for physical camera '" << config.physical_camera_name << "' (ID: " << config.physical_camera_id << ")." << std::endl;
        m_camera_capture->close(); // Attempt to close if stream failed
        m_camera_capture.reset();
        return -EIO;
    }

    config.is_active = true;
    m_active_capture_physical_id = config.physical_camera_id; // Mark this physical camera as the one being captured

    if (!this->m_parseable) {
        std::cout << "Split started for " << config.physical_camera_name << " (ID: " << config.physical_camera_id << ")." << std::endl;
    }
    return 0;
}

int AkVCam::CmdParserPrivate::stopSplit(const StringMap &flags, const StringVector &args) {
    UNUSED(flags);
    if (args.size() < 2) {
        std::cerr << "Usage: " << args[0] << " stop-split <physical_cam_id>" << std::endl;
        return -EINVAL;
    }
    const std::string& physical_cam_id_arg = args[1];

    auto it = m_active_splits.find(physical_cam_id_arg);
     if (it == m_active_splits.end()) {
        auto it_by_name = std::find_if(m_active_splits.begin(), m_active_splits.end(),
            [&](const auto& pair){ return pair.second.physical_camera_name == physical_cam_id_arg; });
        if (it_by_name != m_active_splits.end()) {
            it = it_by_name;
        } else {
            std::cerr << "Error: No split configuration found for physical camera '" << physical_cam_id_arg << "'." << std::endl;
            return -ENOENT;
        }
    }

    WebcamSplitConfig& config = it->second;

    if (!config.is_active) {
        std::cerr << "Error: Split for physical camera '" << config.physical_camera_name << "' (ID: " << config.physical_camera_id << ") is not currently active." << std::endl;
        return -EINVAL;
    }

    if (!this->m_parseable) {
        std::cout << "Stopping split for physical camera '" << config.physical_camera_name << "' (ID: " << config.physical_camera_id << ")..." << std::endl;
    }

    if (m_camera_capture && m_camera_capture->getOpenedDeviceId() == config.physical_camera_id) {
        m_camera_capture->stopStream();
        // Consider if close() should always be called here, or only if no other splits depend on this physical camera.
        // For now, if this specific split is stopped, we close the camera capture instance associated with it.
        // If other splits were hypothetically sharing the m_camera_capture instance (not current design), this would need more complex ref counting.
        m_camera_capture->close();
        m_camera_capture.reset(); // Release the instance
        m_active_capture_physical_id.clear();
         if (!this->m_parseable) {
            std::cout << "  Physical camera capture stopped and closed." << std::endl;
        }
    } else {
        if (!this->m_parseable) {
            std::cerr << "Warning: No active camera capture instance found for " << config.physical_camera_id << " or it was for a different camera. State might be inconsistent." << std::endl;
        }
    }

    config.is_active = false;

    if (!this->m_parseable) {
        std::cout << "Split stopped." << std::endl;
    }
    return 0;
}

int AkVCam::CmdParserPrivate::removeSplit(const StringMap &flags, const StringVector &args) {
    UNUSED(flags);
    if (args.size() < 2) {
        std::cerr << "Usage: " << args[0] << " remove-split <physical_cam_id>" << std::endl;
        return -EINVAL;
    }
    const std::string& physical_cam_id_arg = args[1];

    auto it = m_active_splits.find(physical_cam_id_arg);
    if (it == m_active_splits.end()) {
        auto it_by_name = std::find_if(m_active_splits.begin(), m_active_splits.end(),
            [&](const auto& pair){ return pair.second.physical_camera_name == physical_cam_id_arg; });
        if (it_by_name != m_active_splits.end()) {
            it = it_by_name;
        } else {
            std::cerr << "Error: No split configuration found for physical camera '" << physical_cam_id_arg << "'." << std::endl;
            return -ENOENT;
        }
    }

    WebcamSplitConfig& config = it->second;

    if (config.is_active) {
        if (!this->m_parseable) {
            std::cout << "Split is active, stopping it first..." << std::endl;
        }
        // Call stopSplit logic (simplified here, actual call would be better)
        // TODO: Call this->stopSplit(...) or refactor to share logic
        config.is_active = false;
        if (!this->m_parseable) {
             std::cout << "Split stopped." << std::endl;
        }
    }

    if (!this->m_parseable) {
        std::cout << "Removing split configuration for physical camera '" << config.physical_camera_name << "' (ID: " << config.physical_camera_id << ")..." << std::endl;
    }

    for (const auto& virtual_cam_id : config.virtual_camera_ids) {
        if (!this->m_parseable) {
            std::cout << "  Unregistering virtual camera: " << virtual_cam_id << std::endl;
        }
        this->m_ipcBridge.removeDevice(virtual_cam_id);
        AkLogInfo() << "removeSplit: Unregistered virtual camera ID: " << virtual_cam_id << std::endl;
    }

    m_active_splits.erase(it);

    if (!this->m_parseable) {
        std::cout << "Split configuration removed." << std::endl;
    }
    return 0;
}


void AkVCam::CmdParserPrivate::loadGenerals(Settings &settings)
{
    settings.beginGroup("General");

    if (settings.contains("default_frame"))
        this->m_ipcBridge.setPicture(settings.value("default_frame"));

    if (settings.contains("loglevel")) {
        auto logLevel= settings.value("loglevel");
        char *p = nullptr;
        auto level = strtol(logLevel.c_str(), &p, 10);

        if (*p)
            level = AkVCam::Logger::levelFromString(logLevel);

        this->m_ipcBridge.setLogLevel(level);
    }

    settings.endGroup();
}

AkVCam::VideoFormatMatrix AkVCam::CmdParserPrivate::readFormats(Settings &settings)
{
    VideoFormatMatrix formatsMatrix;
    settings.beginGroup("Formats");
    auto nFormats = settings.beginArray("formats");

    for (size_t i = 0; i < nFormats; i++) {
        settings.setArrayIndex(i);
        formatsMatrix.push_back(this->readFormat(settings));
    }

    settings.endArray();
    settings.endGroup();

    return formatsMatrix;
}

std::vector<AkVCam::VideoFormat> AkVCam::CmdParserPrivate::readFormat(Settings &settings)
{
    std::vector<AkVCam::VideoFormat> formats;

    auto pixFormats = settings.valueList("format", ",");
    auto widths = settings.valueList("width", ",");
    auto heights = settings.valueList("height", ",");
    auto frameRates = settings.valueList("fps", ",");

    if (pixFormats.empty()
        || widths.empty()
        || heights.empty()
        || frameRates.empty()) {
        std::cerr << "Error reading formats." << std::endl;

        return {};
    }

    StringMatrix formatMatrix;
    formatMatrix.push_back(pixFormats);
    formatMatrix.push_back(widths);
    formatMatrix.push_back(heights);
    formatMatrix.push_back(frameRates);

    for (auto &formatList: this->matrixCombine(formatMatrix)) {
        auto pixFormat = VideoFormat::fourccFromString(formatList[0]);
        char *p = nullptr;
        auto width = strtol(formatList[1].c_str(), &p, 10);
        p = nullptr;
        auto height = strtol(formatList[2].c_str(), &p, 10);
        Fraction frameRate(formatList[3]);
        VideoFormat format(pixFormat,
                           width,
                           height,
                           {frameRate});

        if (format.isValid())
            formats.push_back(format);
    }

    return formats;
}

AkVCam::StringMatrix AkVCam::CmdParserPrivate::matrixCombine(const StringMatrix &matrix)
{
    StringVector combined;
    StringMatrix combinations;
    this->matrixCombineP(matrix, 0, combined, combinations);

    return combinations;
}

/* A matrix is a list of lists where each element in the main list is a row,
 * and each element in a row is a column. We combine each element in a row with
 * each element in the next rows.
 */
void AkVCam::CmdParserPrivate::matrixCombineP(const StringMatrix &matrix,
                                              size_t index,
                                              StringVector combined,
                                              StringMatrix &combinations)
{
    if (index >= matrix.size()) {
        combinations.push_back(combined);

        return;
    }

    for (auto &data: matrix[index]) {
        auto combinedP1 = combined;
        combinedP1.push_back(data);
        this->matrixCombineP(matrix, index + 1, combinedP1, combinations);
    }
}

void AkVCam::CmdParserPrivate::createDevices(Settings &settings,
                                             const VideoFormatMatrix &availableFormats)
{
    for (auto &device: this->m_ipcBridge.devices())
        this->m_ipcBridge.removeDevice(device);

    settings.beginGroup("Cameras");
    size_t nCameras = settings.beginArray("cameras");

    for (size_t i = 0; i < nCameras; i++) {
        settings.setArrayIndex(i);
        this->createDevice(settings, availableFormats);
    }

    settings.endArray();
    settings.endGroup();
    this->m_ipcBridge.updateDevices();
}

void AkVCam::CmdParserPrivate::createDevice(Settings &settings,
                                            const VideoFormatMatrix &availableFormats)
{
    auto description = settings.value("description");

    if (description.empty()) {
        std::cerr << "Device description is empty" << std::endl;

        return;
    }

    auto formats = this->readDeviceFormats(settings, availableFormats);

    if (formats.empty()) {
        std::cerr << "Can't read device formats" << std::endl;

        return;
    }

    auto deviceId = settings.value("id");
    deviceId = this->m_ipcBridge.addDevice(description, deviceId);
    auto supportedFormats =
            this->m_ipcBridge.supportedPixelFormats(IpcBridge::StreamType_Output);

    for (auto &format: formats) {
        auto it = std::find(supportedFormats.begin(),
                            supportedFormats.end(),
                            format.fourcc());

        if (it != supportedFormats.end())
            this->m_ipcBridge.addFormat(deviceId, format, -1);
    }
}

std::vector<AkVCam::VideoFormat> AkVCam::CmdParserPrivate::readDeviceFormats(Settings &settings,
                                                                             const VideoFormatMatrix &availableFormats)
{
    std::vector<AkVCam::VideoFormat> formats;
    auto formatsIndex = settings.valueList("formats", ",");

    for (auto &indexStr: formatsIndex) {
        char *p = nullptr;
        auto index = strtoul(indexStr.c_str(), &p, 10);

        if (*p)
            continue;

        index--;

        if (index >= availableFormats.size())
            continue;

        for (auto &format: availableFormats[index])
            formats.push_back(format);
    }

    return formats;
}

std::string AkVCam::operator *(const std::string &str, size_t n)
{
    std::stringstream ss;

    for (size_t i = 0; i < n; i++)
        ss << str;

    return ss.str();
}

std::string AkVCam::operator *(size_t n, const std::string &str)
{
    std::stringstream ss;

    for (size_t i = 0; i < n; i++)
        ss << str;

    return ss.str();
}

AkVCam::CmdParserCommand::CmdParserCommand()
{
}

AkVCam::CmdParserCommand::CmdParserCommand(const std::string &command,
                                           const std::string &arguments,
                                           const std::string &helpString,
                                           const AkVCam::ProgramOptionsFunc &func,
                                           const std::vector<AkVCam::CmdParserFlags> &flags,
                                           bool advanced):
    command(command),
    arguments(arguments),
    helpString(helpString),
    func(func),
    flags(flags),
    advanced(advanced)
{
}
