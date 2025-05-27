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

#include <chrono>
#include <ctime>
#include <fstream>
#include <map>
#include <sstream>
#include <thread>

#include "logger.h"
#include "utils.h"

namespace AkVCam
{
    class LoggerPrivate
    {
        public:
            std::string logFile;
            std::string fileName;
            int logLevel {AKVCAM_LOGLEVEL_DEFAULT};
            std::fstream stream;

            static const std::map<int, std::string> &logLevelStrMap()
            {
                static std::map<int, std::string> llsMap {
                    {AKVCAM_LOGLEVEL_DEFAULT  , "default"  },
                    {AKVCAM_LOGLEVEL_EMERGENCY, "emergency"},
                    {AKVCAM_LOGLEVEL_FATAL    , "fatal"    },
                    {AKVCAM_LOGLEVEL_CRITICAL , "critical" },
                    {AKVCAM_LOGLEVEL_ERROR    , "error"    },
                    {AKVCAM_LOGLEVEL_WARNING  , "warning"  },
                    {AKVCAM_LOGLEVEL_NOTICE   , "notice"   },
                    {AKVCAM_LOGLEVEL_INFO     , "info"     },
                    {AKVCAM_LOGLEVEL_DEBUG    , "debug"    },
                };

                return llsMap;
            }
    };

    LoggerPrivate *loggerPrivate()
    {
        static LoggerPrivate logger;

        return &logger;
    }
}

std::string AkVCam::Logger::logFile()
{
    return loggerPrivate()->logFile;
}

void AkVCam::Logger::setLogFile(const std::string &fileName)
{
    loggerPrivate()->logFile = fileName;
    auto index = fileName.rfind('.');

    if (index == fileName.npos) {
        loggerPrivate()->fileName = fileName + "-" + timeStamp();
    } else {
        std::string fname = fileName.substr(0, index);
        std::string extension = fileName.substr(index + 1);
        loggerPrivate()->fileName = fname + "-" + timeStamp() + "." + extension;
    }
}

int AkVCam::Logger::logLevel()
{
    return loggerPrivate()->logLevel;
}

void AkVCam::Logger::setLogLevel(int logLevel)
{
    loggerPrivate()->logLevel = logLevel;
}

std::string AkVCam::Logger::header(int logLevel,
                                   const std::string &file,
                                   int line)
{
    auto now = std::chrono::system_clock::now();
    auto nowMSecs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    auto time = std::chrono::system_clock::to_time_t(now);

#ifdef _WIN32
    struct tm timeInfo;
    localtime_s(&timeInfo, &time);
#else
    auto timeInfo = *std::localtime(&time);
#endif

    std::stringstream ss;
    ss << "[" << std::put_time(&timeInfo, "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << (nowMSecs.count() % 1000)
       << ", " << std::this_thread::get_id()
       << ", " << file << " (" << line << ")] "
       << levelToString(logLevel) << ": ";

    return ss.str();}

std::ostream &AkVCam::Logger::log(int logLevel)
{
    static std::ostream dummy(nullptr);

    if (logLevel > loggerPrivate()->logLevel)
        return dummy;

    if (loggerPrivate()->fileName.empty())
        return std::cerr;

    if (!loggerPrivate()->stream.is_open())
        loggerPrivate()->stream.open(loggerPrivate()->fileName,
                                     std::ios_base::out | std::ios_base::app);

    if (!loggerPrivate()->stream.is_open())
        return std::cerr;

    return loggerPrivate()->stream;
}

int AkVCam::Logger::levelFromString(const std::string &level)
{
    auto &llsMap = LoggerPrivate::logLevelStrMap();

    for (auto it = llsMap.begin(); it != llsMap.end(); it++)
        if (it->second == level)
            return it->first;

    return AKVCAM_LOGLEVEL_DEFAULT;
}

std::string AkVCam::Logger::levelToString(int level)
{
    auto &llsMap = LoggerPrivate::logLevelStrMap();

    for (auto it = llsMap.begin(); it != llsMap.end(); it++)
        if (it->first == level)
            return it->second;

    return {};
}
