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

#ifndef CMDPARSER_H
#define CMDPARSER_H

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace AkVCam {
    class CmdParserPrivate;
    using StringVector = std::vector<std::string>;
    using StringMap = std::map<std::string, std::string>;
    using ProgramOptionsFunc = std::function<int (const StringMap &flags,
                                                  const StringVector &args)>;

    class CmdParser
    {
        public:
            CmdParser();
            ~CmdParser();
            int parse(int argc, char **argv);
            void setDefaultFuntion(const ProgramOptionsFunc &func);
            void addCommand(const std::string &command,
                            const std::string &arguments,
                            const std::string &helpString,
                            const ProgramOptionsFunc &func,
                            bool advanced=false);
            void addFlags(const std::string &command,
                          const StringVector &flags,
                          const std::string &value,
                          const std::string &helpString);
            void addFlags(const std::string &command,
                          const StringVector &flags,
                          const std::string &helpString);

        private:
            CmdParserPrivate *d;
    };
}

#endif // CMDPARSER_H

// Helper function to be implemented for listing physical cameras
// This is a placeholder and will need platform-specific implementation details.
namespace AkVCam {
    struct PhysicalCamera {
        std::string id;
        std::string name;
        // Other relevant details like model, manufacturer, or path if available
    };
    std::vector<PhysicalCamera> list_physical_cameras_impl();

    // Structure to hold information about a webcam split configuration
    struct WebcamSplitConfig {
        std::string physical_camera_id;
        std::string physical_camera_name; // For convenience
        int num_splits;
        std::string base_name_prefix;
        std::vector<std::string> virtual_camera_ids;
        bool is_active; // Is it currently streaming?
    };
}
