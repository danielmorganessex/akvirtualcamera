# akvirtualcamera, virtual camera for Mac and Windows.
# Copyright (C) 2021  Gonzalo Exequiel Pedone
#
# akvirtualcamera is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# akvirtualcamera is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with akvirtualcamera. If not, see <http://www.gnu.org/licenses/>.
#
# Web-Site: http://webcamoid.github.io/

cmake_minimum_required(VERSION 3.5)

project(Manager LANGUAGES CXX OBJCXX) # Enable Objective-C++

include(../commons.cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_STANDARD 20) # Optional: Set ObjC++ standard if needed, typically matches CXX
set(CMAKE_OBJCXX_STANDARD_REQUIRED ON) # Optional

add_executable(Manager
               src/cmdparser.h
               src/main.cpp
               src/cmdparser.cpp
               src/icameracapture.h
               )

if (APPLE OR FAKE_APPLE)
    target_sources(Manager PRIVATE src/macos_cameracapture.mm)
    set_target_properties(Manager PROPERTIES
                          OUTPUT_NAME ${AKVCAM_MANAGER_NAME}
                          RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${BUILDDIR}/${DATAROOTDIR})
elseif (WIN32)
    target_sources(Manager PRIVATE src/windows_cameracapture.cpp)
    set_target_properties(Manager PROPERTIES
                          OUTPUT_NAME ${AKVCAM_MANAGER_NAME}
                          LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${BUILDDIR}/${BINDIR}
                          RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${BUILDDIR}/${BINDIR})
else ()
    # For other platforms, if any specific capture implementation is added later
    set_target_properties(Manager PROPERTIES
                          OUTPUT_NAME ${AKVCAM_MANAGER_NAME}
                          LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${BUILDDIR}/${BINDIR}
                          RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${BUILDDIR}/${BINDIR})
endif ()

add_dependencies(Manager VCamUtils)

if (APPLE OR FAKE_APPLE)
    add_dependencies(Manager
                     VCamIPC_cmio
                     PlatformUtils_cmio)
elseif (WIN32)
    add_dependencies(Manager
                     VCamIPC_dshow
                     PlatformUtils_dshow)
endif ()

target_include_directories(Manager
                           PRIVATE ..)
target_link_libraries(Manager
                      VCamUtils)

if (APPLE OR FAKE_APPLE)
    target_link_libraries(Manager
                          VCamIPC_cmio
                          PlatformUtils_cmio)
elseif (WIN32)
    target_link_libraries(Manager
                          VCamIPC_dshow
                          PlatformUtils_dshow)
endif ()

if (APPLE OR FAKE_APPLE)
    if (APPLE)
        find_library(COREFOUNDATION_FRAMEWORK NAMES CoreFoundation)
        find_library(COREFOUNDATION_FRAMEWORK NAMES CoreFoundation REQUIRED)
        find_library(FOUNDATION_FRAMEWORK NAMES Foundation REQUIRED)
        # For AVFoundation camera capture
        find_library(AVFOUNDATION_FRAMEWORK NAMES AVFoundation REQUIRED)
        find_library(COREMEDIA_FRAMEWORK NAMES CoreMedia REQUIRED) # Often needed with AVFoundation
        find_library(COREVIDEO_FRAMEWORK NAMES CoreVideo REQUIRED) # For CVImageBufferRef etc.
        # CoreMediaIO is likely for the virtual camera plugin side, not manager capture.
        # find_library(COREMEDIAIO_FRAMEWORK NAMES CoreMediaIO)
        find_library(IOKIT_FRAMEWORK NAMES IOKit) # May not be directly needed by manager capture

        set(EXTRA_LIBS
            ${COREFOUNDATION_FRAMEWORK}
            ${FOUNDATION_FRAMEWORK}
            ${AVFOUNDATION_FRAMEWORK}
            ${COREMEDIA_FRAMEWORK}
            ${COREVIDEO_FRAMEWORK}
            # ${COREMEDIAIO_FRAMEWORK}
            ${IOKIT_FRAMEWORK}
        )
        target_link_libraries(Manager PRIVATE ${EXTRA_LIBS})
    endif ()
elseif (WIN32)
    # For Windows Media Foundation, you'd need libraries like:
    # mfplat.lib, mfreadwrite.lib, mfuuid.lib, Shlwapi.lib, ole32.lib, propsys.lib
    # For DirectShow, you'd need:
    # strmiids.lib, ole32.lib, oleaut32.lib, user32.lib, propsys.lib (for KsPropertySet)
    # For Media Foundation (added for list_physical_cameras_impl)
    target_link_libraries(Manager
                          PRIVATE # Use PRIVATE if these are impl details
                          advapi32
                          gdi32
                          ole32
                          oleaut32
                          shell32
                          strmiids # DShow, might still be needed by other VCamUtils parts
                          uuid     # DShow
                          # Media Foundation libs
                          mfplat
                          mfreadwrite
                          mfuuid
                          shlwapi  # For StrRetToBuf used in MF example, might not be strictly needed by MF itself always
                          propsys  # For IPropertyStore, also sometimes used with MF/DShow
                          )
endif ()

if (APPLE OR FAKE_APPLE)
    install(TARGETS Manager DESTINATION ${DATAROOTDIR})
elseif (WIN32)
    install(TARGETS Manager DESTINATION ${BINDIR})
endif ()
