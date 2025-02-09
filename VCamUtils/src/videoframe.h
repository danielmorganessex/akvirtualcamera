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

#ifndef VIDEOFRAME_H
#define VIDEOFRAME_H

#include <string>
#include <memory>
#include <vector>

#include "videoframetypes.h"
#include "videoformattypes.h"

namespace AkVCam
{
    class VideoFramePrivate;
    class VideoFormat;
    using VideoData = std::vector<uint8_t>;

    class VideoFrame
    {
        public:
            VideoFrame();
            VideoFrame(const std::string &fileName);
            VideoFrame(const VideoFormat &format);
            VideoFrame(const VideoFrame &other);
            VideoFrame &operator =(const VideoFrame &other);
            bool operator ==(const VideoFrame &other) const;
            operator bool() const;
            ~VideoFrame();

            bool load(const std::string &fileName);
            VideoFormat format() const;
            VideoFormat &format();
            VideoData data() const;
            VideoData &data();
            uint8_t *line(size_t plane, size_t y) const;
            void clear();

            VideoFrame mirror(bool horizontalMirror, bool verticalMirror) const;
            VideoFrame scaled(int width,
                              int height,
                              Scaling mode=ScalingFast,
                              AspectRatio aspectRatio=AspectRatioIgnore) const;
            VideoFrame scaled(size_t maxArea,
                              Scaling mode=ScalingFast,
                              int align=32) const;
            VideoFrame swapRgb(bool swap) const;
            VideoFrame swapRgb() const;
            bool canConvert(FourCC input, FourCC output) const;
            VideoFrame convert(FourCC fourcc) const;
            VideoFrame adjustHsl(int hue, int saturation, int luminance);
            VideoFrame adjustGamma(int gamma);
            VideoFrame adjustContrast(int contrast);
            VideoFrame toGrayScale();
            VideoFrame adjust(int hue,
                              int saturation,
                              int luminance,
                              int gamma,
                              int contrast,
                              bool gray);

        private:
            VideoFramePrivate *d;
    };
}

#endif // VIDEOFRAME_H
