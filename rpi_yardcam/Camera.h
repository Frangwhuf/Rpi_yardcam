#pragma once

#include <tools/Interface.h>
#include <tools/String.h>
#include <tools/Tools.h>

namespace rpi_cam
{
    enum ImageSize : tools::uint8
    {
        Image640x480 = 0x00,
	Image320x240 = 0x11,
        Image160x120 = 0x22,
    };

    struct VC0703 {
        virtual tools::AutoDispose< tools::Request > reset( void ) = 0;
        virtual tools::AutoDispose< tools::Request > version( tools::StringId & ) = 0;
        virtual tools::AutoDispose< tools::Request > tvOut( bool ) = 0;
        virtual tools::AutoDispose< tools::Request > takePicture( void ) = 0;
        virtual tools::AutoDispose< tools::Request > readPicture( tools::uint32, tools::uint8 * ) = 0;
        virtual tools::AutoDispose< tools::Request > resumeVideo( void ) = 0;
        virtual tools::AutoDispose< tools::Request > frameLength( tools::uint32 & ) = 0;
        virtual tools::AutoDispose< tools::Request > downsize( tools::uint8 & ) = 0;
        virtual tools::AutoDispose< tools::Request > setDownsize( tools::uint8 ) = 0;
        virtual tools::AutoDispose< tools::Request > imageSize( ImageSize & ) = 0;
        virtual tools::AutoDispose< tools::Request > setImageSize( ImageSize ) = 0;

        virtual tools::AutoDispose< tools::Request > motionDetect( bool & ) = 0;
        virtual tools::AutoDispose< tools::Request > setMotionDetect( bool ) = 0;
        virtual tools::AutoDispose< tools::Request > motionDetected( bool & ) = 0;

        virtual tools::AutoDispose< tools::Request > compression( tools::uint8 & ) = 0;
        virtual tools::AutoDispose< tools::Request > setCompression( tools::uint8 ) = 0;

        virtual tools::AutoDispose< tools::Request > ptz( tools::uint16 &, tools::uint16 &, tools::uint16 &, tools::uint16 &, tools::uint16 &, tools::uint16 & ) = 0;  // get
        virtual tools::AutoDispose< tools::Request > ptz( tools::uint16, tools::uint16, tools::uint16, tools::uint16 ) = 0;  // set
  
        virtual tools::AutoDispose< tools::Request > osd( tools::uint8, tools::uint8, tools::StringId const & ) = 0;
    };
};  // rpi_cam namespace
