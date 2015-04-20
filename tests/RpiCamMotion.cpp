#include <tools/AsyncTools.h>
#include <tools/Environment.h>

#include <rpi_cam/Camera.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace rpi_cam;
using namespace tools;

////////
// Types
////////

namespace {
    struct DoIt
        : StandardManualRequest< DoIt >
    {
        DoIt( VC0703 * );
        ~DoIt( void );

        // Request
        void start( void );

        // local methods
        void doneReset( Error * );
        void doneVer( Error * );
        void doneImageRes( Error * );
        void doneChangeRes( Error * );
        void doneSetMotionDetect( Error * );
        void doneTestMotionDetect( Error * );

        void loopStart( Error * );
        void detectResults( Error * );
        void gotStopDetecting( Error * );
        void tookImage( Error * );
        void gotImageSize( Error * );
        void gotImage( Error * );

        VC0703 * svc_;
        AutoDispose< Request > inner_;
        StringId ver_;
        ImageSize imageSize_;
        bool detected_;
        uint32 imageLen_;
        unsigned frameNum_;
        uint8 * buf_;
    };
};  // anonymous namespace

///////////////////////
// Non-member Functions
///////////////////////

int
main( int, char ** )
{
    AutoDispose<> envLifetime;
    Environment * env = NewSimpleEnvironment( envLifetime );
    VC0703 * service = env->get< VC0703 >();
    AutoDispose< Request > doit( new DoIt( service ));
    AutoDispose< Error::Reference > res( runRequestSynchronously( doit ));
}

///////
// DoIt
///////

DoIt::DoIt(
    VC0703 * svc )
    : svc_( svc )
    , frameNum_( 0 )
    , buf_( NULL )
{
}

DoIt::~DoIt( void )
{
    if( !!buf_ ) {
        delete [] buf_;
    }
}

void
DoIt::start( void )
{
    inner_ = svc_->reset();
    call< &DoIt::doneReset >( *inner_ );
}

void
DoIt::doneReset(
    Error * err )
{
    TOOLS_ASSERT( !err );
    inner_ = svc_->version( ver_ );
    call< &DoIt::doneVer >( *inner_ );
}

void
DoIt::doneVer( Error * err )
{
    TOOLS_ASSERT( !err );
    printf( "Camera version string: %s\n", ver_.c_str() );
    inner_ = svc_->imageSize( imageSize_ );
    call< &DoIt::doneImageRes >( *inner_ );
}

void
DoIt::doneImageRes( Error * err )
{
    TOOLS_ASSERT( !err );
    switch( imageSize_ ) {
    case Image640x480:
        printf( "Camera image resolution: 640 x 480\n" );
	break;
    case Image320x240:
        printf( "Camera image resolution: 320 x 240\n" );
	break;
    case Image160x120:
        printf( "Camera image resolution: 160 x 120\n" );
	break;
    default:
        printf( "Unknown camera resolution\n" );
    }
    inner_ = svc_->setImageSize( Image640x480 );
    call< &DoIt::doneChangeRes >( *inner_ );
}

void
DoIt::doneChangeRes( Error * err )
{
    TOOLS_ASSERT( !err );
    inner_ = svc_->setMotionDetect( true );
    call< &DoIt::doneSetMotionDetect >( *inner_ );
}

void
DoIt::doneSetMotionDetect( Error * err )
{
    TOOLS_ASSERT( !err );
    inner_ = svc_->motionDetect( detected_ );
    call< &DoIt::doneTestMotionDetect >( *inner_ );
}

void
DoIt::doneTestMotionDetect( Error * err )
{
    TOOLS_ASSERT( !err );
    TOOLS_ASSERT( detected_ );
    loopStart( NULL );
}

void
DoIt::loopStart( Error * err )
{
    TOOLS_ASSERT( !err );
    inner_ = svc_->motionDetected( detected_ );
    call< &DoIt::detectResults >( *inner_ );
}

void
DoIt::detectResults( Error * err )
{
    TOOLS_ASSERT( !err );
    if( detected_ ) {
        inner_ = svc_->setMotionDetect( false );
	call< &DoIt::gotStopDetecting >( *inner_ );
	return;
    }
    loopStart( NULL );
}

void
DoIt::gotStopDetecting( Error * err )
{
    TOOLS_ASSERT( !err );
    inner_ = svc_->takePicture();
    call< &DoIt::tookImage >( *inner_ );
}

void
DoIt::tookImage( Error * err )
{
    TOOLS_ASSERT( !err );
    inner_ = svc_->frameLength( imageLen_ );
    call< &DoIt::gotImageSize >( *inner_ );
}

void
DoIt::gotImageSize( Error * err )
{
    TOOLS_ASSERT( !err );
    printf( "Fetching image (%d bytes)\n", imageLen_ );
    buf_ = new uint8[ imageLen_ ];
    inner_ = svc_->readPicture( imageLen_, buf_ );
    call< &DoIt::gotImage >( *inner_ );
}

void
DoIt::gotImage( Error * err )
{
    TOOLS_ASSERT( !err );
    char name[100];
    sprintf( name, "frame_%04d.jpg", frameNum_ );
    auto fd = creat( name, 0666 );
    write( fd, buf_, imageLen_ );
    close( fd );
    printf( "Wrote %s\n", name );
    if( ++frameNum_ >= 15 ) {
        inner_ = svc_->setMotionDetect( false );
        callFinish( *inner_ );
	return;
    }
    delete [] buf_;
    inner_ = svc_->resumeVideo();
    call< &DoIt::doneChangeRes >( *inner_ );
}
