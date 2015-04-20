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
        void doneSetComp( Error * );
        void doneCompression( Error * );
        void doneImageRes( Error * );
        void doneChangeRes( Error * );
        void doneTake( Error * );
        void doneImageSize( Error * );
        void doneRead( Error * );

        VC0703 * svc_;
        AutoDispose< Request > inner_;
        StringId ver_;
        uint8 comp_;
        ImageSize imageSize_;
        uint32 imageLen_;
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
    inner_ = svc_->setCompression( 0x0f );
    call< &DoIt::doneSetComp >( *inner_ );
}

void
DoIt::doneSetComp( Error * err )
{
    TOOLS_ASSERT( !err );
    inner_ = svc_->compression( comp_ );
    call< &DoIt::doneCompression >( *inner_ );
}

void
DoIt::doneCompression( Error * err )
{
    TOOLS_ASSERT( !err );
    printf( "Camera compression set to: %x\n", comp_ );
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
    inner_ = svc_->takePicture();
    call< &DoIt::doneTake >( *inner_ );
}

void
DoIt::doneTake( Error * err )
{
    TOOLS_ASSERT( !err );
    printf( "Took image\n" );
    inner_ = svc_->frameLength( imageLen_ );
    call< &DoIt::doneImageSize >( *inner_ );
}

void
DoIt::doneImageSize( Error * err )
{
    TOOLS_ASSERT( !err );
    printf( "Resulting image size: %d bytes\n", imageLen_ );
    buf_ = new uint8[ imageLen_ ];
    inner_ = svc_->readPicture( imageLen_, buf_ );
    call< &DoIt::doneRead >( *inner_ );
}

void
DoIt::doneRead( Error * err )
{
    TOOLS_ASSERT( !err );
    auto fd = creat( "test.jpg", 0666 );
    write( fd, buf_, imageLen_ );
    close( fd );
    finish();
}
