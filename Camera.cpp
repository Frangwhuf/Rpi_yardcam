#include <rpi_cam/Camera.h>

#include <tools/Environment.h>
#include <tools/AsyncTools.h>
#include <tools/Threading.h>
#include <tools/Timing.h>

#ifdef WINDOWS_PLATFORM
#  pragma warning( disable : 4061 )
#  pragma warning( disable : 4191 )
#  pragma warning( disable : 4242 )
#  pragma warning( disable : 4365 )
#  pragma warning( disable : 4371 )
#  pragma warning( disable : 4571 )
#  pragma warning( disable : 4619 )
#endif // WINDOWS_PLATFORM
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#ifdef WINDOWS_PLATFORM
#  pragma warning( default : 4061 )
#  pragma warning( default : 4191 )
#  pragma warning( default : 4242 )
#  pragma warning( default : 4365 )
#  pragma warning( default : 4371 )
#  pragma warning( default : 4571 )
#  pragma warning( default : 4619 )
#endif // WINDOWS_PLATFORM
#include <boost/bind.hpp>

#include <queue>

using namespace rpi_cam;
using namespace tools;

#ifdef WINDOWS_PLATFORM
#  if (_INTEGRAL_MAX_BITS == 64)
#    define LONG_SIZE_T
#  endif  // 64-bit
#endif // WINDOWS_PLATFORM
#ifdef UNIX_PLATFORM
#  if defined(__i386__) || defined(__x86_64__)
#    if (__SIZEOF_POINTER__ == 8)
#      define LONG_SIZE_T
#    endif // 64-bit
#  endif // intel arch
#endif // UNIX_PLATFORM

////////
// Types
////////

namespace {
    enum WorkState {
        WorkStateSend,
        WorkStateReceive,
        WorkStateValidate,
	WorkStateDone,
    };

    struct CamBuffer
    {
        CamBuffer( void );
        ~CamBuffer( void );

        void reserve( size_t );
        uint8 * begin( void ) const;
        uint8 * end( void ) const;
        bool empty( void ) const;
        void clear( void );
        size_t size( void ) const;
        void print( void ) const;

        size_t length_, capacity_;
        uint8 * data_;
    };

    struct WorkItem
        : Disposable
    {
        typedef std::pair<size_t, size_t> ResponseBound;

        virtual void render( CamBuffer & ) = 0;
        virtual ResponseBound responseSize( void ) = 0;
        virtual WorkState validate( CamBuffer & ) = 0;  // return the next step for this item
        virtual StringId name( void ) = 0;

        Thunk thunk_;
    };

    struct Vc0703Impl
        : VC0703
        , StandardService< Vc0703Impl, boost::mpl::list< VC0703 >::type >
        , Notifiable< Vc0703Impl >
    {
        typedef std::queue< AutoDispose< WorkItem >/*, AllocatorAffinity< AutoDispose< WorkItem >>*/> WorkQueue;

        Vc0703Impl( Environment & );

        // Service
        AutoDispose< Request > serviceStart( void );
        AutoDispose< Request > serviceStop( void );

        // VC0703
        AutoDispose< Request > reset( void );
        AutoDispose< Request > version( StringId & );
        AutoDispose< Request > tvOut( bool );
        AutoDispose< Request > takePicture( void );
        AutoDispose< Request > readPicture( uint32, uint8 * );
        AutoDispose< Request > resumeVideo( void );
        AutoDispose< Request > frameLength( uint32 & );
        AutoDispose< Request > downsize( uint8 & );
        AutoDispose< Request > setDownsize( uint8 );
        AutoDispose< Request > imageSize( ImageSize & );
        AutoDispose< Request > setImageSize( ImageSize );
        AutoDispose< Request > motionDetect( bool & );
        AutoDispose< Request > setMotionDetect( bool );
        AutoDispose< Request > motionDetected( bool & );
        AutoDispose< Request > compression( uint8 & );
        AutoDispose< Request > setCompression( uint8 );
        AutoDispose< Request > ptz( uint16 &, uint16 &, uint16 &, uint16 &, uint16 &, uint16 & );  // get
        AutoDispose< Request > ptz( uint16, uint16, uint16, uint16 );  // set
        AutoDispose< Request > osd( uint8, uint8, StringId const & );

        // local methods
        void ioEntry( void );
        void enqueue( AutoDispose< WorkItem > & );
        void workEntry( void );
        void ioSendCompleted( boost::system::error_code const & );
        void ioRecCompleted( boost::system::error_code const &, size_t );
        void timerFired( boost::system::error_code const & );

        boost::asio::io_service ioService_;
        boost::asio::serial_port serialPort_;
        boost::asio::deadline_timer timer_;
        Threading * threading_;
        ThreadScheduler * scheduler_;
        Timing * timing_;
        Thunk startDone_;
        AutoDispose< Thread > ioThread_;
        WorkQueue work_;
        AutoDispose< ConditionVar > workCond_;
        AutoDispose< Monitor > workCondLock_;
        WorkState workState_;
        AutoDispose< Thread > workThread_;
        AutoDispose< Event > runEvent_;
        bool ioInFlight_;
        CamBuffer buffer_;
        bool volatile stopping_;
        size_t readMin_;
        size_t readMax_;
        size_t readCurrent_;
    };

    struct Vc0703Start
        : StandardManualRequest< Vc0703Start >
        , Notifiable< Vc0703Start >
    {
        Vc0703Start( Vc0703Impl & );

        // Request
        void start( void );

        // local methods
        void done( void );

        Vc0703Impl & parent_;
        AutoDispose< Request > inner_;
      };

    struct WorkReq
        : StandardManualRequest< WorkReq >
        , Notifiable< WorkReq >
    {
        WorkReq( AutoDispose< WorkItem > &&, Vc0703Impl * );

        // Request
        void start( void );

        // local methods
        void done( void );

        AutoDispose< WorkItem > item_;
        Vc0703Impl * svc_;
    };

    struct FlushWork
        : StandardDisposable< FlushWork, WorkItem >
    {
        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );
    };

    struct ResetWork
        : StandardDisposable< ResetWork, WorkItem >
    {
        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );
    };

    struct GetVerWork
        : StandardDisposable< GetVerWork, WorkItem >
    {
        GetVerWork( StringId & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        StringId & result_;
    };

    struct TvOutWork
        : StandardDisposable< TvOutWork, WorkItem >
    {
        TvOutWork( bool );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        bool state_;
    };

    struct TakePictureWork
        : StandardDisposable< TakePictureWork, WorkItem >
    {
        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );
    };

    struct ReadPictureWork
        : StandardDisposable< ReadPictureWork, WorkItem >
    {
        ReadPictureWork( uint32, uint8 * );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint32 size_;
        uint8 * result_;
    };

    struct ResumeVideoWork
        : StandardDisposable< ResumeVideoWork, WorkItem >
    {
        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );
    };

    struct FrameLenWork
        : StandardDisposable< FrameLenWork, WorkItem >
    {
        FrameLenWork( uint32 & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint32 & result_;
    };

    struct DownsizeWork
        : StandardDisposable< DownsizeWork, WorkItem >
    {
        DownsizeWork( uint8 & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint8 & result_;
    };

    struct SetDownsizeWork
        : StandardDisposable< SetDownsizeWork, WorkItem >
    {
        SetDownsizeWork( uint8 );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint8 status_;
    };

    struct ImageSizeWork
        : StandardDisposable< ImageSizeWork, WorkItem >
    {
        ImageSizeWork( ImageSize & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        ImageSize & result_;
    };

    struct SetImageSizeWork
        : StandardDisposable< SetImageSizeWork, WorkItem >
    {
        SetImageSizeWork( ImageSize );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        ImageSize size_;
    };

    struct MotionDetectWork
        : StandardDisposable< MotionDetectWork, WorkItem >
    {
        MotionDetectWork( bool & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        bool & result_;
    };

    struct SetMotionDetectWork
        : StandardDisposable< SetMotionDetectWork, WorkItem >
    {
        SetMotionDetectWork( bool );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        bool state_;
        bool firstPass_;
    };

    struct MotionDetectedWork
        : StandardDisposable< MotionDetectedWork, WorkItem >
    {
        MotionDetectedWork( bool & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        bool & result_;
    };

    struct CompressionWork
        : StandardDisposable< CompressionWork, WorkItem >
    {
        CompressionWork( uint8 & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint8 & result_;
    };

    struct SetCompressionWork
        : StandardDisposable< SetCompressionWork, WorkItem >
    {
        SetCompressionWork( uint8 );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint8 state_;
    };

    struct Ptz1Work
        : StandardDisposable< Ptz1Work, WorkItem >
    {
        Ptz1Work( uint16 &, uint16 &, uint16 &, uint16 &, uint16 &, uint16 & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint16 & w_;
        uint16 & h_;
        uint16 & wz_;
        uint16 & hz_;
        uint16 & pan_;
        uint16 & tilt_;
    };

    struct Ptz2Work
        : StandardDisposable< Ptz2Work, WorkItem >
    {
        Ptz2Work( uint16, uint16, uint16, uint16 );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint16 wz_;
        uint16 hz_;
        uint16 pan_;
        uint16 tilt_;
    };

    struct OsdWork
        : StandardDisposable< OsdWork, WorkItem >
    {
        OsdWork( uint8, uint8, StringId const & );

        void render( CamBuffer & );
        ResponseBound responseSize( void );
        WorkState validate( CamBuffer & );
        StringId name( void );

        uint8 x_;
        uint8 y_;
        StringId text_;
    };

    static RegisterEnvironment< VC0703, Vc0703Impl > regRpiCam;
}; // anonymous namespace

///////////////////////
// Non-member Functions
///////////////////////


////////////
// CamBuffer
////////////

CamBuffer::CamBuffer( void )
    : length_( 0 )
    , capacity_( 0 )
    , data_( NULL )
{
}

CamBuffer::~CamBuffer( void )
{
    if( !!data_ ) {
        delete [] data_;
    }
}

void
CamBuffer::reserve(
    size_t len )
{
    if( len > capacity_ ) {
        delete [] data_;
        data_ = new uint8[ len ];
        capacity_ = len;
    }
    length_ = 0;
    memset( data_, 0, capacity_ );
}

uint8 *
CamBuffer::begin( void ) const
{
    return data_;
}

uint8 *
CamBuffer::end( void ) const
{
    return data_ + length_;
}

bool
CamBuffer::empty( void ) const
{
    return ( length_ == 0 );
}

void
CamBuffer::clear( void )
{
    length_ = 0;
}

size_t
CamBuffer::size( void ) const
{
    return length_;
}

void
CamBuffer::print( void ) const
{
    printf( "Debug buffer print (%d / %d):\n", length_, capacity_ );
    if( length_ > 0 ) {
        std::for_each( data_, data_ + length_, []( uint8 b )->void {
            printf( "0x%x, ", b );
        });
        printf( "\n" );
    }
}

/////////////
// Vc0703Impl
/////////////

Vc0703Impl::Vc0703Impl( Environment & env )
    : serialPort_( ioService_, "/dev/ttyAMA0" )
    , timer_( ioService_, boost::posix_time::milliseconds( 200 ))
    , threading_( env.get< Threading >() )
    , scheduler_( env.get< TaskScheduler >() )
    , timing_( env.get< Timing >() )
    , workCond_( conditionVarNew() )
    , workCondLock_( workCond_->monitorNew() )
    , workState_( WorkStateSend )
    , runEvent_( eventNew() )
    , ioInFlight_( false )
    , stopping_( false )
{
}

AutoDispose< Request >
Vc0703Impl::serviceStart( void )
{
    if( !serialPort_.is_open() ) {
        printf("Could not open serial port at /dev/ttyAMA0\n");
        TOOLS_ASSERTR(!"Could not open serial port");
    }
    return scheduler_->bind( new Vc0703Start( *this ));
}

AutoDispose< Request >
Vc0703Impl::serviceStop( void )
{
    serialPort_.close();
    stopping_ = true;
    workCond_->signal();  // wake this up to notice the port is closed
    return ioThread_->wait();
}

AutoDispose< Request >
Vc0703Impl::reset( void )
{
    return scheduler_->bind( new WorkReq( new ResetWork(), this ));
}

AutoDispose< Request >
Vc0703Impl::version(
    StringId & result )
{
    return scheduler_->bind( new WorkReq( new GetVerWork( result ), this ));
}

AutoDispose< Request >
Vc0703Impl::tvOut( bool state )
{
    return scheduler_->bind( new WorkReq( new TvOutWork( state ), this ));
}

AutoDispose< Request >
Vc0703Impl::takePicture( void )
{
    return scheduler_->bind( new WorkReq( new TakePictureWork(), this ));
}

AutoDispose< Request >
Vc0703Impl::readPicture( uint32 count, uint8 * buf )
{
    return scheduler_->bind( new WorkReq( new ReadPictureWork( count, buf ), this ));
}

AutoDispose< Request >
Vc0703Impl::resumeVideo( void )
{
    return scheduler_->bind( new WorkReq( new ResumeVideoWork(), this ));
}

AutoDispose< Request >
Vc0703Impl::frameLength( uint32 & len )
{
    return scheduler_->bind( new WorkReq( new FrameLenWork( len ), this ));
}

AutoDispose< Request >
Vc0703Impl::downsize( uint8 & res )
{
    return scheduler_->bind( new WorkReq( new DownsizeWork( res ), this ));
}

AutoDispose< Request >
Vc0703Impl::setDownsize( uint8 state )
{
    return scheduler_->bind( new WorkReq( new SetDownsizeWork( state ), this ));
}

AutoDispose< Request >
Vc0703Impl::imageSize( ImageSize & size )
{
    return scheduler_->bind( new WorkReq( new ImageSizeWork( size ), this ));
}

AutoDispose< Request >
Vc0703Impl::setImageSize( ImageSize size )
{
    return scheduler_->bind( new WorkReq( new SetImageSizeWork( size ), this ));
}

AutoDispose< Request >
Vc0703Impl::motionDetect( bool & res )
{
    return scheduler_->bind( new WorkReq( new MotionDetectWork( res ), this ));
}

AutoDispose< Request >
Vc0703Impl::setMotionDetect( bool state )
{
    return scheduler_->bind( new WorkReq( new SetMotionDetectWork( state ), this ));
}

AutoDispose< Request >
Vc0703Impl::motionDetected( bool & res )
{
    return scheduler_->bind( new WorkReq( new MotionDetectedWork( res ), this ));
}

AutoDispose< Request >
Vc0703Impl::compression( uint8 & res )
{
    return scheduler_->bind( new WorkReq( new CompressionWork( res ), this ));
}

AutoDispose< Request >
Vc0703Impl::setCompression( uint8 state )
{
    return scheduler_->bind( new WorkReq( new SetCompressionWork( state ), this ));
}

AutoDispose< Request >
Vc0703Impl::ptz( uint16 & w, uint16 & h, uint16 & wz, uint16 & hz, uint16 & pan, uint16 & tilt )
{
    return scheduler_->bind( new WorkReq( new Ptz1Work( w, h, wz, hz, pan, tilt ), this ));
}

AutoDispose< Request >
Vc0703Impl::ptz( uint16 wz, uint16 hz, uint16 pan, uint16 tilt )
{
    return scheduler_->bind( new WorkReq( new Ptz2Work( wz, hz, pan, tilt ), this ));
}

AutoDispose< Request >
Vc0703Impl::osd( uint8 x, uint8 y, StringId const & text )
{
    return scheduler_->bind( new WorkReq( new OsdWork( x, y, text ), this ));
}

void
Vc0703Impl::ioEntry( void )
{
    if( !!startDone_ ) {
        startDone_.fire();
    }
    while( !stopping_ ) {
        runEvent_->wait();
        // printf( "calling run().\n" );
        ioService_.run();
        ioService_.reset();
    }
}

void
Vc0703Impl::enqueue( AutoDispose< WorkItem > & item )
{
    {
        AutoDispose<> l( workCondLock_->enter() );
        work_.push( std::move( item ));
    }
    // wake up the work thread in case it's waiting for something
    workCond_->signal();
}

void
Vc0703Impl::workEntry( void )
{
    AutoDispose<> l( workCondLock_->enter() );
    while( serialPort_.is_open() ) {
        if( work_.empty() || ioInFlight_ ) {
            workCond_->wait();  // wait for more work, or I/O to complete
            continue;
        }
        // pull the next thing from work and 
        AutoDispose< WorkItem > & item = work_.front();
        switch( workState_ ) {
        case WorkStateSend:
                item->render( buffer_ );
                workState_ = WorkStateReceive;
                if( buffer_.empty() ) {
                    continue;
                }
                ioInFlight_ = true;
                {
                    // printf( "sending %d bytes for %s.\n", buffer_.size(), item->name().c_str() );
                    // serialPort_.cancel();
                    // timer_.cancel();
                    auto sent = boost::asio::write( serialPort_, boost::asio::buffer( buffer_.begin(), buffer_.size() ));
                    TOOLS_ASSERT( sent == buffer_.size() );
                }
                ioInFlight_ = false;
                break;
        case WorkStateReceive:
                workState_ = WorkStateValidate;
                ioInFlight_ = true;
                {
                    auto expect = item->responseSize();
                    readMin_ = expect.first;
                    readMax_ = expect.second;
                    if( readMax_ == 0 ) {
                        buffer_.clear();
                        ioInFlight_ = false;
                        continue;
                    }
                    buffer_.reserve( expect.second );
                    readCurrent_ = 0;
                    // printf( "Waiting for responce (%d-%d bytes) for %s.\n", expect.first, expect.second, item->name().c_str() );
                    // serialPort_.cancel();
                    // timer_.cancel();
                    if( expect.first != expect.second ) {
                        timer_.expires_from_now( boost::posix_time::milliseconds( 200 ));
                        timer_.async_wait( boost::bind( &Vc0703Impl::timerFired, this, boost::asio::placeholders::error ));
                    }
                    boost::asio::async_read( serialPort_, boost::asio::buffer( buffer_.begin(), 1 ),
                        boost::bind( &Vc0703Impl::ioRecCompleted, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ));
                    runEvent_->post();
                }
                break;
        case WorkStateValidate:
                ioInFlight_ = true;
                // printf( "validating %s.\n", item->name().c_str() );
                workState_ = item->validate( buffer_ );
                ioInFlight_ = false;
                break;
        case WorkStateDone:
                work_.pop();
                workState_ = WorkStateSend;
                break;
        default:
                TOOLS_ASSERT( !"Unknown work staet" );
                workState_ = WorkStateSend;
        }
    }
}

void
Vc0703Impl::ioSendCompleted(
    boost::system::error_code const & )
{
    // TODO: do something about errors
    {
        AutoDispose<> l( workCondLock_->enter() );
    }
    ioInFlight_ = false;
    workCond_->signal();
}

void
Vc0703Impl::ioRecCompleted(
    boost::system::error_code const & err,
    size_t len )
{
    timer_.cancel();
    if( !err || len > 0 ) {
        if(( readMin_ > 0 ) && ( readCurrent_ == 0 )) {
            size_t issue = 1;
            if( buffer_.begin()[ 0 ] == 0x76 ) {
                // got the message start byte, read the rest
                ++readCurrent_;
                if( readMin_ == readMax_ ) {
                    issue = readMax_ - 1;  // fixed size, read all of the rest
                }
            }  // otherwise keep trying
            if( issue == 1 ) {
                // if only getting the next byte, engage timeout
                timer_.expires_from_now( boost::posix_time::milliseconds( 200 ));
                timer_.async_wait( boost::bind( &Vc0703Impl::timerFired, this, boost::asio::placeholders::error ));
            }
            boost::asio::async_read( serialPort_, boost::asio::buffer( buffer_.begin() + readCurrent_, issue ),
                boost::bind( &Vc0703Impl::ioRecCompleted, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ));
            runEvent_->post();
            return;
        }
        TOOLS_ASSERT( ( readCurrent_ + len ) <= buffer_.capacity_ );
        buffer_.length_ = readCurrent_ + len;
        if( readMin_ != readMax_ ) {
            readCurrent_ += len;
            if( readCurrent_ != readMax_ ) {
                // serialPort_.cancel();
                // timer_.cancel();
                timer_.expires_from_now( boost::posix_time::milliseconds( 200 ));
                timer_.async_wait( boost::bind( &Vc0703Impl::timerFired, this, boost::asio::placeholders::error ));
                boost::asio::async_read( serialPort_, boost::asio::buffer( buffer_.begin() + readCurrent_, 1 ),
                    boost::bind( &Vc0703Impl::ioRecCompleted, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ));
                runEvent_->post();
                return;
            }
        }
    }
    // serialPort_.cancel();
    // timer_.cancel();
    // printf( "finished read for %s (got %d bytes).\n", work_.front()->name().c_str(), buffer_.size() );
    {
        AutoDispose<> l( workCondLock_->enter() );
    }
    if( buffer_.length_ < readMin_ ) {
      printf( "read ran short for %s.  Got %d, wanted %d\n", work_.front()->name().c_str(), buffer_.length_, readMin_ );
      TOOLS_ASSERT( buffer_.length_ >= readMin_ );
    }
    ioInFlight_ = false;
    workCond_->signal();
}

void
Vc0703Impl::timerFired(
    boost::system::error_code const & err )
{
    if( !err ) {
        // printf( "canceled read for %s.\n", work_.front()->name().c_str() );
        serialPort_.cancel();
    }
}

//////////////
// Vc0703Start
//////////////

Vc0703Start::Vc0703Start(
    Vc0703Impl & p )
    : parent_( p )
{
}

void
Vc0703Start::start( void )
{
    // TODO: make a regular locl to use in places like this
    AutoDispose<> l( parent_.workCondLock_->enter() );
    parent_.workThread_ = std::move( parent_.threading_->fork( "Work thread", parent_.toThunk< &Vc0703Impl::workEntry >() ));
    boost::asio::serial_port_base::baud_rate baud_option( 38400 );
    parent_.serialPort_.set_option( baud_option );
    parent_.startDone_ = toThunk< &Vc0703Start::done >();
    parent_.ioThread_ = std::move( parent_.threading_->fork( "ASIO thread", parent_.toThunk< &Vc0703Impl::ioEntry >() ));
}

void
Vc0703Start::done( void )
{
    // inner_.reset( parent_.scheduler_->bind( new WorkReq( new FlushWork(), &parent_ )));
    // callFinish( *inner_ );
    finish();
}

//////////
// WorkReq
//////////

WorkReq::WorkReq(
    AutoDispose< WorkItem > && item,
    Vc0703Impl * svc )
    : item_( std::move( item ))
    , svc_( svc )
{
}

void
WorkReq::start( void )
{
    item_->thunk_ = toThunk< &WorkReq::done >();
    {
        AutoDispose<> l( svc_->workCondLock_->enter() );
        svc_->work_.push( std::move( item_ ));
    }
    svc_->workCond_->signal();
}

void
WorkReq::done( void )
{
    finish();
}

////////////
// FlushWork
////////////

void
FlushWork::render( CamBuffer & buffer )
{
  buffer.length_ = 0;
}

WorkItem::ResponseBound
FlushWork::responseSize( void )
{
  return ResponseBound( 0, 1000 );
}

WorkState
FlushWork::validate( CamBuffer & )
{
    return WorkStateDone;
}

StringId
FlushWork::name( void )
{
    return nameOf< FlushWork >();
}

////////////
// ResetWork
////////////

void
ResetWork::render(
    CamBuffer & buffer )
{
    buffer.reserve( 4 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x26;
    data[ 3 ] = 0x00;
    buffer.length_ = 4;
}

WorkItem::ResponseBound
ResetWork::responseSize( void )
{
    return ResponseBound(5, 205);
}

WorkState
ResetWork::validate(
    CamBuffer & buffer )
{
    if( buffer.size() < 5 ) {
#ifndef LONG_SIZE_T
        printf( "Reset - wrong sized response buffer (%d / 5+)\n", buffer.size() );
#else // size_T size
        printf( "Reset - wrong sized response buffer (%ld / 5+)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x26 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "Reset - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
ResetWork::name( void )
{
  return nameOf< ResetWork >();
}

/////////////
// GetVerWork
/////////////

GetVerWork::GetVerWork(
    StringId & result )
    : result_( result )
{
}

void
GetVerWork::render(
    CamBuffer & buffer )
{
    buffer.reserve( 4 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x11;
    data[ 3 ] = 0x00;
    buffer.length_ = 4;
}

WorkItem::ResponseBound
GetVerWork::responseSize( void )
{
    return ResponseBound(6, 200);
}

WorkState
GetVerWork::validate(
    CamBuffer & buffer )
{
    if( buffer.size() < 6 ) {
#ifndef LONG_SIZE_T
        printf( "GetVersion - wrong sized response buffer (%d / 6+)\n", buffer.size() );
#else // size_t size
        printf( "GetVersion - wrong sized response buffer (%ld / 6+)\n", buffer.size() );
#endif // size_t size
        buffer.print();
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x11 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "GetVersion - responce does not verify\n" );
        buffer.print();
        abort();
    }
    int len = data[ 4 ];
    if( buffer.size() < ( len + 5 )) {
#ifndef LONG_SIZE_T
        printf( "GetVersion - wrong sized response buffer (%d / %d + 5)\n", buffer.size(), len );
#else // size_t size
        printf( "GetVersion - wrong sized response buffer (%ld / %d + 5)\n", buffer.size(), len );
#endif // size_t size
        abort();
    }
    result_ = StringId( reinterpret_cast< char * >( buffer.begin() + 5 ), len );
    thunk_.fire();
    return WorkStateDone;
}

StringId
GetVerWork::name( void )
{
  return nameOf< GetVerWork >();
}

////////////
// TvOutWork
////////////

TvOutWork::TvOutWork( bool state )
    : state_( state )
{
}

void
TvOutWork::render( CamBuffer & buffer )
{
    buffer.reserve( 5 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x44;
    data[ 3 ] = 0x01;
    data[ 4 ] = static_cast< uint8 >( state_ ? 0x01 : 0x00 );
    buffer.length_ = 5;
}

WorkItem::ResponseBound
TvOutWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
TvOutWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "TvOut - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "TvOut - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        buffer.print();
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x44 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "TvOut - responce does not verify\n" );
        buffer.print();
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
TvOutWork::name( void )
{
    return nameOf< TvOutWork >();
}

//////////////////
// TakePictureWork
//////////////////

void
TakePictureWork::render( CamBuffer & buffer )
{
    buffer.reserve( 5 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x36;  // fbuf ctrl
    data[ 3 ] = 0x01;
    data[ 4 ] = 0x00;  // stop current frame
    buffer.length_ = 5;
}

WorkItem::ResponseBound
TakePictureWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
TakePictureWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "TakePicture - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "TakePicture - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x36 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "TakePicture - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
TakePictureWork::name( void )
{
    return nameOf< TakePictureWork >();
}

//////////////////
// ReadPictureWork
//////////////////

ReadPictureWork::ReadPictureWork( uint32 size, uint8 * res )
    : size_( size )
    , result_( res )
{
}

void
ReadPictureWork::render( CamBuffer & buffer )
{
    buffer.reserve( 16 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x32;
    data[ 3 ] = 0x0c;
    data[ 4 ] = 0x00;
    data[ 5 ] = 0x0a;
    data[ 6 ] = 0x00;  // offset into the buffer
    data[ 7 ] = 0x00;
    data[ 8 ] = 0x00;
    data[ 9 ] = 0x00;
    data[ 10 ] = static_cast< uint8 >( ( size_ >> 24 ) & 0xff );  // length to send back
    data[ 11 ] = static_cast< uint8 >( ( size_ >> 16 ) & 0xff );
    data[ 12 ] = static_cast< uint8 >( ( size_ >> 8 ) & 0xff );
    data[ 13 ] = static_cast< uint8 >( size_ & 0xff );
    data[ 14 ] = 0x01;  // lead-out
    data[ 15 ] = 0x00;
    buffer.length_ = 16;
}

WorkItem::ResponseBound
ReadPictureWork::responseSize( void )
{
    return ResponseBound( 10, size_ + 10 );  // 5 byte header on each end
}

WorkState
ReadPictureWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != ( size_ + 10 ) ) {
#ifndef LONG_SIZE_T
        printf( "ReadPicture - wrong sized response buffer (%d / %d + 10)\n", buffer.size(), size_ );
#else // size_t size
        printf( "ReadPicture - wrong sized response buffer (%ld / %d + 10)\n", buffer.size(), size_ );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x32 ) && ( data[ 3 ] == 0x00 ) &&
        ( data[ size_ + 5 ] == 0x76 ) && ( data[ size_ + 6 ] == 0x00 ) && ( data[ size_ + 7 ] == 0x32 ) &&
        ( data[ size_ + 8 ] == 0x00 );
    if( !ok ) {
        printf( "ReadPicture - responce does not verify\n" );
        abort();
    }
    memcpy( result_, data + 5, size_ );
    thunk_.fire();
    return WorkStateDone;
}

StringId
ReadPictureWork::name( void )
{
    return nameOf< ReadPictureWork >();
}

//////////////////
// ResumeVideoWork
//////////////////

void
ResumeVideoWork::render( CamBuffer & buffer )
{
    buffer.reserve( 5 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x36;  // fbuf ctrl
    data[ 3 ] = 0x01;
    data[ 4 ] = 0x03;  // resume frame
    buffer.length_ = 5;
}

WorkItem::ResponseBound
ResumeVideoWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
ResumeVideoWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "ResumeVideo - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "ResumeVideo - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x36 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "ResumeVideo - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
ResumeVideoWork::name( void )
{
    return nameOf< ResumeVideoWork >();
}

///////////////
// FrameLenWork
///////////////

FrameLenWork::FrameLenWork( uint32 & res )
    : result_( res )
{
}

void
FrameLenWork::render( CamBuffer & buffer )
{
    buffer.reserve( 5 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x34;
    data[ 3 ] = 0x01;
    data[ 4 ] = 0x00;
    buffer.length_ = 5;
}

WorkItem::ResponseBound
FrameLenWork::responseSize( void )
{
    return ResponseBound( 9, 9 );
}

WorkState
FrameLenWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 9 ) {
#ifndef LONG_SIZE_T
        printf( "FrameLength - wrong sized response buffer (%d / 9)\n", buffer.size() );
#else // size_t size
        printf( "FrameLength - wrong sized response buffer (%ld / 9)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x34 ) && ( data[ 3 ] == 0x00 ) && ( data[ 4 ] == 0x04 );
    if( !ok ) {
        printf( "FrameLength - responce does not verify\n" );
        abort();
    }
    result_ = ( static_cast< uint32 >( data[ 5 ] ) << 24 ) |
        ( static_cast< uint32 >( data[ 6 ] ) << 16 ) |
        ( static_cast< uint32 >( data[ 7 ] ) << 8 ) |
        static_cast< uint32 >( data[ 8 ] );
    thunk_.fire();
    return WorkStateDone;
}

StringId
FrameLenWork::name( void )
{
    return nameOf< FrameLenWork >();
}

///////////////
// DownsizeWork
///////////////

DownsizeWork::DownsizeWork( uint8 & result )
    : result_( result )
{
}

void
DownsizeWork::render( CamBuffer & buffer )
{
    buffer.reserve( 4 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x55;
    data[ 3 ] = 0x00;
    buffer.length_ = 4;
}

WorkItem::ResponseBound
DownsizeWork::responseSize( void )
{
    return ResponseBound( 6, 6 );
}

WorkState
DownsizeWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 6 ) {
#ifndef LONG_SIZE_T
        printf( "DownsizeWork - wrong sized response buffer (%d / 6)\n", buffer.size() );
#else // size_t size
        printf( "DownsizeWork - wrong sized response buffer (%ld / 6)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x55 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "DownsizeWork - responce does not verify\n" );
        abort();
    }
    result_ = data[ 5 ];
    thunk_.fire();
    return WorkStateDone;
}

StringId
DownsizeWork::name( void )
{
    return nameOf< DownsizeWork >();
}

//////////////////
// SetDownsizeWork
//////////////////

SetDownsizeWork::SetDownsizeWork( uint8 status )
    : status_( status )
{
}

void
SetDownsizeWork::render( CamBuffer & buffer )
{
    buffer.reserve( 5 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x54;
    data[ 3 ] = 0x01;
    data[ 4 ] = status_;
    buffer.length_ = 5;
}

WorkItem::ResponseBound
SetDownsizeWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
SetDownsizeWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "SetDownsize - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "SetDownsize - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x54 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "SetDownsize - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
SetDownsizeWork::name( void )
{
    return nameOf< SetDownsizeWork >();
}

////////////////
// ImageSizeWork
////////////////

ImageSizeWork::ImageSizeWork( ImageSize & result )
    : result_( result )
{
}

void
ImageSizeWork::render( CamBuffer & buffer )
{
    buffer.reserve( 8 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x30;  // read data
    data[ 3 ] = 0x04;
    data[ 4 ] = 0x04;
    data[ 5 ] = 0x01;
    data[ 6 ] = 0x00;
    data[ 7 ] = 0x19;
    buffer.length_ = 8;
}

WorkItem::ResponseBound
ImageSizeWork::responseSize( void )
{
    return ResponseBound( 6, 6 );
}

WorkState
ImageSizeWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 6 ) {
#ifndef LONG_SIZE_T
        printf( "ImageSize - wrong sized response buffer (%d / 6)\n", buffer.size() );
#else // size_t size
        printf( "ImageSize - wrong sized response buffer (%ld / 6)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x30 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "ImageSize - responce does not verify\n" );
        abort();
    }
    switch( data[ 5 ] ) {
    case 0x00:
        result_ = Image640x480;
        break;
    case 0x11:
        result_ = Image320x240;
        break;
    case 0x22:
        result_ = Image160x120;
        break;
    default:
        printf( "ImageSize - unknown size result %x\n", data[ 5 ] );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
ImageSizeWork::name( void )
{
    return nameOf< ImageSizeWork >();
}

///////////////////
// SetImageSizeWork
///////////////////

SetImageSizeWork::SetImageSizeWork( ImageSize size )
    : size_( size )
{
}

void
SetImageSizeWork::render( CamBuffer & buffer )
{
    buffer.reserve( 9 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x31;  // write data
    data[ 3 ] = 0x05;
    data[ 4 ] = 0x04;
    data[ 5 ] = 0x01;
    data[ 6 ] = 0x00;
    data[ 7 ] = 0x19;
    data[ 8 ] = size_;
    buffer.length_ = 9;
}

WorkItem::ResponseBound
SetImageSizeWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
SetImageSizeWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "SetImageSize - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "SetImageSize - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x31 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "SetImageSize - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
SetImageSizeWork::name( void )
{
    return nameOf< SetImageSizeWork >();
}

///////////////////
// MotionDetectWork
///////////////////

MotionDetectWork::MotionDetectWork( bool & result )
    : result_( result )
{
}

void
MotionDetectWork::render( CamBuffer & buffer )
{
    buffer.reserve( 4 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x38;
    data[ 3 ] = 0x00;
    buffer.length_ = 4;
}

WorkItem::ResponseBound
MotionDetectWork::responseSize( void )
{
    return ResponseBound( 6, 6 );
}

WorkState
MotionDetectWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 6 ) {
#ifndef LONG_SIZE_T
        printf( "MotionDetect - wrong sized response buffer (%d / 6)\n", buffer.size() );
#else // size_t size
        printf( "MotionDetect - wrong sized response buffer (%ld / 6)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool detect = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x39 ) && ( data[ 3 ] == 0x00 );
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x38 ) && ( data[ 3 ] == 0x00 );
    if( !ok && !detect ) {
        printf( "MotionDetect - responce does not verify\n" );
        buffer.print();
        abort();
    }
    if( detect ) {
        // we detected motion, it stands to reason that it is active
        result_ = true;
    } else {
        result_ = !!data[ 5 ];
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
MotionDetectWork::name( void )
{
    return nameOf< MotionDetectWork >();
}

//////////////////////
// SetMotionDetectWork
//////////////////////

SetMotionDetectWork::SetMotionDetectWork( bool state )
    : state_( state )
    , firstPass_( true )
{
}

void
SetMotionDetectWork::render( CamBuffer & buffer )
{
    if( firstPass_ ) {
        buffer.reserve( 7 );
        auto data = buffer.begin();
        data[ 0 ] = 0x56;
        data[ 1 ] = 0x00;
        data[ 2 ] = 0x42;  // motion ctrl
        data[ 3 ] = 0x03;
        data[ 4 ] = 0x00;
        data[ 5 ] = 0x01;
        data[ 6 ] = 0x01;
        buffer.length_ = 7;
    } else {
        buffer.reserve( 5 );
        auto data = buffer.begin();
        data[ 0 ] = 0x56;
        data[ 1 ] = 0x00;
        data[ 2 ] = 0x37;
        data[ 3 ] = 0x01;
        data[ 4 ] = static_cast< uint8 >( state_ );
        buffer.length_ = 5;
    }
}

WorkItem::ResponseBound
SetMotionDetectWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
SetMotionDetectWork::validate( CamBuffer & buffer )
{
    if( buffer.size() < 5 ) {
#ifndef LONG_SIZE_T
        printf( "SetMotionDetect - wrong sized response buffer (%d / 5+) pass %d\n", buffer.size(), firstPass_ ? 1 : 2 );
#else // size_t size
        printf( "SetMotionDetect - wrong sized response buffer (%ld / 5+) pass %d\n", buffer.size(), firstPass_ ? 1 : 2 );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = false;
    if( firstPass_ ) {
        ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x42 ) && ( data[ 3 ] == 0x00 );
    } else {
        ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x37 ) && ( data[ 3 ] == 0x00 );
    }
    if( !ok ) {
        printf( "SetMotionDetect - responce does not verify (pass %d)\n", firstPass_ ? 1 : 2 );
        buffer.print();
        abort();
    }
    if( firstPass_ ) {
        firstPass_ = false;
        return WorkStateSend;  // now send the next message
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
SetMotionDetectWork::name( void )
{
    return nameOf< SetMotionDetectWork >();
}

/////////////////////
// MotionDetectedWork
/////////////////////

MotionDetectedWork::MotionDetectedWork( bool & result )
    : result_( result )
{
}

void
MotionDetectedWork::render( CamBuffer & buffer )
{
    buffer.clear();
}

WorkItem::ResponseBound
MotionDetectedWork::responseSize( void )
{
    return ResponseBound( 0, 4 );
}

WorkState
MotionDetectedWork::validate( CamBuffer & buffer )
{
    result_ = false;
    if( buffer.size() == 4 ) {
        auto data = buffer.begin();
        bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x39 ) && ( data[ 3 ] == 0x00 );
        if( ok ) {
                result_ = true;
        }
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
MotionDetectedWork::name( void )
{
    return nameOf< MotionDetectedWork >();
}

//////////////////
// CompressionWork
//////////////////

CompressionWork::CompressionWork( uint8 & result )
    : result_( result )
{
}

void
CompressionWork::render( CamBuffer & buffer )
{
    buffer.reserve( 8 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x30;  // read data
    data[ 3 ] = 0x04;
    data[ 4 ] = 0x01;
    data[ 5 ] = 0x01;
    data[ 6 ] = 0x12;
    data[ 7 ] = 0x04;
    buffer.length_ = 8;
}

WorkItem::ResponseBound
CompressionWork::responseSize( void )
{
    return ResponseBound( 6, 6 );
}

WorkState
CompressionWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 6 ) {
#ifndef LONG_SIZE_T
        printf( "Compression - wrong sized response buffer (%d / 6)\n", buffer.size() );
#else // size_t size
        printf( "Compression - wrong sized response buffer (%ld / 6)\n", buffer.size() );
#endif // size_t
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x30 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "Compression - responce does not verify\n" );
        abort();
    }
    result_ = data[ 5 ];
    thunk_.fire();
    return WorkStateDone;
}

StringId
CompressionWork::name( void )
{
    return nameOf< CompressionWork >();
}

/////////////////////
// SetCompressionWork
/////////////////////

SetCompressionWork::SetCompressionWork( uint8 state )
    : state_( state )
{
}

void
SetCompressionWork::render( CamBuffer & buffer )
{
    buffer.reserve( 9 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x31;  // write data
    data[ 3 ] = 0x05;
    data[ 4 ] = 0x01;
    data[ 5 ] = 0x01;
    data[ 6 ] = 0x12;
    data[ 7 ] = 0x04;
    data[ 8 ] = state_;
    buffer.length_ = 9;
}

WorkItem::ResponseBound
SetCompressionWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
SetCompressionWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "SetCompression - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "SetCompression - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x31 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "SetCompression - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
SetCompressionWork::name( void )
{
    return nameOf< SetCompressionWork >();
}

///////////
// Ptz1Work
///////////

Ptz1Work::Ptz1Work( uint16 & w, uint16 & h, uint16 & wz, uint16 & hz, uint16 & pan, uint16 & tilt )
    : w_( w )
    , h_( h )
    , wz_( wz )
    , hz_( hz )
    , pan_( pan )
    , tilt_( tilt )
{
}

void
Ptz1Work::render( CamBuffer & buffer )
{
    buffer.reserve( 4 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x53;  // write data
    data[ 3 ] = 0x00;
    buffer.length_ = 4;
}

WorkItem::ResponseBound
Ptz1Work::responseSize( void )
{
    return ResponseBound( 17, 17 );
}

WorkState
Ptz1Work::validate( CamBuffer & buffer )
{
    if( buffer.size() != 17 ) {
#ifndef LONG_SIZE_T
        printf( "GetZoom - wrong sized response buffer (%d / 17)\n", buffer.size() );
#else // size_t size
        printf( "GetZoom - wrong sized response buffer (%ld / 17)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x53 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "GetZoom - responce does not verify\n" );
        abort();
    }
    w_ = static_cast< uint16 >(( static_cast< uint16 >( data[ 5 ] ) << 8 ) | static_cast< uint16 >( data[ 6 ] ));
    h_ = static_cast< uint16 >(( static_cast< uint16 >( data[ 7 ] ) << 8 ) | static_cast< uint16 >( data[ 8 ] ));
    wz_ = static_cast< uint16 >(( static_cast< uint16 >( data[ 9 ] ) << 8 ) | static_cast< uint16 >( data[ 10 ] ));
    hz_ = static_cast< uint16 >(( static_cast< uint16 >( data[ 11 ] ) << 8 ) | static_cast< uint16 >( data[ 12 ] ));
    pan_ = static_cast< uint16 >(( static_cast< uint16 >( data[ 13 ] ) << 8 ) | static_cast< uint16 >( data[ 14 ] ));
    tilt_ = static_cast< uint16 >(( static_cast< uint16 >( data[ 15 ] ) << 8 ) | static_cast< uint16 >( data[ 16 ] ));
    thunk_.fire();
    return WorkStateDone;
}

StringId
Ptz1Work::name( void )
{
    return nameOf< Ptz1Work >();
}

///////////
// Ptz2Work
///////////

Ptz2Work::Ptz2Work( uint16 wz, uint16 hz, uint16 pan, uint16 tilt )
    : wz_( wz )
    , hz_( hz )
    , pan_( pan )
    , tilt_( tilt )
{
}

void
Ptz2Work::render( CamBuffer & buffer )
{
    buffer.reserve( 12 );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x52;  // set zoom
    data[ 3 ] = 0x08;
    data[ 4 ] = static_cast< uint8 >( wz_ >> 8 );
    data[ 5 ] = static_cast< uint8 >( wz_ & 0xff );
    data[ 6 ] = static_cast< uint8 >( hz_ >> 8 );
    data[ 7 ] = static_cast< uint8 >( hz_ & 0xff );
    data[ 8 ] = static_cast< uint8 >( pan_ >> 8 );
    data[ 9 ] = static_cast< uint8 >( pan_ & 0xff );
    data[ 10 ] = static_cast< uint8 >( tilt_ >> 8 );
    data[ 11 ] = static_cast< uint8 >( tilt_ & 0xff );
    buffer.length_ = 12;
}

WorkItem::ResponseBound
Ptz2Work::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
Ptz2Work::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "SetZoom - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "SetZoom - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x52 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "SetZoom - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
Ptz2Work::name( void )
{
    return nameOf< Ptz2Work >();
}

//////////
// OsdWork
//////////

OsdWork::OsdWork( uint8 x, uint8 y, StringId const & text )
    : x_( x )
    , y_( y )
    , text_( text )
{
}

void
OsdWork::render( CamBuffer & buffer )
{
    size_t len = text_.length();
    if( len > 14 ) {
        len = 14;
    }
    size_t bufsize = len + 6;
    buffer.reserve( bufsize );
    auto data = buffer.begin();
    data[ 0 ] = 0x56;
    data[ 1 ] = 0x00;
    data[ 2 ] = 0x45;  // osd add char
    data[ 3 ] = static_cast< uint8 >( len );
    data[ 4 ] = static_cast< uint8 >( len - 1 );
    data[ 5 ] = static_cast< uint8 >( ( y_ & 0x0f) | (( x_ & 0x03 ) << 4) );
    auto txt = text_.c_str();
    for( int i=0; i<len; ++i ) {
        auto c = txt[ i ];
	if(( c >= '0' ) && ( c <= '9' )) {
            c -= '0';
	} else if(( c >= 'A' ) && ( c <= 'Z' )) {
            c -= 'A';
	    c += 10;
	} else if(( c >= 'a' ) && ( c <= 'z' )) {
            c -= 'a';
	    c += 36;
	}
        data[ i + 6 ] = static_cast< uint8 >( c );
    }
    buffer.length_ = bufsize;
}

WorkItem::ResponseBound
OsdWork::responseSize( void )
{
    return ResponseBound( 5, 5 );
}

WorkState
OsdWork::validate( CamBuffer & buffer )
{
    if( buffer.size() != 5 ) {
#ifndef LONG_SIZE_T
        printf( "OSD - wrong sized response buffer (%d / 5)\n", buffer.size() );
#else // size_t size
        printf( "OSD - wrong sized response buffer (%ld / 5)\n", buffer.size() );
#endif // size_t size
        abort();
    }
    auto data = buffer.begin();
    bool ok = ( data[ 0 ] == 0x76 ) && ( data[ 1 ] == 0x00 ) && ( data[ 2 ] == 0x45 ) && ( data[ 3 ] == 0x00 );
    if( !ok ) {
        printf( "OSD - responce does not verify\n" );
        abort();
    }
    thunk_.fire();
    return WorkStateDone;
}

StringId
OsdWork::name( void )
{
    return nameOf< OsdWork >();
}
