#ifndef vrpn_LEAP_MOTION_TRACKER_H
#define vrpn_LEAP_MOTION_TRACKER_H
/*
@author Zachary Wartell
@brief

Stage I - copy code from vrpn_Tracker_NULL

@copyright Copyright Zachary Wartell 2020.
*/
#include <stdio.h> // for NULL, FILE

/*
@author Zachary Wartell

*/
//#define LEAP_API_INTERNAL true
#include "Leap.h"
#include "vrpn_Tracker.h"
#include <list>

class VRPN_API vrpn_RedundantTransmission;

#define LEAPMOTION_DEBUG_OPENL_WINDOW
#define USE_GLASSES_TRACKING
#ifdef USE_GLASSES_TRACKING
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/opengl.hpp"
#endif

namespace vrpnExt {

    /*
    @author Zachary Wartell
    @brief C++ wrapper for C q_vec_type

    ZJW: [work-in-progress] I'll add to wrappers as I need them...
    */
    class q_vec {
    public:
        typedef double Scalar;

        // for tag dispatch constructor
        struct ZERO_VECTOR {
        };

        // tag dispatch constructor
        inline q_vec(ZERO_VECTOR tag) { v[0] = v[1] = v[2] = 0.0; }
        inline q_vec(q_vec_type v0) { q_vec_copy(v, v0); }
        inline q_vec(Scalar x, Scalar y, Scalar z)
        {
            v[0] = x;
            v[1] = y;
            v[2] = z;
        }

        // typecast operators (cast to C style q_vec_type)
        inline operator Scalar *() { return &v[0]; }
        inline operator const Scalar *() { return &v[0]; }

        inline Scalar x() const { return v[0]; }
        inline Scalar y() const { return v[1]; }
        inline Scalar z() const { return v[2]; }

        q_vec &operator+=(const q_vec av)
        {
            v[0] += av.v[0];
            v[1] += av.v[1];
            v[2] += av.v[2];
            return *this;
        }

        q_vec &operator*=(Scalar s)
        {
            v[0] *= s;
            v[1] *= s;
            v[2] *= s;
            return *this;
        }

        q_vec &operator=(const q_vec &qv)
        {
            memcpy(&v, &qv.v, sizeof(q_vec));
            return *this;
        }

        q_vec &operator=(const Leap::Vector &lv)
        {
            v[0] = lv.x;
            v[1] = lv.y;
            v[2] = lv.z;
            return *this;
        }

        friend inline q_vec operator+(const q_vec &v1, const q_vec &v2)
        {
            q_vec_type t;
            q_vec_add(t, v1.v, v2.v);
            return q_vec(t);
        }

        std::string toString() const
        {
            std::stringstream result;
            result << "(" << x() << ", " << y() << ", " << z() << ")";
            return result.str();
        }

        friend std::ostream &operator<<(std::ostream &out, const q_vec &v)
        {
            return out << v.toString();
        }

    private:
        q_vec_type v;
    };

    /*
    @author Zachary Wartell
    */
    class VRPN_API vrpn_CoordinateSystem {
    public:
        vrpn_CoordinateSystem()
            : child(nullptr)
        {
        }
        q_type quat;
        q_vec_type location;
        vrpn_CoordinateSystem *child;
    };

} // namespace vrpnExt

using namespace vrpnExt;
// This is an example of a tracker server.  It basically reports the
// position at the origin with zero velocity and acceleration over and
// over again at the rate requested.  It is here mostly as an example of
// how to build a tracker server, and also serves as a test object for
// client codes and VRPN builds.

/*
@author Zachary Wartell


file:///C:/Users/zwartell/Dropbox%20(UNC%20Charlotte)/Research/vrpn/submodules/LeapSDK/docs/cpp/devguide/Leap_Coordinate_Mapping.html

P1) Units "The Leap Motion Controller provides coordinates in units of real
world millimeters within the Leap Motion frame of reference"
*/
class VRPN_API vrpn_Tracker_LeapMotion : public vrpn_Tracker {

public:
    vrpn_Tracker_LeapMotion(const char *name, vrpn_Connection *c,
                            vrpn_int32 sensors = 1, vrpn_float64 Hz = 1.0);
    ~vrpn_Tracker_LeapMotion();
    /// This function should be called each time through app mainloop.
    virtual void mainloop();

    void setRedundantTransmission(vrpn_RedundantTransmission *);

    enum struct Finger { Thumb, Index, Middle, Ring, Pinky };
    enum struct Bone { Metacarpal, Proximal, Middle, Distal };

    static const unsigned short CAMERA_IMAGE_WIDTH = 640;
    static const unsigned short CAMERA_IMAGE_HEIGHT = 240;  

#if 0
    enum struct Finger { Thumb, Index, Middle, Ring, Pinky };
    enum struct Bone { Metacarpal, Proximal, Middle, Distal };
#endif

    class VRPN_API Hand {
    public:
        Hand();

        const static unsigned FINGER_SENSORS =
            (int(Leap::Finger::TYPE_PINKY) + 1) *
            (int(Leap::Bone::TYPE_DISTAL) + 1);
        const static unsigned HAND_SENSORS = FINGER_SENSORS + 5;

        vrpn_CoordinateSystem elbow;
        vrpn_CoordinateSystem wrist;
        vrpn_CoordinateSystem palm;

        vrpn_CoordinateSystem thumbMetacarpal;
        vrpn_CoordinateSystem thumbProximal;
        vrpn_CoordinateSystem thumbMiddle;
        vrpn_CoordinateSystem thumbDistal;

        vrpn_CoordinateSystem indexMetacarpal;
        vrpn_CoordinateSystem indexProximal;
        vrpn_CoordinateSystem indexMiddle;
        vrpn_CoordinateSystem indexDistal;

        vrpn_CoordinateSystem middleMetacarpal;
        vrpn_CoordinateSystem middleProximal;
        vrpn_CoordinateSystem middleMiddle;
        vrpn_CoordinateSystem middleDistal;

        vrpn_CoordinateSystem ringMetacarpal;
        vrpn_CoordinateSystem ringProximal;
        vrpn_CoordinateSystem ringMiddle;
        vrpn_CoordinateSystem ringDistal;

        vrpn_CoordinateSystem pinkyMetacarpal;
        vrpn_CoordinateSystem pinkyProximal;
        vrpn_CoordinateSystem pinkyMiddle;
        vrpn_CoordinateSystem pinkyDistal;

        /*
        \brief return finger CoordianteSystem data member by Leap index name
        */
        inline vrpn_CoordinateSystem &fingerJoint(Leap::Finger::Type f,
                                                  Leap::Bone::Type b)
        {
            return *(&thumbMetacarpal + (int)f * 4 + (int)b);
        }
    };
    Hand leftHand;
    Hand rightHand;

    /*
    \brief left hand is with in order that data members are listed
    vrpn_Tracker_LeapMotion::Hand, then right hand.
    */
    inline vrpn_CoordinateSystem &bySensorID(unsigned int sid)
    {
        if (sid < 3)
            return *(&leftHand.elbow + sid);
        else if (sid < 3 + Hand::FINGER_SENSORS)
            return *(&leftHand.thumbMetacarpal + sid - 3);
        else if (sid < Hand::FINGER_SENSORS * 2 + 3)
            return *(&rightHand.elbow + sid - (Hand::FINGER_SENSORS * 2 + 3));
        else
            return *(&rightHand.thumbMetacarpal +
                     (sid - (Hand::FINGER_SENSORS * 2 + 3)));
    }

protected:
    bool debugOutput = false;
    vrpn_float64 update_rate;

    vrpn_RedundantTransmission *d_redundancy;

protected:
    int read_config_file(FILE *config_file, const char *tracker_name);

    /*
    @author Zachary Wartell

    C++ coding and porting by Zachary Wartell. Algorithm by Rajarshi Roy
    originally in OpenCV and Processing
    (https://github.com/rajarshiroy/CS231A_PROJECT)
    */
    class VRPN_API GlassesTracking {
    public:
        GlassesTracking();
        void update(const Leap::Frame &frame);

        const q_vec &leftMarker() const { return leftMarkerPosAvg; }
        const q_vec &rightMarker() const { return rightMarkerPosAvg; }
        const cv::Mat &left() const { return left_; }
        const cv::Mat &right() const { return right_; }

    private:
        static const unsigned short QUEUE_LENGTH = 3;
      
        std::list<q_vec> leftMarkerQueue;
        std::list<q_vec> rightMarkerQueue;

        Leap::Image leftCam;
        Leap::Image rightCam;
        bool leapInit = false;
        // OpenCV leftcvUnstretched, rightcvUnstretched, leftcv, rightcv;
        // PImage srcLeft, srcRight;

#ifdef USE_GLASSES_TRACKING
        // Blob detector
        // cv::Ptr<cv::FeatureDetector>
        cv::Ptr<cv::SimpleBlobDetector> blobDetector;

        // Rectified image plane marker position
        q_vec leftCamLeftMarker;
        q_vec leftCamRightMarker;
        q_vec rightCamLeftMarker;
        q_vec rightCamRightMarker;
        // Triangulated 3D marker position
        q_vec leftMarkerPos;
        q_vec rightMarkerPos;

        // Moving average queues for smoothing
        std::list<q_vec> leftMarkerPosQueue;
        std::list<q_vec> rightMarkerPosQueue;
        q_vec leftMarkerPosAvg;
        q_vec rightMarkerPosAvg;
#endif
        cv::Mat leftUnstretched;
        cv::Mat rightUnstretched;
        cv::Mat left_;
        cv::Mat right_;
        cv::Mat leftCopy;
        cv::Mat rightCopy;
#if 0
        leftcvUnstretched = new OpenCV(this, 640, 240);
        rightcvUnstretched = new OpenCV(this, 640, 240);
        leftcv = new OpenCV(this, 640, 480);
        rightcv = new OpenCV(this, 640, 480);
#endif
    };

    class VRPN_API Eyes {
    public:
        Eyes();

        inline q_vec_type &left()
        {
            q_vec_add(left_, middle, toLeft);
            return left_;
        }
        inline q_vec_type &right()
        {
            q_vec_subtract(right_, middle, toLeft);
            return right_;
        }
        inline float separation() const { return separation_; }
        inline void separation(float s)
        {
            separation_ = s;
            q_vec_subtract(toLeft, leftMarker, rightMarker);
            q_vec_normalize(toLeft, toLeft);
            q_vec_scale(toLeft, s * 0.5, toLeft);
        }

    private:
        void update();

    private:
        q_vec_type left_;
        q_vec_type right_;
        float separation_;

        q_vec toLeft;
        q_vec middle;
        q_vec leftMarker;
        q_vec rightMarker;
    };

    class VRPN_API Listener : public Leap::Listener {
    public:
        virtual void onInit(const Leap::Controller &);
        virtual void onConnect(const Leap::Controller &);
        virtual void onDisconnect(const Leap::Controller &);
        virtual void onExit(const Leap::Controller &);
        virtual void onFrame(const Leap::Controller &);
        virtual void onImages(const Leap::Controller &) override;
        virtual void onFocusGained(const Leap::Controller &);
        virtual void onFocusLost(const Leap::Controller &);
        virtual void onDeviceChange(const Leap::Controller &);
        virtual void onServiceConnect(const Leap::Controller &);
        virtual void onServiceDisconnect(const Leap::Controller &);

        vrpn_Tracker_LeapMotion *vrpnTracker;
#ifdef USE_GLASSES_TRACKING
        GlassesTracking glassesTracking;
#endif

    private:
    };

    Listener listener;
    Leap::Controller controller;

    private:
#if defined(USE_GLASSES_TRACKING) && defined(LEAPMOTION_DEBUG_OPENL_WINDOW)
    static void on_opengl(vrpn_Tracker_LeapMotion *tracker_LeapMotion);
    cv::ogl::Texture2D textRight;
    cv::ogl::Texture2D textLeft;
#endif

public:
};
// End of vrpn_LEAP_MOTION_TRACKER_H
#endif