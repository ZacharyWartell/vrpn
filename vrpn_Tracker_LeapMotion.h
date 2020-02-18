#ifndef vrpn_LEAP_MOTION_TRACKER_H
#define vrpn_LEAP_MOTION_TRACKER_H
/*
@author Zachary Wartell
@brief

Stage I - copy code from vrpn_Tracker_NULL

*/
#include <stdio.h> // for NULL, FILE

/*
@author Zachary Wartell

*/
#include "Leap.h"
#include "vrpn_Tracker.h"

class VRPN_API vrpn_RedundantTransmission;

// This is an example of a tracker server.  It basically reports the
// position at the origin with zero velocity and acceleration over and
// over again at the rate requested.  It is here mostly as an example of
// how to build a tracker server, and also serves as a test object for
// client codes and VRPN builds.

/*
@author Zachary Wartell


*/
class VRPN_API vrpn_Tracker_LeapMotion : public vrpn_Tracker {
public:
    vrpn_Tracker_LeapMotion(const char *name, vrpn_Connection *c,
                            vrpn_int32 sensors = 1, vrpn_float64 Hz = 1.0);
    ~vrpn_Tracker_LeapMotion();
    /// This function should be called each time through app mainloop.
    virtual void mainloop();

    void setRedundantTransmission(vrpn_RedundantTransmission *);

protected:
    vrpn_float64 update_rate;

    vrpn_RedundantTransmission *d_redundancy;

private:
    class Listener : public Leap::Listener {
    public:
        virtual void onInit(const Leap::Controller &);
        virtual void onConnect(const Leap::Controller &);
        virtual void onDisconnect(const Leap::Controller &);
        virtual void onExit(const Leap::Controller &);
        virtual void onFrame(const Leap::Controller &);
        virtual void onFocusGained(const Leap::Controller &);
        virtual void onFocusLost(const Leap::Controller &);
        virtual void onDeviceChange(const Leap::Controller &);
        virtual void onServiceConnect(const Leap::Controller &);
        virtual void onServiceDisconnect(const Leap::Controller &);

    private:
    };
    Listener listener;
    Leap::Controller controller;
};

// End of vrpn_LEAP_MOTION_TRACKER_H
#endif
