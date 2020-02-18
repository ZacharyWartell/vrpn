#ifndef vrpn_LEAP_MOTION_TRACKER_H
#define vrpn_LEAP_MOTION_TRACKER_H
#include <stdio.h> // for NULL, FILE

/*
@author Zachary Wartell

*/
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
    
    /// This function should be called each time through app mainloop.
    virtual void mainloop();


    void setRedundantTransmission(vrpn_RedundantTransmission *);

protected:
    vrpn_float64 update_rate;

    vrpn_RedundantTransmission *d_redundancy;
};

// End of vrpn_LEAP_MOTION_TRACKER_H
#endif
