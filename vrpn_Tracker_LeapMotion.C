/*
@author Zachary Wartell
@brief

Stage I - copy code from vrpn_Tracker_NULL

*/

/* BEGIN COPY FROM vrpn_Tracker.C */
#include <ctype.h>  // for isspace
#include <stdio.h>  // for fprintf, stderr, NULL, etc
#include <string.h> // for memcpy, strlen, strncmp, etc

// NOTE: a vrpn tracker must call user callbacks with tracker data (pos and
//       ori info) which represent the transformation xfSourceFromSensor.
//       This means that the pos info is the position of the origin of
//       the sensor coord sys in the source coord sys space, and the
//       quat represents the orientation of the sensor relative to the
//       source space (ie, its value rotates the source's axes so that
//       they coincide with the sensor's)

// Include vrpn_Shared.h _first_ to avoid conflicts with sys/time.h
// and unistd.h
#include "vrpn_Shared.h" // for timeval, vrpn_buffer, etc

#ifdef _WIN32
#ifndef _WIN32_WCE
#include <io.h>
#endif
#endif

#include "vrpn_RedundantTransmission.h" // for vrpn_RedundantTransmission
#include "vrpn_Tracker_LeapMotion.h"
#include "quat.h"

/* END COPY FROM vrpn_Tracker.C */

/*
@uathor Zachary Wartell
@brief

- Stage I - copy code from vrpn_Tracker_NULL
*/
vrpn_Tracker_LeapMotion::vrpn_Tracker_LeapMotion(const char *name,
                                                 vrpn_Connection *c,
                                     vrpn_int32 sensors, vrpn_float64 Hz)
    : vrpn_Tracker(name, c)
    , update_rate(Hz)
    , d_redundancy(NULL)
{
    num_sensors = sensors;
    register_server_handlers();
    // Nothing left to do
}

/*
@uathor Zachary Wartell
@brief

- Stage I - copy code from vrpn_Tracker_NULL
*/
void vrpn_Tracker_LeapMotion::mainloop()
{
    {
        struct timeval current_time;
        char msgbuf[1000];
        vrpn_int32 i, len;

        // Call the generic server mainloop routine, since this is a server
        server_mainloop();

        // See if its time to generate a new report
        vrpn_gettimeofday(&current_time, NULL);
        if (vrpn_TimevalDuration(current_time, timestamp) >=
            1000000.0 / update_rate) {

            // Update the time
            timestamp.tv_sec = current_time.tv_sec;
            timestamp.tv_usec = current_time.tv_usec;

            // Send messages for all sensors if we have a connection
            if (d_redundancy) {
                for (i = 0; i < num_sensors; i++) {
                    d_sensor = i;

                    // Pack position report
                    len = encode_to(msgbuf);
                    if (d_redundancy->pack_message(
                            len, timestamp, position_m_id, d_sender_id, msgbuf,
                            vrpn_CONNECTION_LOW_LATENCY)) {
                        fprintf(stderr,
                                "NULL tracker: can't write message: tossing\n");
                    }

                    // Pack velocity report
                    len = encode_vel_to(msgbuf);
                    if (d_redundancy->pack_message(
                            len, timestamp, velocity_m_id, d_sender_id, msgbuf,
                            vrpn_CONNECTION_LOW_LATENCY)) {
                        fprintf(stderr,
                                "NULL tracker: can't write message: tossing\n");
                    }

                    // Pack acceleration report
                    len = encode_acc_to(msgbuf);
                    if (d_redundancy->pack_message(
                            len, timestamp, accel_m_id, d_sender_id, msgbuf,
                            vrpn_CONNECTION_LOW_LATENCY)) {
                        fprintf(stderr,
                                "NULL tracker: can't write message: tossing\n");
                    }
                }
            }
            else if (d_connection) {
                for (i = 0; i < num_sensors; i++) {
                    d_sensor = i;

                    // Pack position report
                    len = encode_to(msgbuf);
                    if (d_connection->pack_message(
                            len, timestamp, position_m_id, d_sender_id, msgbuf,
                            vrpn_CONNECTION_LOW_LATENCY)) {
                        fprintf(stderr,
                                "NULL tracker: can't write message: tossing\n");
                    }

                    // Pack velocity report
                    len = encode_vel_to(msgbuf);
                    if (d_connection->pack_message(
                            len, timestamp, velocity_m_id, d_sender_id, msgbuf,
                            vrpn_CONNECTION_LOW_LATENCY)) {
                        fprintf(stderr,
                                "NULL tracker: can't write message: tossing\n");
                    }

                    // Pack acceleration report
                    len = encode_acc_to(msgbuf);
                    if (d_connection->pack_message(
                            len, timestamp, accel_m_id, d_sender_id, msgbuf,
                            vrpn_CONNECTION_LOW_LATENCY)) {
                        fprintf(stderr,
                                "NULL tracker: can't write message: tossing\n");
                    }
                }
            }
        }
    }
}

/*
@author Zachary Wartell
@brief

- Stage I - copy code from vrpn_Tracker_NULL
*/
void vrpn_Tracker_LeapMotion::setRedundantTransmission(vrpn_RedundantTransmission *t)
{
    d_redundancy = t;
}