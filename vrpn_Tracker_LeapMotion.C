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

#include "quat.h"
#include "vrpn_RedundantTransmission.h" // for vrpn_RedundantTransmission
#include "vrpn_Tracker_LeapMotion.h"

/* END COPY FROM vrpn_Tracker.C */

/**

[Stage I] Temporary hack
*/
#if 0
#define main HIDE_MAIN
#include "submodules\LeapSDK\samples\Sample.cpp"
#undef main
#endif

static const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring",
                                          "Pinky"};
static const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle",
                                        "Distal"};
static const std::string stateNames[] = {"STATE_INVALID", "STATE_START",
                                         "STATE_UPDATE", "STATE_END"};

vrpn_Tracker_LeapMotion::Hand::Hand()
{
    thumbMetacarpal.child = &thumbProximal;
    thumbProximal.child = &thumbMiddle;
    thumbMiddle.child = &thumbDistal;

    indexMetacarpal.child = &indexProximal;
    indexProximal.child = &indexMiddle;
    indexMiddle.child = &indexDistal;

    middleMetacarpal.child = &middleProximal;
    middleProximal.child = &middleMiddle;
    middleMiddle.child = &middleDistal;

    ringMetacarpal.child = &ringProximal;
    ringProximal.child = &ringMiddle;
    ringMiddle.child = &ringDistal;

    pinkyMetacarpal.child = &pinkyProximal;
    pinkyProximal.child = &pinkyMiddle;
    pinkyMiddle.child = &pinkyDistal;
}

/*
@uathor Zachary Wartell
@brief

- Stage I - copy code from vrpn_Tracker_NULL
*/
vrpn_Tracker_LeapMotion::vrpn_Tracker_LeapMotion(const char* name,
                                                 vrpn_Connection* c,
                                                 vrpn_int32 sensors,
                                                 vrpn_float64 Hz)
    : vrpn_Tracker(name, c)
    , update_rate(Hz)
    , d_redundancy(NULL)
{
    num_sensors = sensors;
    // num_sensors = sensors;
    num_sensors = 2 * Hand::HAND_SENSORS;

    register_server_handlers();

    // Have the sample listener receive events from the controller
    listener.vrpnTracker = this;
    controller.addListener(listener);

#if 0
    // [FIX-ME] from Sample.cpp
    if (argc > 1 && strcmp(argv[1], "--bg") == 0)
        controller.setPolicy(Leap::Leap::Controller::POLICY_BACKGROUND_FRAMES);
#endif
}

/*
@author Zachary Wartell
@brief

- Stage I - copy code from vrpn_Tracker_NULL
*/
void vrpn_Tracker_LeapMotion::mainloop()
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

                const vrpn_CoordinateSystem& sensorCS = bySensorID(i);
                q_vec_copy(pos, sensorCS.location);
                q_vec_copy(d_quat, sensorCS.location);

                // Pack position report
                len = encode_to(msgbuf);
                if (d_redundancy->pack_message(len, timestamp, position_m_id,
                                               d_sender_id, msgbuf,
                                               vrpn_CONNECTION_LOW_LATENCY)) {
                    fprintf(stderr,
                            "NULL tracker: can't write message: tossing\n");
                }
            }
        }
        else if (d_connection) {
            for (i = 0; i < num_sensors; i++) {
                d_sensor = i;

                const vrpn_CoordinateSystem& sensorCS = bySensorID(i);
                q_vec_copy(pos, sensorCS.location);
                q_vec_copy(d_quat, sensorCS.location);

                // Pack position report
                len = encode_to(msgbuf);
                if (d_connection->pack_message(len, timestamp, position_m_id,
                                               d_sender_id, msgbuf,
                                               vrpn_CONNECTION_LOW_LATENCY)) {
                    fprintf(stderr,
                            "NULL tracker: can't write message: tossing\n");
                }
            }
        }
    }
}

/*
@author Zachary Wartell
@brief

@todo this is unimplemented, just copied scaffolding code from vprn_Tracker.C

- Stage I - copy code from vrpn_Tracker.C
*/
int vrpn_Tracker_LeapMotion::read_config_file(FILE* config_file,
                                              const char* tracker_name)
{

    char line[512]; // line read from input file
    // vrpn_int32 num_sens;
    // vrpn_int32 which_sensor;
    // float f[14];
    // int i, j;

    // Read lines from the file until we run out
    while (fgets(line, sizeof(line), config_file) != NULL) {
        // Make sure the line wasn't too long
        if (strlen(line) >= sizeof(line) - 1) {
            fprintf(stderr, "Line too long in config file: %s\n", line);
            return -1;
        }
        // find tracker name in file
        if ((!(strncmp(line, tracker_name, strlen(tracker_name)))) &&
            (isspace(line[strlen(tracker_name)]))) {

            /**
            \todo Add LeapMotion specific config file items
            */
            return 0; // success
        }
    }
    fprintf(stderr, "Error reading or %s not found in config file\n",
            tracker_name);
    return -1;
}
/*
@author Zachary Wartell
@brief

- Stage I - copy code from vrpn_Tracker_NULL
*/
void vrpn_Tracker_LeapMotion::setRedundantTransmission(
    vrpn_RedundantTransmission* t)
{
    d_redundancy = t;
}

vrpn_Tracker_LeapMotion::~vrpn_Tracker_LeapMotion()
{
    // Remove the sample listener when done
    controller.removeListener(listener);
}

/*
BEGIN COPY AND NODIFIY FROM Leap SDK Sample.cpp
*/
void vrpn_Tracker_LeapMotion::Listener::onInit(
    const Leap::Controller& controller)
{
    std::cout << "Initialized" << std::endl;
}

void vrpn_Tracker_LeapMotion::Listener::onConnect(
    const Leap::Controller& controller)
{
    std::cout << "Connected" << std::endl;
    controller.enableGesture(Leap::Gesture::TYPE_CIRCLE);
    controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);
    controller.enableGesture(Leap::Gesture::TYPE_SCREEN_TAP);
    controller.enableGesture(Leap::Gesture::TYPE_SWIPE);
}

void vrpn_Tracker_LeapMotion::Listener::onDisconnect(
    const Leap::Controller& controller)
{
    // Note: not dispatched when running in a debugger.
    std::cout << "Disconnected" << std::endl;
}

void vrpn_Tracker_LeapMotion::Listener::onExit(
    const Leap::Controller& controller)
{
    std::cout << "Exited" << std::endl;
}

void vrpn_Tracker_LeapMotion::Listener::onFrame(
    const Leap::Controller& controller)
{
    const q_vec_type Y = {0, 1, 0};
    // Get the most recent frame and report some basic information
    const Leap::Frame frame = controller.frame();
    std::cout << "Leap::Frame id: " << frame.id()
              << ", timestamp: " << frame.timestamp()
              << ", hands: " << frame.hands().count()
              << ", extended fingers: " << frame.fingers().extended().count()
              << ", tools: " << frame.tools().count()
              << ", gestures: " << frame.gestures().count() << std::endl;

    Leap::HandList hands = frame.hands();
    for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end();
         ++hl) {
        // Get the first hand
        const Leap::Hand hand = *hl;
        std::string handType = hand.isLeft() ? "Left hand" : "Right hand";
        vrpn_Tracker_LeapMotion::Hand& vrpnHand = hand.isLeft() ? vrpnTracker->leftHand : vrpnTracker->rightHand;

        if (vrpnTracker->debugOutput)
            std::cout << std::string(2, ' ') << handType
                      << ", id: " << hand.id()
                      << ", palm position: " << hand.palmPosition()
                      << std::endl;
        // Get the hand's normal vector and direction
        const Leap::Vector normal = hand.palmNormal();
        const Leap::Vector direction = hand.direction();

        // Calculate the hand's pitch, roll, and yaw angles
        if (vrpnTracker->debugOutput)
            std::cout << std::string(2, ' ')
                      << "pitch: " << direction.pitch() * Leap::RAD_TO_DEG
                      << " degrees, "
                      << "roll: " << normal.roll() * Leap::RAD_TO_DEG
                      << " degrees, "
                      << "yaw: " << direction.yaw() * Leap::RAD_TO_DEG
                      << " degrees" << std::endl;

        // convert to VRPN CS
        q_vec_set(vrpnHand.palm.location, hand.palmPosition()[0], hand.palmPosition()[1],
                  hand.palmPosition()[2]);
        // \todo check Leap vs VRPN convention for ypr (xyz vs yxz, etc.)
        q_from_euler(vrpnHand.palm.quat, direction.yaw(), direction.pitch(),
                     direction.roll());

        // Get the Arm bone
        Leap::Arm arm = hand.arm();
        if (vrpnTracker->debugOutput)
            std::cout << std::string(2, ' ')
                      << "Arm direction: " << arm.direction()
                      << " wrist position: " << arm.wristPosition()
                      << " elbow position: " << arm.elbowPosition()
                      << std::endl;

        // convert to VRPN CS
        q_vec_set(vrpnHand.elbow.location, arm.elbowPosition()[0],
                  arm.elbowPosition()[1], arm.elbowPosition()[2]);        
        q_vec_set(vrpnHand.wrist.location, arm.wristPosition()[0],
                  arm.wristPosition()[1], arm.wristPosition()[2]);
        q_vec_type dir;
        q_vec_subtract(dir, vrpnHand.wrist.location, vrpnHand.elbow.location);
        q_from_two_vecs(vrpnHand.elbow.quat, dir, Y);


        // Get fingers
        const Leap::FingerList fingers = hand.fingers();
        for (Leap::FingerList::const_iterator fl = fingers.begin();
             fl != fingers.end(); ++fl) {
            const Leap::Finger finger = *fl;
            if (vrpnTracker->debugOutput)
                std::cout << std::string(4, ' ') << fingerNames[finger.type()]
                          << " finger, id: " << finger.id()
                          << ", length: " << finger.length()
                          << "mm, width: " << finger.width() << std::endl;

            // Get finger bones
            for (int b = 0; b < 4; ++b) {
                Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(b);
                Leap::Bone bone = finger.bone(boneType);
                if (vrpnTracker->debugOutput)
                    std::cout << std::string(6, ' ') << boneNames[boneType]
                              << " bone, start: " << bone.prevJoint()
                              << ", end: " << bone.nextJoint()
                              << ", direction: " << bone.direction()
                              << std::endl;

                // convert to VRPN CS
                vrpn_CoordinateSystem& fingerJoint =
                    vrpnHand.fingerJoint(finger.type(), boneType);

                q_vec_set(fingerJoint.location, bone.prevJoint()[0],
                          bone.prevJoint()[1], bone.prevJoint()[2]);
                q_vec_type dir;
                
                q_vec_set(dir, bone.direction()[0], bone.direction()[1],
                          bone.direction()[2]);
                q_from_two_vecs(fingerJoint.quat, dir, Y);
            }
        }
    }

    // TODO: ZJW - probably leave this out for VRPN

    // Get tools
    const Leap::ToolList tools = frame.tools();
    for (Leap::ToolList::const_iterator tl = tools.begin(); tl != tools.end();
         ++tl) {
        const Leap::Tool tool = *tl;
        std::cout << std::string(2, ' ') << "Tool, id: " << tool.id()
                  << ", position: " << tool.tipPosition()
                  << ", direction: " << tool.direction() << std::endl;
    }

    // TODO: ZJW - probably leave this out or VRPN or convert to VRPN Button
    // "event"?

    // Get gestures
    const Leap::GestureList gestures = frame.gestures();
    for (int g = 0; g < gestures.count(); ++g) {
        Leap::Gesture gesture = gestures[g];

        switch (gesture.type()) {
        case Leap::Gesture::TYPE_CIRCLE: {
            Leap::CircleGesture circle = gesture;
            std::string clockwiseness;

            if (circle.pointable().direction().angleTo(circle.normal()) <=
                Leap::PI / 2) {
                clockwiseness = "clockwise";
            }
            else {
                clockwiseness = "counterclockwise";
            }

            // Calculate angle swept since last frame
            float sweptAngle = 0;
            if (circle.state() != Leap::Gesture::STATE_START) {
                Leap::CircleGesture previousUpdate = Leap::CircleGesture(
                    controller.frame(1).gesture(circle.id()));
                sweptAngle = (circle.progress() - previousUpdate.progress()) *
                             2 * Leap::PI;
            }
            std::cout << std::string(2, ' ') << "Circle id: " << gesture.id()
                      << ", state: " << stateNames[gesture.state()]
                      << ", progress: " << circle.progress()
                      << ", radius: " << circle.radius() << ", angle "
                      << sweptAngle * Leap::RAD_TO_DEG << ", " << clockwiseness
                      << std::endl;
            break;
        }
        case Leap::Gesture::TYPE_SWIPE: {
            Leap::SwipeGesture swipe = gesture;
            std::cout << std::string(2, ' ') << "Swipe id: " << gesture.id()
                      << ", state: " << stateNames[gesture.state()]
                      << ", direction: " << swipe.direction()
                      << ", speed: " << swipe.speed() << std::endl;
            break;
        }
        case Leap::Gesture::TYPE_KEY_TAP: {
            Leap::KeyTapGesture tap = gesture;
            std::cout << std::string(2, ' ') << "Key Tap id: " << gesture.id()
                      << ", state: " << stateNames[gesture.state()]
                      << ", position: " << tap.position()
                      << ", direction: " << tap.direction() << std::endl;
            break;
        }
        case Leap::Gesture::TYPE_SCREEN_TAP: {
            Leap::ScreenTapGesture screentap = gesture;
            std::cout << std::string(2, ' ')
                      << "Screen Tap id: " << gesture.id()
                      << ", state: " << stateNames[gesture.state()]
                      << ", position: " << screentap.position()
                      << ", direction: " << screentap.direction() << std::endl;
            break;
        }
        default:
            std::cout << std::string(2, ' ') << "Unknown gesture type."
                      << std::endl;
            break;
        }
    }

    if (!frame.hands().isEmpty() || !gestures.isEmpty()) {
        std::cout << std::endl;
    }
}

void vrpn_Tracker_LeapMotion::Listener::onFocusGained(
    const Leap::Controller& controller)
{
    std::cout << "Focus Gained" << std::endl;
}

void vrpn_Tracker_LeapMotion::Listener::onFocusLost(
    const Leap::Controller& controller)
{
    std::cout << "Focus Lost" << std::endl;
}

void vrpn_Tracker_LeapMotion::Listener::onDeviceChange(
    const Leap::Controller& controller)
{
    std::cout << "Device Changed" << std::endl;
    const Leap::DeviceList devices = controller.devices();

    for (int i = 0; i < devices.count(); ++i) {
        std::cout << "id: " << devices[i].toString() << std::endl;
        std::cout << "  isStreaming: "
                  << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
}

void vrpn_Tracker_LeapMotion::Listener::onServiceConnect(
    const Leap::Controller& controller)
{
    std::cout << "Service Connected" << std::endl;
}

void vrpn_Tracker_LeapMotion::Listener::onServiceDisconnect(
    const Leap::Controller& controller)
{
    std::cout << "Service Disconnected" << std::endl;
}
/*
END COPY AND NODIFIY FROM Leap SDK Sample.cpp
*/
