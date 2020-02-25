/*
@author Zachary Wartell
@brief

Stage I - copy code from vrpn_Tracker_NULL

@copyright Copyright Zachary Wartell 2020.

REFERENCES

- [R1] Rajarshi RoyStanford University (rroy@stanford.edu). Eyeglass positional
tracking using leap motion controller for parallax projection.
[https://www.youtube.com/watch?v=2AcUEn5iE2s][https://web.stanford.edu/class/cs231a/prev_projects_2016/rroy_finalreport.pdf]

- [R2] Image API Basics. Leap Motion developer documentation
*/

/* BEGIN COPY FROM vrpn_Tracker.C */
#include <cassert>
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

#ifdef USE_GLASSES_TRACKING
#include <GL/GL.h>
#include <opencv2/highgui.hpp>
#endif
using namespace vrpnExt;

/* END COPY FROM vrpn_Tracker.C */

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
#ifdef USE_GLASSES_TRACKING
#if 1
    cv::namedWindow("window", cv::WINDOW_OPENGL);
    cv::resizeWindow("window", CAMERA_IMAGE_WIDTH * 2, CAMERA_IMAGE_HEIGHT * 2);
    cv::setMouseCallback("window", NULL);
    cv::setOpenGlDrawCallback("window",
                              reinterpret_cast<cv::OpenGlDrawCallback>(
                                  vrpn_Tracker_LeapMotion::on_opengl),
                              this);
#endif
    controller.setPolicyFlags(Leap::Controller::POLICY_IMAGES);
#endif
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

    cv::updateWindow("window");
    cv::waitKey(1);

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

void vrpn_Tracker_LeapMotion::Listener::onImages(
    const Leap::Controller& controller)
{
#ifdef USE_GLASSES_TRACKING
    // std::cout << __FUNCTION__ << std::endl;
    glassesTracking.update(controller.frame());
    std::cout << "Glasses Markers: " << glassesTracking.leftMarker() << " "
              << glassesTracking.rightMarker() << std::endl;
#endif
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
        vrpn_Tracker_LeapMotion::Hand& vrpnHand =
            hand.isLeft() ? vrpnTracker->leftHand : vrpnTracker->rightHand;

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
        q_vec_set(vrpnHand.palm.location, hand.palmPosition()[0],
                  hand.palmPosition()[1], hand.palmPosition()[2]);
        // \todo [BUG?] check Leap vs VRPN convention for ypr (xyz vs yxz, etc.)
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

    // \todo [PRIORIY=LOW] LM tools add into VRPN

    // Get tools
    const Leap::ToolList tools = frame.tools();
    for (Leap::ToolList::const_iterator tl = tools.begin(); tl != tools.end();
         ++tl) {
        const Leap::Tool tool = *tl;
        std::cout << std::string(2, ' ') << "Tool, id: " << tool.id()
                  << ", position: " << tool.tipPosition()
                  << ", direction: " << tool.direction() << std::endl;
    }

    // \todo ZJW - probably leave this out or VRPN or convert to VRPN Button
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

inline void glTV(float tx, float ty, float x, float y)
{
    glTexCoord2f(tx, ty);
    glVertex2f(x, y);
}
/*
@author Zachary Wartell

@todo [PRIORITY=LOW][PERFORMANCE] improve performance of transfer to texture image (low priority as this display is for just debugging)
*/
void vrpn_Tracker_LeapMotion::on_opengl(
    vrpn_Tracker_LeapMotion* tracker_LeapMotion)
{
    static const GLubyte WHITE[] = {255, 255, 255};
    
    glClear(GL_COLOR_BUFFER_BIT);

    // set viewport for left half of window
    glViewport(0, 0, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT * 2);
    
    if (tracker_LeapMotion->textLeft.empty())
        tracker_LeapMotion->textLeft.create(
            CAMERA_IMAGE_HEIGHT * 2, CAMERA_IMAGE_WIDTH,
                        cv::ogl::Texture2D::Format::DEPTH_COMPONENT);
    
    tracker_LeapMotion->textLeft.copyFrom(
        tracker_LeapMotion->listener.glassesTracking.left());
    tracker_LeapMotion->textLeft.bind();

    glLoadIdentity();
    glEnable(GL_TEXTURE_2D);

    glBegin(GL_QUADS);
    glColor3ubv(WHITE);
        glTV(0, 0, -1, +1);
        glTV(1, 0, +1, +1);
        glTV(1, 1, +1, -1);
        glTV(0, 1, -1, -1);
    glEnd();

    // set viewport for right half of window
    glViewport(CAMERA_IMAGE_WIDTH, 0, CAMERA_IMAGE_WIDTH,
               CAMERA_IMAGE_HEIGHT * 2);
    if (tracker_LeapMotion->textRight.empty())
        tracker_LeapMotion->textRight.create(
            CAMERA_IMAGE_HEIGHT * 2, CAMERA_IMAGE_WIDTH,
            cv::ogl::Texture2D::Format::DEPTH_COMPONENT);

    tracker_LeapMotion->textRight.copyFrom(
        tracker_LeapMotion->listener.glassesTracking.right());
    tracker_LeapMotion->textRight.bind();
    glLoadIdentity();
    glBegin(GL_QUADS);
    glColor3ubv(WHITE);
        glTV(0, 0, -1, +1);
        glTV(1, 0, +1, +1);
        glTV(1, 1, +1, -1);
        glTV(0, 1, -1, -1);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    /* set viewport back to left side of window */
    glViewport(0, 0, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT * 2);


    // debugging/testing code
    static const int coords[6][4][3] = {
        {{+1, -1, -1}, {-1, -1, -1}, {-1, +1, -1}, {+1, +1, -1}},
        {{+1, +1, -1}, {-1, +1, -1}, {-1, +1, +1}, {+1, +1, +1}},
        {{+1, -1, +1}, {+1, -1, -1}, {+1, +1, -1}, {+1, +1, +1}},
        {{-1, -1, -1}, {-1, -1, +1}, {-1, +1, +1}, {-1, +1, -1}},
        {{+1, -1, +1}, {-1, -1, +1}, {-1, -1, -1}, {+1, -1, -1}},
        {{-1, -1, +1}, {+1, -1, +1}, {+1, +1, +1}, {-1, +1, +1}}};
    for (int i = 0; i < 6; ++i) {
        glColor3ub(i * 20, 100 + i * 10, i * 42);
        glBegin(GL_QUADS);
        for (int j = 0; j < 4; ++j) {
            glVertex3d(0.2 * coords[i][j][0], 0.2 * coords[i][j][1],
                       0.2 * coords[i][j][2]);
        }
        glEnd();
    }
}
vrpn_Tracker_LeapMotion::GlassesTracking::GlassesTracking()
    : leftCamLeftMarker(q_vec::ZERO_VECTOR())
    , leftCamRightMarker(q_vec::ZERO_VECTOR())
    , rightCamLeftMarker(q_vec::ZERO_VECTOR())
    , rightCamRightMarker(q_vec::ZERO_VECTOR())
    , leftMarkerPos(q_vec::ZERO_VECTOR())
    , rightMarkerPos(q_vec::ZERO_VECTOR())
    , leftMarkerPosAvg(q_vec::ZERO_VECTOR())
    , rightMarkerPosAvg(q_vec::ZERO_VECTOR())
    , leftUnstretched(cv::Size(640, 240), CV_8UC1)
    , rightUnstretched(cv::Size(640, 240), CV_8UC1)
    , left_(cv::Size(640, 480), CV_8UC1)
    , right_(cv::Size(640, 480), CV_8UC1)
    , leftCopy(cv::Size(640, 480), CV_8UC1)
    , rightCopy(cv::Size(640, 480), CV_8UC1)
{

    /*
    init blob detector
    see [R1]
    */
    cv::SimpleBlobDetector::Params params;

    params.thresholdStep = 5;
    params.minThreshold = 55;
    params.maxThreshold = 255;
    params.minRepeatability = 2;
    params.minDistBetweenBlobs = 10;
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByArea = true;
    params.minArea = 5;
    params.maxArea = 20;
    params.filterByCircularity = false;
    params.minCircularity = 8.0000001192092896e-01;
    params.maxCircularity = 3.4028234663852886e+38;
    params.filterByInertia = true;
    params.minInertiaRatio = 1.0000000149011612e-01;
    params.maxInertiaRatio = 3.4028234663852886e+38;
    params.filterByConvexity = true;
    params.minConvexity = 9.4999998807907104e-01;
    params.maxConvexity = 3.4028234663852886e+38;

    blobDetector = cv::SimpleBlobDetector::create(params);

    /*
    init queue's
    */
    for (int i = QUEUE_LENGTH; i >= 0; i--) {
        leftMarkerPosQueue.push_back(q_vec(0, 0, 0));
        rightMarkerPosQueue.push_back(q_vec(0, 0, 0));
    }
}

#define PORTED

void vrpn_Tracker_LeapMotion::GlassesTracking::update(const Leap::Frame& frame)
{
    // std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;
#ifdef PORTED
    if (!frame.images().isEmpty()) {
        leapInit = true;
        for (Leap::Image image : frame.images()) {
            if (image.id() == 0) {
                leftCam = image;
            }
            else {
                rightCam = image;
            }
        }
    }
#endif

    // If first images not yet captured then skip
    // Otherwise null pointer at leftCam, rightCam
    if (!leapInit) return;
        // std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;

#ifdef PORTED
        // Stretch images vertically*2 from 640*240->640*480
#if 0
    // error: ZJW mis-understanding OpenCV constructor; right now the nested for loops below is the only solution I have
    leftUnstretched.setTo(cv::Mat(leftUnstretched.size().height,leftUnstretched.size().width,leftUnstretched.type(),static_cast<const
        void*>(leftCam.data()));
    rightUnstretched.setTo(cv::Mat(rightUnstretched.size().height,rightUnstretched.size().width,rightUnstretched.type(),static_cast<const
        void*>((rightCam.data()));
#endif
    assert(leftCam.bytesPerPixel() == 1);
    assert(leftCam.width() == rightCam.width() &&
           leftCam.height() == rightCam.height() && rightCam.height() == 240);
    assert(leftCam.width() == right_.cols);
    /* \todo [PERFORMANCE] figure out how to memcpy using cv:Mat (must verify
     * that it legitimate in this case -- array layout, endianness etc.)
     */
    for (int i = 0; i < left_.rows; i += 2)
        for (int j = 0; j < left_.cols; j++) {
#if 0
            leftUnstretched.at<uchar>(i, j) =
                (leftCam.width() * j + i) * leftCam.bytesPerPixel();
            rightUnstretched.at<uchar>(i, j) =
                (rightCam.width() * j + i) * rightCam.bytesPerPixel();
#else
            left_.at<uchar>(i, j) = left_.at<uchar>(i + 1, j) =
                leftCam.data()[(leftCam.width() * (i / 2) + j) *
                               leftCam.bytesPerPixel()];
            right_.at<uchar>(i, j) = right_.at<uchar>(i + 1, j) =
                rightCam.data()[(rightCam.width() * (i / 2) + j) *
                                rightCam.bytesPerPixel()];
#endif
        };
            // left_.copyTo(leftCopy);
            // right_.copyTo(rightCopy);

#else
    PImage leftCamStretched = leftUnstretched.getSnapshot();
    PImage rightCamStretched = rightUnstretched.getSnapshot();
    leftCamStretched.resize(640, 480);
    rightCamStretched.resize(640, 480);
    leftcv.loadImage(leftCamStretched);
    rightcv.loadImage(rightCamStretched);

    // ZJW: Porting Note: skipped this: with a single channel image this step
    // seems uncessary in C++ OpenCV

    // Get OpenCV Matrices of Left and Right images
    // Output: leftImage, rightImage Mat
    srcLeft = leftcv.getSnapshot(); // src is PImage type
    srcRight = rightcv.getSnapshot();
    leftcv.gray();
    rightcv.gray();
    Mat leftImage = leftcv.getGray();
    Mat rightImage = rightcv.getGray();
#endif

#ifdef PORTED
    // Detect Blobs in Left and Right Images
    std::vector<cv::KeyPoint> blobsLeft;
    blobDetector->detect(left_, blobsLeft);
    std::vector<cv::KeyPoint> blobsRight;
    blobDetector->detect(right_, blobsRight);
    std::cout << "blobsRight.size(): " << blobsRight.size() << std::endl;
    std::cout << "blobsLeft.size(): " << blobsLeft.size() << std::endl;
#else
    // Detect Blobs in Left and Right Images
    // Output: blobsLeft, blobsRight List<KeyPoint>
    MatOfKeyPoint blobMatLeft = new MatOfKeyPoint();
    blobDetector.detect(leftImage, blobMatLeft);
    List<KeyPoint> blobsLeft = blobMatLeft.toList();
    MatOfKeyPoint blobMatRight = new MatOfKeyPoint();
    blobDetector.detect(rightImage, blobMatRight);
    List<KeyPoint> blobsRight = blobMatRight.toList();
#endif

#ifdef PORTED
    // Rectify blobsLeft and blobsRight
    std::vector<Leap::Vector> leftSlopes;
    std::vector<Leap::Vector> rightSlopes;

    Leap::Image leftCamRectifier(leftCam);
    Leap::Image rightCamRectifier(rightCam);
    // com.leapmotion.leap.Image leftCamRectifier = leftCam.getRaw();
    // com.leapmotion.leap.Image rightCamRectifier = rightCam.getRaw();
#else
    // Rectify blobsLeft and blobsRight
    ArrayList<PVector> leftSlopes = new ArrayList<PVector>();
    ArrayList<PVector> rightSlopes = new ArrayList<PVector>();
    com.leapmotion.leap.Image leftCamRectifier = leftCam.getRaw();
    com.leapmotion.leap.Image rightCamRectifier = rightCam.getRaw();
#endif
#ifdef PORTED
    for (int i = 0; i < blobsLeft.size(); i++)
        leftSlopes.push_back(leftCamRectifier.rectify(Leap::Vector(
            (float)blobsLeft[i].pt.x, (float)blobsLeft[i].pt.y / 2, 0)));
    for (int i = 0; i < blobsRight.size(); i++)
        rightSlopes.push_back(rightCamRectifier.rectify(Leap::Vector(
            (float)blobsRight[i].pt.x, (float)blobsRight[i].pt.y / 2, 0)));
#else
    if (blobsLeft.size() > 0) {
        for (int i = 0; i < blobsLeft.size(); i++) {
            KeyPoint blob = blobsLeft.get(i);
            com.leapmotion.leap.Vector blobvector =
                new com.leapmotion.leap.Vector((float)blob.pt.x,
                                               (float)blob.pt.y / 2, 0);
            com.leapmotion.leap.Vector blobslope =
                leftCamRectifier.rectify(blobvector);
            leftSlopes.add(new PVector(blobslope.get(0), blobslope.get(1), 0));
        }
    }

    if (blobsRight.size() > 0) {
        for (int i = 0; i < blobsRight.size(); i++) {
            KeyPoint blob = blobsRight.get(i);
            com.leapmotion.leap.Vector blobvector =
                new com.leapmotion.leap.Vector((float)blob.pt.x,
                                               (float)blob.pt.y / 2, 0);
            com.leapmotion.leap.Vector blobslope =
                rightCamRectifier.rectify(blobvector);
            rightSlopes.add(new PVector(blobslope.get(0), blobslope.get(1), 0));
        }
    }
#endif

#ifdef PORTED
    // Make a epipolar constraint filtered match list
    std::vector<Leap::Vector> leftSlopesEpifilt;
    std::vector<Leap::Vector> rightSlopesEpifilt;

    for (Leap::Vector leftSlope : leftSlopes) {
        for (Leap::Vector rightSlope : rightSlopes) {
            if (((leftSlope.x - rightSlope.x) < 0) &&
                ((rightSlope.x - leftSlope.x) < 0.2) &&
                ((leftSlope.y - rightSlope.y) < 0.025) &&
                ((rightSlope.y - leftSlope.y) < 0.025)) {
                leftSlopesEpifilt.push_back(leftSlope);
                rightSlopesEpifilt.push_back(rightSlope);
            }
        }
    }
#else
    // Make a epipolar constraint filtered match list
    ArrayList<PVector> leftSlopesEpifilt = new ArrayList<PVector>();
    ArrayList<PVector> rightSlopesEpifilt = new ArrayList<PVector>();
    for (PVector leftSlope : leftSlopes) {
        for (PVector rightSlope : rightSlopes) {
            if (((leftSlope.x - rightSlope.x) < 0) &&
                ((rightSlope.x - leftSlope.x) < 0.2) &&
                ((leftSlope.y - rightSlope.y) < 0.025) &&
                ((rightSlope.y - leftSlope.y) < 0.025)) {
                leftSlopesEpifilt.add(leftSlope);
                rightSlopesEpifilt.add(rightSlope);
            }
        }
    }
#endif
#ifdef PORTED
    // Triangulate filtered list
    // ZJW: according to [R1] these values are based on [R2]
    std::vector<Leap::Vector> triEpiFilt;
    for (int i = 0; i < leftSlopesEpifilt.size(); i++) {
        Leap::Vector leftPoint = leftSlopesEpifilt[i];
        Leap::Vector rightPoint = rightSlopesEpifilt[i];
        float z = 40 / (rightPoint.x - leftPoint.x);
        float y = z * (rightPoint.y + leftPoint.y) / 2;
        float x = 20 - z * (rightPoint.x + leftPoint.x) / 2;
#if 0
        if ((z > 200) && (z < 700) && (x > -500) && (x < 500) && (y > -300) &&
            (y < 300))
#endif
        triEpiFilt.push_back(Leap::Vector(x, y, z));
    }

#else
    // Triangulate filtered list
    ArrayList<PVector> triEpiFilt = new ArrayList<PVector>();
    for (int i = 0; i < leftSlopesEpifilt.size(); i++) {
        PVector leftPoint = leftSlopesEpifilt.get(i);
        PVector rightPoint = rightSlopesEpifilt.get(i);
        float z = 40 / (rightPoint.x - leftPoint.x);
        float y = z * (rightPoint.y + leftPoint.y) / 2;
        float x = 20 - z * (rightPoint.x + leftPoint.x) / 2;
        if ((z > 200) && (z < 700) && (x > -500) && (x < 500) && (y > -300) &&
            (y < 300)) {
            triEpiFilt.add(new PVector(x, y, z));
        }
    }
#endif
#ifdef PORTED
    // Further filtering if excess markers detected
    if (triEpiFilt.size() == 0) {
        // No markers detected: don't update either
    }
    else if (triEpiFilt.size() == 1) {
        // One marker detected: don't update either (for now)
    }
    else if (triEpiFilt.size() == 2) {
        // Two markers detected: left marker has smaller x
        Leap::Vector temp0 = triEpiFilt.at(0);
        Leap::Vector temp1 = triEpiFilt.at(1);
        if (temp0.x < temp1.x) {
            leftMarkerPos = temp0;
            rightMarkerPos = temp1;
        }
        else {
            leftMarkerPos = temp1;
            rightMarkerPos = temp0;
        }
    }
    else {
        // More than two markers detected
        // Find the two separated by real world marker distance:
        // 115 to 145
        for (Leap::Vector temp0 : triEpiFilt) {
            for (Leap::Vector temp1 : triEpiFilt) {
                // \todo [PERFORMANCE] replace with distSquared
                if ((temp0.distanceTo(temp1) > 115) &&
                    (temp0.distanceTo(temp1) < 145)) {
                    if (temp0.x < temp1.x) {
                        leftMarkerPos = temp0;
                        rightMarkerPos = temp1;
                    }
                    else {
                        leftMarkerPos = temp1;
                        rightMarkerPos = temp0;
                    }
                }
            }
        }
    }

#else
    // Further filtering if excess markers detected
    if (triEpiFilt.size() == 0) {
        // No markers detected: don't update either
    }
    else if (triEpiFilt.size() == 1) {
        // One marker detected: don't update either (for now)
    }
    else if (triEpiFilt.size() == 2) {
        // Two markers detected: left marker has smaller x
        PVector temp0 = triEpiFilt.get(0);
        PVector temp1 = triEpiFilt.get(1);
        if (temp0.x < temp1.x) {
            leftMarkerPos = temp0;
            rightMarkerPos = temp1;
        }
        else {
            leftMarkerPos = temp1;
            rightMarkerPos = temp0;
        }
    }
    else {
        // More than two markers detected
        // Find the two separated by real world marker distance:
        // 115 to 145
        for (PVector temp0 : triEpiFilt) {
            for (PVector temp1 : triEpiFilt) {
                if ((temp0.dist(temp1) > 115) && (temp0.dist(temp1) < 145)) {
                    if (temp0.x < temp1.x) {
                        leftMarkerPos = temp0;
                        rightMarkerPos = temp1;
                    }
                    else {
                        leftMarkerPos = temp1;
                        rightMarkerPos = temp0;
                    }
                }
            }
        }
    }

    System.out.println("distance: " + rightMarkerPos.dist(leftMarkerPos));
#endif
#ifdef PORTED
    // Smoothing via moving average
    leftMarkerPosQueue.push_back(leftMarkerPos);
    leftMarkerPosQueue.pop_front();
    rightMarkerPosQueue.push_back(rightMarkerPos);
    rightMarkerPosQueue.pop_front();

    leftMarkerPosAvg = q_vec(q_vec::ZERO_VECTOR());
    for (q_vec temp : leftMarkerPosQueue)
        leftMarkerPosAvg += temp;
    rightMarkerPosAvg = q_vec(q_vec::ZERO_VECTOR());
    for (q_vec temp : rightMarkerPosQueue)
        rightMarkerPosAvg += temp;

    leftMarkerPosAvg *= 1.0 / leftMarkerPosQueue.size();
    rightMarkerPosAvg *= 1.0 / rightMarkerPosQueue.size();
#else
    // Smoothing via moving average
    leftMarkerPosQueue.add(leftMarkerPos);
    leftMarkerPosQueue.removeFirst();
    rightMarkerPosQueue.add(rightMarkerPos);
    rightMarkerPosQueue.removeFirst();

    leftMarkerPosAvg = new PVector(0, 0, 0);
    rightMarkerPosAvg = new PVector(0, 0, 0);
    for (PVector temp : leftMarkerPosQueue) {
        leftMarkerPosAvg.add(temp);
    }
    for (PVector temp : rightMarkerPosQueue) {
        rightMarkerPosAvg.add(temp);
    }
    leftMarkerPosAvg.div(leftMarkerPosQueue.size());
    rightMarkerPosAvg.div(rightMarkerPosQueue.size());
#endif
}
