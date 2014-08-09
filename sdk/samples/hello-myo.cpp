// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
//
#if defined(WIN32)
#include <conio.h>
#else
#include "conio.h"
#endif

#include "irrKlang.h"
//
using namespace irrklang;
// 
#pragma comment(lib, "irrKlang.lib") // link with irrKlang.dll
//
// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.

float ROLL = 0;
float PITCH = 0;
float YAW = 0;
unsigned int WAVEINCOUNTER = 1;
unsigned int FINGSERSPREADCOUNTER = 1;
myo::Pose CURRENTPOSE;
int THRESHOLD = 21;
float CURAVGROLL = (float)M_PI;   //to always center volume up/down based on most common resting position
myo::Arm WHICHARM;

class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;

        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        //hacky globals
        ROLL = roll;
        PITCH = pitch;
        YAW = yaw;

        // Convert the floating point angles in radians to a scale from 0 to 20.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        if (CURRENTPOSE == myo::Pose::waveIn) {
            WAVEINCOUNTER += 2;
        }
        if (CURRENTPOSE == myo::Pose::fingersSpread) {
            FINGSERSPREADCOUNTER += 2;
        }
        if (WAVEINCOUNTER != 1) {
            WAVEINCOUNTER -= 1;
        }
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        //register a wavein only if majority of recent samples are wavein
        
        currentPose = pose;
        CURRENTPOSE = pose;

        // Vibrate the Myo whenever we've detected that the user has made a fist.
        if (pose == myo::Pose::fist) {
            myo->vibrate(myo::Myo::vibrationShort);
        }
    }

    // onArmRecognized() is called whenever Myo has recognized a setup gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmRecognized(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        onArm = true;
        whichArm = arm;
        WHICHARM = arm;
    }

    // onArmLost() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmLost(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
        std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
                  << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
                  << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';

        if (onArm) {
            // Print out the currently recognized pose and which arm Myo is being worn on.

            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();

            std::cout << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                      << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        } else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << "[?]" << '[' << std::string(14, ' ') << ']';
        }

        std::cout << std::flush;
    }

    // These values are set by onArmRecognized() and onArmLost() above.
    bool onArm;
    myo::Arm whichArm;

    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
};

int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
        // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
        // publishing your application. The Hub provides access to one or more Myos.
        myo::Hub hub("com.example.hello-myo");
        std::cout << "Attempting to find a Myo..." << std::endl;
    
        // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
        // immediately.
        // waitForAnyMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
        // if that fails, the function will return a null pointer.
        myo::Myo* myo = hub.waitForMyo(10000);

        // If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
        if (!myo) {
            throw std::runtime_error("Unable to find a Myo!");
        }
        // We've found a Myo.
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

        // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
        DataCollector collector;

        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        hub.addListener(&collector);
        
        //CALIBRATE FOR SIMULATED SPATIAL LOCATION
        //Copy the start position of the calibration gesture during this time.
        double roll_orient = 0;
        double yaw_orient = 0;
        for(int i=0; i<50; i++) {   //about 1 second of sampling - keep still!
            hub.run(1000/20);
            roll_orient += ROLL;
            yaw_orient += YAW;
        }
        roll_orient /= 50;
        yaw_orient /= 50;
        
        int hand;
        
        // start the sound engine with default parameters
        ISoundEngine* engine = createIrrKlangDevice();
        if (!engine)
        {
            printf("Could not startup engine\n");
            return 0; // error starting up the engine
        }
        // play some sound stream, looped
        
        ISound *samples[3];
        
        samples[0] = engine->play3D("BeatK03B 70-01.wav", vec3df(0,0,0), true, false, true);
        samples[1] = engine->play3D("GrulerK03 70B-01.wav", vec3df(0,0,0), true, false, true);
        samples[2] = engine->play3D("Wind-Mark_DiAngelo-1940285615.wav", vec3df(0,0,0), true, false, true);
        
        for(int i = 0; i < 3; i++) {
            samples[i]->setPosition(vec3df(0,0,1));
        }
        
        engine->setListenerPosition(vec3df(0,0,0), vec3df(0,0,1));
        
        const float radius = 1;
        int currentSample = 0;
        float vol = .5;
        
        while(1)
        {
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
            hub.run(1000/20);
            
            // After processing events, we call the print() member function we defined above to print out the values we've
            // obtained from any events that have occurred.
            //
            // printf("Press any key to play some sound, press 'q' to quit.\n");
            // play a single sound
            collector.print();
        
            vec3df pos3d(radius * 2*cosf(YAW + yaw_orient - M_PI/2), 0, radius * 2*x`sinf(YAW + yaw_orient - M_PI/2));
            
            (WHICHARM == myo::armLeft ? hand=1 : hand=-1);
            vol = hand*(-ROLL) + roll_orient + 1;
            
            if (vol > 1)
                vol = 1;
            if (vol < 0)
                vol = 0;
        
            if (CURRENTPOSE == myo::Pose::fist) {
                if (samples[currentSample]) {
                    samples[currentSample]->setPosition(pos3d);            }
            }
            if (CURRENTPOSE == myo::Pose::fingersSpread) {
                if (samples[currentSample]) {
                    samples[currentSample]->setVolume(vol);
                }
            }
            if (CURRENTPOSE == myo::Pose::waveIn && WAVEINCOUNTER > THRESHOLD) {
                currentSample += 1;
                currentSample %= 3;
                WAVEINCOUNTER = 1;
            }
            
            std::cout << '[' << "Sample: " << currentSample << "    Thresh: " << WAVEINCOUNTER << "    Vol: " << vol << "   roll/yaw orient: " << roll_orient << "  " << yaw_orient << "]";

        }
        // If a standard exception occurred, we print out its message and exit.
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cerr << "Press enter to continue.";
            std::cin.ignore();
            return 1;
        }
}
