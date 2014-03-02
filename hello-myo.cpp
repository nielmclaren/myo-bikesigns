// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Confidential and not for redistribution. See LICENSE.txt.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo.hpp>

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2f;
        using std::asinf;
        using std::sqrtf;

        // Calculate the normalized quaternion.
        float norm = sqrtf(quat.x() * quat.x() + quat.y() * quat.y() + quat.z() * quat.z() + quat.w() * quat.w());
        myo::Quaternion<float> normalized(quat.x()/norm, quat.y()/norm, quat.z()/norm, quat.w()/norm);

        // Calculate Euler angles (roll, pitch, and yaw) from the normalized quaternion.
        float roll = atan2f(2.0f * (normalized.w() * normalized.x() + normalized.y() * normalized.z()),
                           1.0f - 2.0f * (normalized.x() * normalized.x() + normalized.y() * normalized.y()));
        float pitch = asinf(2.0f * (normalized.w() * normalized.y() - normalized.z() * normalized.x()));
        float yaw = atan2f(2.0f * (normalized.w() * normalized.z() + normalized.x() * normalized.y()),
                           1.0f - 2.0f * (normalized.y() * normalized.y() + normalized.z() * normalized.z()));

        // Convert the floating point angles in radians to a scale from 0 to 20.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

		if (currentPose == myo::Pose::fist) {
			if (pitch_w < 5) {
				// Vibrate the Myo whenever we've detected that the user has made a fist pump.
				myo->vibrate(myo::Myo::VibrationMedium);
			}
		}
	}

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
        // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
        // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
        std::string poseString = currentPose.toString();

        // Output the current values
        std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
                  << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
                  << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']'
                  << '[' << poseString << std::string(16 - poseString.size(), ' ') << ']'
                  << std::flush;
    }

    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
};

int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {

    // First, we create a Hub. The Hub provides access to one or more Myos.
    myo::Hub hub;

	std::cout << "Bike signs." << std::endl;

    std::cout << "Attempting to find a Myo..." << std::endl;

    // Next, we try to find a Myo (any Myo) that's nearby and connect to it. waitForAnyMyo() takes a timeout
    // value in milliseconds. In this case we will try to find a Myo for 10 seconds, and if that fails, the function
    // will return a null pointer.
    myo::Myo* myo = hub.waitForAnyMyo(10000);

    // If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }

    // We've found a Myo, let's output its MAC address.
    std::cout << "Connected to " << std::hex << std::setfill('0') << std::setw(12) << myo->macAddress() << std::dec
              << "." << std::endl << std::endl;

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

    // Finally we enter our main loop.
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/20);
        // After processing events, we call the print() member function we defined above to print out the values we've
        // obtained from any events that have occurred.
        collector.print();
    }

    // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
