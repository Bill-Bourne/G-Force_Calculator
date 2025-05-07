#ifndef G_FORCE_CALC_H
#define G_FORCE_CALC_H

#include <cmath>

class GForceCalculator {
private:
    const float STATIC_THRESHOLD = 0.1f;
    float pitch = 0.0f, roll = 0.0f;
    //const float ALPHA = 0.96f;
    float lastGx = 0, lastGy = 0, lastGz = 1.0f;

public:
    float calculateGForce(float ax, float ay, float az) {
        return sqrt(ax*ax + ay*ay + az*az);
    }
    
    void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
        // initial orientation from accelerometer
        float accel_magnitude = calculateGForce(ax, ay, az);
        
        // update if acceleration is close to 1g
        if(fabs(accel_magnitude - 1.0f) < STATIC_THRESHOLD) {
            float accel_pitch = asin(ax / accel_magnitude);
            float accel_roll = asin(ay / (accel_magnitude * cos(accel_pitch)));
            
            // Low-pass filter for orientation
            pitch = 0.1f * accel_pitch + 0.9f * pitch;
            roll = 0.1f * accel_roll + 0.9f * roll;
            
            // Update gravity components
            lastGx = sin(pitch);
            lastGy = -sin(roll) * cos(pitch);
            lastGz = -cos(roll) * cos(pitch);
        }
    }
    
    void removeGravity(float& ax, float& ay, float& az) {
        float raw_magnitude = calculateGForce(ax, ay, az);
        
        // If almost stationary, zero out acceleration
        if(fabs(raw_magnitude - 1.0f) < STATIC_THRESHOLD) {
            ax = ay = az = 0.0f;
            return;
        }
        
        // Remove gravity components
        ax -= lastGx;
        ay -= lastGy;
        az -= lastGz;
    }
};

#endif