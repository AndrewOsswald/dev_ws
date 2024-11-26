#include <iostream>
#include <pigpio.h>
#include <cmath>
#include <chrono>
#include <array>
#include <thread>
#include <iomanip>
#include <vector>

#define MULTIPLEXER_ADDR 0x70  // PCA9548A I2C address
#define ENCODER_ADDR 0x36     // AS5600 I2C address

#define MAX_ENCODER_VALUE 4096  // 12-bit encoder (0-4095)
#define NUM_WHEELS 4            // Number of wheels
#define DEG_PER_TICK (360.0 / MAX_ENCODER_VALUE)
#define RPM_SCALING_FACTOR (60.0 / 360.0)
#define SAMPLE_WINDOW_SIZE 20   // Number of samples to average

// Function to select the correct channel on the multiplexer
inline void selectChannel(int channel, int muxHandle) {
    char data = 1 << channel;
    i2cWriteByte(muxHandle, data);
}

// Function to read encoder value
inline int readEncoder(int i2cHandle) {
    char data[2];
    if (i2cReadDevice(i2cHandle, data, 2) < 0) {
        return -1;
    }
    return (data[0] << 8) | data[1];
}

// Function to calculate RPM
inline double calculateRPM(int deltaPosition, double deltaTime) {
    return (deltaPosition * DEG_PER_TICK / deltaTime) * RPM_SCALING_FACTOR;
}

int main() {
    // Initialize pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
        return -1;
    }

    // Open I2C connections
    int muxHandle = i2cOpen(1, MULTIPLEXER_ADDR, 0);
    int i2cHandle = i2cOpen(1, ENCODER_ADDR, 0);
    if (muxHandle < 0 || i2cHandle < 0) {
        std::cerr << "Failed to open I2C connections!" << std::endl;
        return -1;
    }

    std::array<int, NUM_WHEELS> previousPosition = {0};
    std::array<std::vector<double>, NUM_WHEELS> rpmSamples;
    std::array<size_t, NUM_WHEELS> sampleIndex = {0};
    auto previousTime = std::chrono::high_resolution_clock::now();

    // Initialize buffers
    for (auto& samples : rpmSamples) {
        samples.resize(SAMPLE_WINDOW_SIZE, 0.0);
    }

    while (true) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        double deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - previousTime).count() / 1e6;

        for (int wheel = 0; wheel < NUM_WHEELS; ++wheel) {
            selectChannel(wheel, muxHandle);

            int currentPosition = readEncoder(i2cHandle);
            if (currentPosition == -1) continue;

            // Calculate delta position
            int deltaPosition = currentPosition - previousPosition[wheel];
            if (deltaPosition > MAX_ENCODER_VALUE / 2) deltaPosition -= MAX_ENCODER_VALUE;
            if (deltaPosition < -MAX_ENCODER_VALUE / 2) deltaPosition += MAX_ENCODER_VALUE;

            // Calculate RPM
            double rpm = calculateRPM(deltaPosition, deltaTime);

            // Add RPM to the circular buffer
            rpmSamples[wheel][sampleIndex[wheel] % SAMPLE_WINDOW_SIZE] = rpm;
            sampleIndex[wheel]++;

            // Update previous position
            previousPosition[wheel] = currentPosition;
        }

        // Print averaged RPM values
        for (int wheel = 0; wheel < NUM_WHEELS; ++wheel) {
            double avgRPM = 0;
            for (double sample : rpmSamples[wheel]) {
                avgRPM += sample;
            }
            avgRPM /= SAMPLE_WINDOW_SIZE;

            std::cout << (wheel == 0 ? "Front_left  - " : (wheel == 1 ? "Front_right - " : (wheel == 2 ? "Back_left   - " : "Back_right  - ")));
            std::cout << "Average RPM: " << std::fixed << std::setprecision(2) << avgRPM << std::endl;
        }
        std::cout << "----------------------------------------" << std::endl;

        previousTime = currentTime;

        // Minimal sleep for very high sampling rate
        std::this_thread::sleep_for(std::chrono::microseconds(50)); // 1 ns sleep
    }

    i2cClose(i2cHandle);
    i2cClose(muxHandle);
    gpioTerminate();
    return 0;
}
