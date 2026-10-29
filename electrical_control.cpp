// l298n_servo_test.cpp
// Interactive hardware test for an L298N motor driver and a servo on a Raspberry Pi.
//
// Pin map (BCM numbering):
//   L298N IN1 = GPIO 2   (Motor A)
//   L298N IN2 = GPIO 3   (Motor A)
//   L298N IN3 = GPIO 15  (Motor B)
//   L298N IN4 = GPIO 14  (Motor B)
//   Servo signal = GPIO 12
//
// Build:
//   sudo apt install pigpio libpigpio-dev
//   g++ -Wall -O2 -o l298n_servo_test l298n_servo_test.cpp -lpigpio -lrt -pthread
//
// Run (pigpio needs root):
//   sudo ./l298n_servo_test
//
// Note: this uses the pigpio library, which works on Pi 4 and earlier.
// It does NOT work on the Raspberry Pi 5.

#include <pigpio.h>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <iostream>
#include <string>

// ---------------------------------------------------------------- Pins
constexpr unsigned PIN_IN1 = 2;
constexpr unsigned PIN_IN2 = 3;
constexpr unsigned PIN_IN3 = 15;
constexpr unsigned PIN_IN4 = 14;
constexpr unsigned PIN_SERVO = 12;

// ---------------------------------------------------------------- Servo limits
// Typical hobby servo: 500 us = 0 deg, 2500 us = 180 deg.
// If your servo buzzes or hits its end stops, narrow these (e.g. 1000 / 2000).
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

static std::atomic<bool> g_running{true};

// ---------------------------------------------------------------- Motor helpers
enum class Dir { Stop, Forward, Reverse, Brake };

static void setMotor(unsigned inA, unsigned inB, Dir d) {
    switch (d) {
        case Dir::Forward: gpioWrite(inA, 1); gpioWrite(inB, 0); break;
        case Dir::Reverse: gpioWrite(inA, 0); gpioWrite(inB, 1); break;
        case Dir::Brake:   gpioWrite(inA, 1); gpioWrite(inB, 1); break;
        case Dir::Stop:
        default:           gpioWrite(inA, 0); gpioWrite(inB, 0); break;
    }
}

static void motorA(Dir d) { setMotor(PIN_IN1, PIN_IN2, d); }
static void motorB(Dir d) { setMotor(PIN_IN3, PIN_IN4, d); }

static void allMotorsStop() {
    motorA(Dir::Stop);
    motorB(Dir::Stop);
}

// ---------------------------------------------------------------- Servo helpers
static int angleToPulse(int angle) {
    angle = std::clamp(angle, 0, 180);
    return SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
}

static void servoAngle(int angle) {
    gpioServo(PIN_SERVO, angleToPulse(angle));
}

static void servoOff() {
    gpioServo(PIN_SERVO, 0);  // 0 stops the pulses
}

static void servoSweep() {
    std::cout << "Sweeping 0 -> 180 -> 0 ...\n";
    for (int a = 0; a <= 180 && g_running; a += 2) {
        servoAngle(a);
        gpioDelay(20000);  // 20 ms
    }
    for (int a = 180; a >= 0 && g_running; a -= 2) {
        servoAngle(a);
        gpioDelay(20000);
    }
    std::cout << "Sweep done (servo still holding at 0).\n";
}

// ---------------------------------------------------------------- Menu
static void printMenu() {
    std::cout <<
        "\n================ TEST MENU ================\n"
        " L298N pin states (IN1/IN2/IN3/IN4 = GPIO 2/3/15/14)\n"
        "  Motor A (OUT1/OUT2):\n"
        "   1) Forward   (IN1=H IN2=L)\n"
        "   2) Reverse   (IN1=L IN2=H)\n"
        "   3) Stop      (IN1=L IN2=L)\n"
        "   4) Brake     (IN1=H IN2=H)\n"
        "  Motor B (OUT3/OUT4):\n"
        "   5) Forward   (IN3=H IN4=L)\n"
        "   6) Reverse   (IN3=L IN4=H)\n"
        "   7) Stop      (IN3=L IN4=L)\n"
        "   8) Brake     (IN3=H IN4=H)\n"
        "  9) Stop ALL motors\n"
        "\n Servo (GPIO 12)\n"
        "   a) Sweep 0-180-0\n"
        "   c) Center (90 deg)\n"
        "   s) Set angle (0-180)\n"
        "   o) Servo off (stop pulses)\n"
        "\n   p) Print current pin config\n"
        "   q) Quit\n"
        "===========================================\n"
        "> ";
}

static void printPins() {
    std::cout << "\nPin states:\n"
              << "  IN1 (GPIO " << PIN_IN1 << ") = " << gpioRead(PIN_IN1) << "\n"
              << "  IN2 (GPIO " << PIN_IN2 << ") = " << gpioRead(PIN_IN2) << "\n"
              << "  IN3 (GPIO " << PIN_IN3 << ") = " << gpioRead(PIN_IN3) << "\n"
              << "  IN4 (GPIO " << PIN_IN4 << ") = " << gpioRead(PIN_IN4) << "\n"
              << "  Servo pulse width = " << gpioGetServoPulsewidth(PIN_SERVO)
              << " us (0 = off)\n";
}

static void onSignal(int) { g_running = false; }

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed. Are you running with sudo?\n"
                     "(Also: pigpio does not support the Raspberry Pi 5.)\n";
        return 1;
    }

    // Use our own handlers so we can clean up on Ctrl+C.
    gpioSetSignalFunc(SIGINT, nullptr);
    gpioSetSignalFunc(SIGTERM, nullptr);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    for (unsigned pin : {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4}) {
        gpioSetMode(pin, PI_OUTPUT);
        gpioWrite(pin, 0);
    }
    gpioSetMode(PIN_SERVO, PI_OUTPUT);
    servoOff();

    std::cout << "L298N + servo test. Motors start STOPPED, servo starts OFF.\n";

    std::string line;
    while (g_running) {
        printMenu();
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;

        switch (std::tolower(line[0])) {
            case '1': motorA(Dir::Forward); std::cout << "Motor A forward\n"; break;
            case '2': motorA(Dir::Reverse); std::cout << "Motor A reverse\n"; break;
            case '3': motorA(Dir::Stop);    std::cout << "Motor A stopped\n"; break;
            case '4': motorA(Dir::Brake);   std::cout << "Motor A brake\n";   break;
            case '5': motorB(Dir::Forward); std::cout << "Motor B forward\n"; break;
            case '6': motorB(Dir::Reverse); std::cout << "Motor B reverse\n"; break;
            case '7': motorB(Dir::Stop);    std::cout << "Motor B stopped\n"; break;
            case '8': motorB(Dir::Brake);   std::cout << "Motor B brake\n";   break;
            case '9': allMotorsStop();      std::cout << "All motors stopped\n"; break;

            case 'a': servoSweep(); break;
            case 'c': servoAngle(90); std::cout << "Servo centered\n"; break;
            case 's': {
                std::cout << "Angle (0-180): ";
                std::string a;
                if (!std::getline(std::cin, a)) break;
                try {
                    int angle = std::stoi(a);
                    servoAngle(angle);
                    std::cout << "Servo -> " << std::clamp(angle, 0, 180)
                              << " deg (" << angleToPulse(angle) << " us)\n";
                } catch (...) {
                    std::cout << "Not a number.\n";
                }
                break;
            }
            case 'o': servoOff(); std::cout << "Servo pulses off\n"; break;

            case 'p': printPins(); break;
            case 'q': g_running = false; break;
            default:  std::cout << "Unknown option.\n"; break;
        }
    }

    std::cout << "\nCleaning up...\n";
    allMotorsStop();
    servoOff();
    gpioTerminate();
    return 0;
}
