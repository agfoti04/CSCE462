// l298n_servo_test.cpp
// Interactive hardware test for an L298N motor driver and a servo on a Raspberry Pi.
// Uses the lgpio library (works on Pi 3/4/5 and current Raspberry Pi OS).
//
// Pin map (BCM numbering):
//   L298N IN1 = GPIO 2   (Motor A)
//   L298N IN2 = GPIO 3   (Motor A)
//   L298N IN3 = GPIO 15  (Motor B)
//   L298N IN4 = GPIO 14  (Motor B)
//   Servo signal = GPIO 12
//
// Install:
//   sudo apt install liblgpio-dev
//
// Build:
//   g++ -Wall -O2 -o l298n_servo_test l298n_servo_test.cpp -llgpio -lrt -pthread
//
// Run:
//   ./l298n_servo_test
//   (if you get a permission error: sudo ./l298n_servo_test)

#include <lgpio.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <csignal>
#include <iostream>
#include <string>

// ---------------------------------------------------------------- Pins
constexpr int PIN_IN1 = 2;
constexpr int PIN_IN2 = 3;
constexpr int PIN_IN3 = 15;
constexpr int PIN_IN4 = 14;
constexpr int PIN_SERVO = 12;

// ---------------------------------------------------------------- Servo limits
// Typical hobby servo: 500 us = 0 deg, 2500 us = 180 deg.
// If your servo buzzes or hits its end stops, narrow these (e.g. 1000 / 2000).
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int SERVO_HZ = 50;

static std::atomic<bool> g_running{true};
static int g_h = -1;          // lgpio chip handle
static int g_servoUs = 0;     // last commanded pulse width (0 = off)
static int g_pinState[4] = {0, 0, 0, 0};  // IN1..IN4 as we last set them

// ---------------------------------------------------------------- Motor helpers
enum class Dir { Stop, Forward, Reverse, Brake };

static void writePin(int idx, int pin, int level) {
    lgGpioWrite(g_h, pin, level);
    g_pinState[idx] = level;
}

static void setMotor(int idxA, int pinA, int pinB, Dir d) {
    int a = 0, b = 0;
    switch (d) {
        case Dir::Forward: a = 1; b = 0; break;
        case Dir::Reverse: a = 0; b = 1; break;
        case Dir::Brake:   a = 1; b = 1; break;
        case Dir::Stop:
        default:           a = 0; b = 0; break;
    }
    writePin(idxA, pinA, a);
    writePin(idxA + 1, pinB, b);
}

static void motorA(Dir d) { setMotor(0, PIN_IN1, PIN_IN2, d); }
static void motorB(Dir d) { setMotor(2, PIN_IN3, PIN_IN4, d); }

static void allMotorsStop() {
    motorA(Dir::Stop);
    motorB(Dir::Stop);
}

// ---------------------------------------------------------------- Servo helpers
static int angleToPulse(int angle) {
    angle = std::clamp(angle, 0, 180);
    return SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
}

static void servoPulse(int us) {
    // servoCycles = 0 means "run until changed".
    int rc = lgTxServo(g_h, PIN_SERVO, us, SERVO_HZ, 0, 0);
    // Asking lgpio to stop pulses on a pin that has none running returns
    // LG_BAD_PWM_MICROS (-86). That just means "already off", so it is not an error.
    if (us == 0 && rc == LG_BAD_PWM_MICROS) rc = 0;
    if (rc < 0) {
        std::cerr << "lgTxServo failed (error " << rc << ": " << lguErrorText(rc) << ")\n";
        return;
    }
    g_servoUs = us;
}

// Diagnostic: drive GPIO 12 as a plain steady HIGH or LOW so a multimeter can see it.
static void servoPinSteady(int level) {
    servoPulse(0);  // stop any servo pulses first
    int rc = lgGpioWrite(g_h, PIN_SERVO, level);
    if (rc < 0) {
        std::cerr << "lgGpioWrite failed (error " << rc << ": " << lguErrorText(rc) << ")\n";
        return;
    }
    std::cout << "GPIO " << PIN_SERVO << " (physical pin 32) held "
              << (level ? "HIGH (expect ~3.3V)" : "LOW (expect ~0V)") << "\n";
}

static void servoAngle(int angle) { servoPulse(angleToPulse(angle)); }
static void servoOff()            { servoPulse(0); }

static void servoSweep() {
    std::cout << "Sweeping 0 -> 180 -> 0 ...\n";
    for (int a = 0; a <= 180 && g_running; a += 2) {
        servoAngle(a);
        lguSleep(0.02);
    }
    for (int a = 180; a >= 0 && g_running; a -= 2) {
        servoAngle(a);
        lguSleep(0.02);
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
        "   h) DIAGNOSTIC: hold GPIO 12 steady HIGH (meter test)\n"
        "   l) DIAGNOSTIC: hold GPIO 12 steady LOW\n"
        "\n   p) Print current pin states\n"
        "   q) Quit\n"
        "===========================================\n"
        "> ";
}

static void printPins() {
    std::cout << "\nPin states (as last commanded):\n"
              << "  IN1 (GPIO " << PIN_IN1 << ") = " << g_pinState[0] << "\n"
              << "  IN2 (GPIO " << PIN_IN2 << ") = " << g_pinState[1] << "\n"
              << "  IN3 (GPIO " << PIN_IN3 << ") = " << g_pinState[2] << "\n"
              << "  IN4 (GPIO " << PIN_IN4 << ") = " << g_pinState[3] << "\n"
              << "  Servo pulse width = " << g_servoUs << " us (0 = off)\n";
}

static void onSignal(int) { g_running = false; }

// Pi 5 exposes the header on gpiochip4; older Pis use gpiochip0.
static int openChip() {
    int h = lgGpiochipOpen(4);
    if (h >= 0) return h;
    return lgGpiochipOpen(0);
}

int main() {
    g_h = openChip();
    if (g_h < 0) {
        std::cerr << "Could not open GPIO chip (error " << g_h << ").\n"
                     "Try running with sudo, and make sure liblgpio is installed.\n";
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const int motorPins[4] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
    for (int pin : motorPins) {
        int rc = lgGpioClaimOutput(g_h, 0, pin, 0);
        if (rc < 0) {
            std::cerr << "Failed to claim GPIO " << pin << " (error " << rc << ").\n"
                         "GPIO 2/3 are I2C pins: make sure I2C is disabled in raspi-config.\n";
            lgGpiochipClose(g_h);
            return 1;
        }
    }
    int rc = lgGpioClaimOutput(g_h, 0, PIN_SERVO, 0);
    if (rc < 0) {
        std::cerr << "Failed to claim servo GPIO " << PIN_SERVO << " (error " << rc << ").\n";
        lgGpiochipClose(g_h);
        return 1;
    }

    std::cout << "L298N + servo test. Motors start STOPPED, servo starts OFF.\n";

    std::string line;
    while (g_running) {
        printMenu();
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;

        switch (std::tolower(static_cast<unsigned char>(line[0]))) {
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
            case 'h': servoPinSteady(1); break;
            case 'l': servoPinSteady(0); break;

            case 'p': printPins(); break;
            case 'q': g_running = false; break;
            default:  std::cout << "Unknown option.\n"; break;
        }
    }

    std::cout << "\nCleaning up...\n";
    allMotorsStop();
    servoOff();
    for (int pin : motorPins) lgGpioFree(g_h, pin);
    lgGpioFree(g_h, PIN_SERVO);
    lgGpiochipClose(g_h);
    return 0;
}
