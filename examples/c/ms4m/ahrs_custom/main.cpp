#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <wiringPi.h>
#include <filesystem>

namespace fs = std::filesystem;

// Pines
#define BTN_START   23
#define BTN_CREATE  24
#define BTN_FINISH  25
#define LED_PIN     16

#define DEBOUNCE_TIME 100  // ms
#define LONG_PRESS_TIME 800 // ms
#define TIME_WAIT_TO_START 10000
#define TIME_WAIT_TO_CAPTURE_GYRO_BIAS 30 // s

enum State { START, STARTED, CREATED, FINISHED };

bool ledState = false;
bool fastBlink = false;

bool flagFinish = false;
bool flagWaitingToStart = false;

unsigned long lastReadyToStarted = 0;


std::string find_device(const std::string& keyword) {
    std::string base = "/dev/serial/by-id/";

    for (const auto& entry : fs::directory_iterator(base)) {
        std::string path = entry.path();
        if (path.find(keyword) != std::string::npos) {
            return path;
        }
    }
    return "";
}

// ================== TIEMPO ==================
unsigned long millis_() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

// ================== BOTÓN ==================
struct Button {
    int pin;
    bool stableState = HIGH;
    bool lastReading = HIGH;
    unsigned long lastDebounceTime = 0;

    unsigned long pressStartTime = 0;
    bool longPressTriggered = false;
};

bool readButton(Button &btn, unsigned long now, bool &longPress) {

    bool reading = digitalRead(btn.pin);
    bool shortPressEvent = false;
    longPress = false;

    if (reading != btn.lastReading) {
        btn.lastDebounceTime = now;
    }

    if ((now - btn.lastDebounceTime) > DEBOUNCE_TIME) {

        if (reading != btn.stableState) {
            btn.stableState = reading;

            // PRESIONADO
            if (btn.stableState == LOW) {
                btn.pressStartTime = now;
                btn.longPressTriggered = false;
            }
            // SOLTADO
            else {
                if (!btn.longPressTriggered) {
                    shortPressEvent = true;
                }
            }
        }

        // LONG PRESS
        if (btn.stableState == LOW &&
            !btn.longPressTriggered &&
            (now - btn.pressStartTime > LONG_PRESS_TIME)) {

            longPress = true;
            btn.longPressTriggered = true;
        }
    }

    btn.lastReading = reading;
    return shortPressEvent;
}

// ================== PROCESOS ==================
void run_program(const std::string& port,
                 const std::string& program_name,
                 int pipe_fd[2]) {

    close(pipe_fd[1]);
    dup2(pipe_fd[0], STDIN_FILENO);
    close(pipe_fd[0]);

    char* args[] = {
        const_cast<char*>(program_name.c_str()),
        const_cast<char*>(port.c_str()),
        nullptr
    };

    execvp(args[0], args);
    perror("Error execvp");
    exit(1);
}

// ================== MAIN ==================
int main() {

    wiringPiSetupGpio();

    pinMode(BTN_START, INPUT);
    pinMode(BTN_CREATE, INPUT);
    pinMode(BTN_FINISH, INPUT);

    pullUpDnControl(BTN_START, PUD_UP);
    pullUpDnControl(BTN_CREATE, PUD_UP);
    pullUpDnControl(BTN_FINISH, PUD_UP);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Botones
    Button btnS{BTN_START};
    Button btnC{BTN_CREATE};
    Button btnF{BTN_FINISH};

    // std::string com_port_ins = "/dev/ttyUSB0";
    // std::string com_port_gps = "/dev/ttyUSB1";

    std::string com_port_ins = find_device("FTDI");
    // std::string com_port_gps = find_device("Prolific");

    std::cout << "INS: " << com_port_ins << std::endl;
    // std::cout << "GPS: " << com_port_gps << std::endl;

    int pipe_ins[2];
    // int pipe_gps[2];

    pipe(pipe_ins);
    // pipe(pipe_gps);

    pid_t pid1 = fork();
    if (pid1 == 0) {
        // close(pipe_gps[0]); close(pipe_gps[1]);
        run_program(com_port_ins, "./7_series_ahrs_custom_example_c", pipe_ins);
    }

    // pid_t pid2 = fork();
    // if (pid2 == 0) {
    //     close(pipe_ins[0]); close(pipe_ins[1]);
    //     run_program(com_port_gps, "./recording_tramaGPS", pipe_gps);
    // }

    close(pipe_ins[0]);
    // close(pipe_gps[0]);

    sleep(TIME_WAIT_TO_CAPTURE_GYRO_BIAS);

    State currentState = START;
    std::cout << "MAIN : Press Button\n";

    unsigned long lastBlink = 0;

    while (true) {

        unsigned long now = millis_();

        // ===== lectura botones =====
        bool longPressS = false;
        bool longPressC = false;
        bool longPressF = false;

        bool flagS = readButton(btnS, now, longPressS);
        bool flagC = readButton(btnC, now, longPressC);
        bool flagF = readButton(btnF, now, longPressF);

        // ===== LONG PRESS START =====
        if (longPressS && currentState == STARTED) {
            fastBlink = !fastBlink;

            char cmd = 'p';
            write(pipe_ins[1], &cmd, 1);
            // write(pipe_gps[1], &cmd, 1);

            std::cout << "MAIN : LONG PRESS S -> TOGGLE BLINK\n";

            // evitar que entre como click corto
            flagS = false;
        }

        // ===== máquina de estados =====
        switch (currentState) {

            case START:
                digitalWrite(LED_PIN, HIGH);

                if (flagS) {
                    std::cout << "MAIN : STARTED\n";
                    currentState = STARTED;

                    char cmd = 's';
                    write(pipe_ins[1], &cmd, 1);
                    // write(pipe_gps[1], &cmd, 1);
                }
                break;

            case STARTED: {
                unsigned long blinkInterval = fastBlink ? 150 : 500;

                if (now - lastBlink > blinkInterval) {
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);
                    lastBlink = now;
                }

                if (flagF) {
                    std::cout << "MAIN : FINISHED\n";
                    currentState = FINISHED;

                    char cmd = 'q';
                    write(pipe_ins[1], &cmd, 1);
                    // write(pipe_gps[1], &cmd, 1);
                }
                else if (flagC) {
                    std::cout << "MAIN : CREATED\n";
                    currentState = CREATED;

                    char cmd = 'n';
                    write(pipe_ins[1], &cmd, 1);
                    // write(pipe_gps[1], &cmd, 1);

                    lastReadyToStarted = now;
                }
                break;
            }

            case CREATED:
                digitalWrite(LED_PIN, HIGH);
                
                fastBlink = false;

                if (!flagWaitingToStart && (now - lastReadyToStarted > TIME_WAIT_TO_START)) {
                    flagWaitingToStart = true;
                }

                if (flagF) {
                    std::cout << "MAIN : FINISHED\n";
                    currentState = FINISHED;

                    char cmd = 'q';
                    write(pipe_ins[1], &cmd, 1);
                    // write(pipe_gps[1], &cmd, 1);
                }
                else if (flagS && flagWaitingToStart) {
                    std::cout << "MAIN : STARTED\n";
                    currentState = STARTED;
                    flagWaitingToStart = false;

                    char cmd = 's';
                    write(pipe_ins[1], &cmd, 1);
                    // write(pipe_gps[1], &cmd, 1);
                }
                break;

            case FINISHED:
                digitalWrite(LED_PIN, LOW);
                flagFinish = true;
                break;
        }

        delay(10);

        if (flagFinish) break;
    }

    sleep(20);

    close(pipe_ins[1]);
    // close(pipe_gps[1]);

    waitpid(pid1, nullptr, 0);
    // waitpid(pid2, nullptr, 0);

    std::cout << "MAIN : Program finished\n";
    return 0;
}