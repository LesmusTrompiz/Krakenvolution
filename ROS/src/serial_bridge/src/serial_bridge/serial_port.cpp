#include "serial_bridge/serial_port.hpp" 
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream> // -- DEBUG --

// RMI implementation

// ================================================================================
// Low level, serial port configuration
// ================================================================================

struct termios tty;

#define BAUDRATE B115200

int open_serial(const char* t_port) {
    // Serial port configuration
    // Control modes
    tty.c_cflag &= ~PARENB; // Remove parity
    tty.c_cflag &= ~CSIZE; // Set bits per byte
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS; // Disable flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on read and ignore ctrl lines
    // Local modes
    tty.c_lflag &= ~ICANON; // Dsiable cannonical mode
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG; // Disable signal chars
    // Input modes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Dsiable flow control (xon xoff)
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    // Output modes
    tty.c_oflag &= ~OPOST; // Dsiable interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent CR on newline

    // Configure blocking mode with unlimited timeout
    tty.c_cc[VTIME] = 10; // 1s timeout
    tty.c_cc[VMIN] = 1;

    // Configure baudrate 
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    // Try to open serial port
    int serial_port = open(t_port, O_RDWR);
    if (serial_port > 0) {
        // Set tty attributes
        tcsetattr(serial_port, TCSANOW, &tty);
    }
    return serial_port; // Returns > 0 if open and configured correctly
}
