#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>
#include <unistd.h>
#include <fcntl.h>

static struct termios oldt, newt;
int serial_descriptor;

void clean_and_exit(int code);

extern "C" void quit_signal_handler(int signum);
extern "C" void uart_signal_handler(int signum);

int main(int argc, char **argv) {
    // help variables
    int c;

    // tty settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Open serial port
    serial_descriptor = open("/dev/ttyTHS2", O_RDWR | O_NDELAY | O_NONBLOCK);
    if (serial_descriptor == -1) {
        printf("Could not open serial port on /dev/ttyTHS2!\n");
        clean_and_exit(-1);
    } else fcntl(serial_descriptor, F_SETFL, 0);

    // Configure serial port
    struct sigaction saio;
    saio.sa_handler = uart_signal_handler;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO, &saio, NULL);

    fcntl(serial_descriptor, F_SETFL, FNDELAY);         // Make fd wait
    fcntl(serial_descriptor, F_SETOWN, getpid());       // Allow to receive SIGIO
    fcntl(serial_descriptor, F_SETFL, O_ASYNC);       // Make fd asynchronous

    // UART settings
    struct termios termAttr;
    tcgetattr(serial_descriptor, &termAttr);
    termAttr.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    termAttr.c_cflag |= CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    termAttr.c_cflag |= CS8; // 8 bits per byte (most common)
    termAttr.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    termAttr.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    termAttr.c_lflag &= ~ICANON;
    termAttr.c_lflag &= ~ECHO; // Disable echo
    termAttr.c_lflag &= ~ECHOE; // Disable erasure
    termAttr.c_lflag &= ~ECHONL; // Disable new-line echo
    termAttr.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    termAttr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                          ICRNL); // Disable any special handling of received bytes

    termAttr.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    termAttr.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
// termAttr.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
// termAttr.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    termAttr.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    termAttr.c_cc[VMIN] = 8;

// Set in/out baud rate to be 9600
    cfsetispeed(&termAttr, B115200);
    cfsetospeed(&termAttr, B115200);

    /* Flush anything already in the serial buffer */
    tcflush(serial_descriptor, TCIFLUSH);
    tcflush(STDIN_FILENO, TCIFLUSH);

    //Kill signal handler
    signal(SIGINT, quit_signal_handler);

    float out[6] = {1, 2, 3, 4, 5, 6};
    // loop
    while ((c = getchar()) != 'q') {
        if (c == '0') {
            printf("Writing command 0...\n\r");
            write(serial_descriptor, out, 6 * sizeof(float));
        } else if (c == '1') {
            printf("Writing command 1...\n\r");
            write(serial_descriptor, out, 6 * sizeof(float));
        } else if (c == 'q') {
            printf("Writing command q...\n\r");
            write(serial_descriptor, out, 6 * sizeof(float));
        } else if (c > 0)
            printf("Press 1 or 0 or q.\n\r");
    }

    // cleanup
    printf("Will end now!\n");
    clean_and_exit(0);
}

void clean_and_exit(int code) {
    // close serial
    close(serial_descriptor);

    // tty reset settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    // exit
    std::exit(code);
}

void quit_signal_handler(int signum) {
    printf("Will end now!\n");
    clean_and_exit(0);
}

void uart_signal_handler(int signum) {
    float buffer[2];
//    memset(buffer, '\0', 1024);
    if ((read(serial_descriptor, buffer, 2 * sizeof(float))) > 0) {
        if (buffer[0] == 'q') {
            printf("Will end now!\n");
            clean_and_exit(0);
        } else printf("Got command = %f, %f\n", buffer[0], buffer[1]);
    }
}
