// Adapted from https://www.pololu.com/docs/0J73/15.5
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>

#define JPEG_BUF_SIZE 5128
#define MAGIC_HEADER_SIZE 6
#define min(a, b) a < b ? a : b
 
// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
int open_serial_port(const char * device, uint32_t baud_rate)
{
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
    return -1;
  }
 
  // Flush away any bytes previously read or written.
  int result = tcflush(fd, TCIOFLUSH);
  if (result)
  {
    perror("tcflush failed");  // just a warning, not a fatal error
  }
 
  // Get the current configuration of the serial port.
  struct termios options;
  result = tcgetattr(fd, &options);
  if (result)
  {
    perror("tcgetattr failed");
    close(fd);
    return -1;
  }
 
  // Turn off any options that might interfere with our ability to send and
  // receive raw binary bytes.
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
 
  // Set up timeouts: Calls to read() will return as soon as there is
  // at least one byte available or when 100 ms has passed.
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = 0;
 
  // This code only supports certain standard baud rates. Supporting
  // non-standard baud rates should be possible but takes more work.
  switch (baud_rate)
  {
  case 4800:   cfsetospeed(&options, B4800);   break;
  case 9600:   cfsetospeed(&options, B9600);   break;
  case 19200:  cfsetospeed(&options, B19200);  break;
  case 38400:  cfsetospeed(&options, B38400);  break;
  case 115200: cfsetospeed(&options, B115200); break;
  default:
    fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
      baud_rate);
    cfsetospeed(&options, B9600);
    break;
  }
  cfsetispeed(&options, cfgetospeed(&options));
 
  result = tcsetattr(fd, TCSANOW, &options);
  if (result)
  {
    perror("tcsetattr failed");
    close(fd);
    return -1;
  }
 
  return fd;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int write_port(int fd, uint8_t * buffer, size_t size)
{
  ssize_t result = write(fd, buffer, size);
  if (result != (ssize_t)size)
  {
    perror("failed to write to port");
    return -1;
  }
  return 0;
}

ssize_t read_port(int fd, uint8_t * buffer, size_t size, bool wait)
{
  size_t received = 0;
  while (received < size)
  {
    ssize_t r = read(fd, buffer + received, size - received);
    if (r < 0) {
      perror("failed to read from port");
      return -1;
    } else if (r == 0) {
      // Timeout
      if (!wait || received > 0) break;
    }
    received += r;
  }
  return received;
}

int main()
{
  // Linux USB example:          "/dev/ttyACM0"  (see also: /dev/serial/by-id)
  // macOS USB example:          "/dev/cu.usbmodem001234562"
  // Cygwin example:             "/dev/ttyS7"
  const char * device = "/dev/cu.usbmodem1442302";
 
  // Choose the baud rate (bits per second).
  uint32_t baud_rate = 9600;
 
  // Open serial port
  int fd = open_serial_port(device, baud_rate);
  if (fd < 0) { return 1; }
  
  // Read header
  uint8_t header_buf[MAGIC_HEADER_SIZE];
  ssize_t read_bytes;
  printf("Awaiting header...\n");
  read_bytes = read_port(fd, header_buf, MAGIC_HEADER_SIZE, true);
  if (read_bytes == -1) {
    fprintf(stderr, "failed to read header\n");
  } else if (read_bytes < MAGIC_HEADER_SIZE) {
    fprintf(stderr, "incomplete header\n");
  }
  printf("Header received\n");

  // Verify header
  uint8_t expected_header[] = {0x75, 0x03, 0x30, 0x75, 0x03, 0x30};
  for (int i = 0; i < sizeof(expected_header); i++) {
    if (expected_header[i] != header_buf[i]) {
      fprintf(stderr,
              "Header mismatch at byte %d: found 0x%02x but expected 0x%02x\n",
              i, header_buf[i], expected_header[i]);
    }
  }

  // Read image size
  uint8_t len_msbs = 0;
  uint8_t len_lsbs = 0;
  read_bytes = read_port(fd, &len_msbs, 1, false);
  if (read_bytes < 0) {
    fprintf(stderr, "failed to read msbs");
    return 1;
  } else if (read_bytes == 0) {
    fprintf(stderr, "timeout reading len msbs\n");
    return 1;
  }
  read_bytes = read_port(fd, &len_lsbs, 1, false);
  if (read_bytes < 0) {
    fprintf(stderr, "failed to read lsbs");
    return 1;
  } else if (read_bytes == 0) {
    fprintf(stderr, "timeout reading len lsbs\n");
    return 1;
  }
  uint16_t len = (len_msbs << 8) | len_lsbs;

  printf("Received reported image size of %i bytes\n", len);

  // Read image
  uint8_t *buf = malloc(len);
  read_bytes = read_port(fd, buf, len, false);

  if (read_bytes < -1) {
    fprintf(stderr, "failed to read image bytes\n");
  } else if (read_bytes < len) {
    fprintf(stderr, "Read only %ld of %d expected bytes\n", read_bytes, len);
  }

  // Write to file
  FILE *f = fopen("cam_data.jpg", "w+");
  if (f == NULL) {
    perror("fopen");
    return 1;
  }
  size_t bytes_to_write = min(read_bytes, len);
  size_t written = fwrite(buf, 1, bytes_to_write, f);
  if (written < bytes_to_write) {
    fprintf(stderr, "Wrote only %li of %li bytes to file\n", written,
              bytes_to_write);
  }
  int res;
  res = fclose(f);
  if (res < 0) {
    perror("(output file) fclose");
  }
  free(buf);
  res = close(fd);
  if (res < 0) {
    perror("(serial port) close");
  }
  printf("Photo written to disk\n");
  return 0;
}
