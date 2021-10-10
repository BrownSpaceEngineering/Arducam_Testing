/**
* @author SebastianMartino
* Test client side application to connect to Arducam_demo
* 
* Modified to also send appropriate commands to get server to send data.
*/

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 2002
#define PI_IP "172.18.129.147"
#define BUFFER_SIZE (1024 * 1024 * 5)

int main(int argc, char const *argv[])
{
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char *hello = "Hello from client";
    char buffer[BUFFER_SIZE];
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    printf("Connecting to %s\n", PI_IP);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, PI_IP, &serv_addr.sin_addr)<=0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    printf("\taddress valid\n");

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }

    printf("Connection Made\n");

    // Very janky setup
    // Quality level can be between 1 and 8 -- see server source code
    char in[] = "ql=4";
    write(sock, in, 5);
    char in2[] = "stream";
    write(sock, in2, 7);
    printf("reading\n");

    // Write some amount of image stream data to a file to examine
    // NOTE: this will include the HTTP 200 response and "123" string at the start of the buffer
    valread = read(sock , buffer, BUFFER_SIZE);
    FILE *f = fopen("test_data", "w+");
    fwrite(buffer, 1, BUFFER_SIZE, f);
    fclose(f);
    return 0;
}
