////
//// Created by ColinLaganier on 25/04/2022.
////
//#include <arpa/inet.h>
//#include <stdio.h>
//#include <string.h>
//#include <sys/socket.h>
//#include <unistd.h>
//#define PORT 8888
//#define TRUE 1
//#define FALSE 0
//int main(int argc, char const* argv[])
//{
//    int sock = 0, valread;
//    struct sockaddr_in serv_addr;
//    char* hello = "Hello from client";
//    char buffer[1024] = { 0 };
//    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
//        printf("\n Socket creation error \n");
//        return -1;
//    }
//
//    serv_addr.sin_family = AF_INET;
//    serv_addr.sin_port = htons(PORT);
//
//    // Convert IPv4 and IPv6 addresses from text to binary
//    if (inet_pton(AF_INET, "192.168.0.21", &serv_addr.sin_addr)<= 0) {
//        printf("\nInvalid address/ Address not supported \n");
//        return -1;
//    }
//
//    if (connect(sock, (struct sockaddr*)&serv_addr,
//                sizeof(serv_addr))
//        < 0) {
//        printf("\nConnection Failed \n");
//        return -1;
//    }
//    send(sock, hello, strlen(hello), 0);
//    printf("Hello message sent\n");
//    while(TRUE)
//    {
//        valread = read(sock, buffer, 1024);
//    }
//    printf("%s\n", buffer);
//    return 0;
//}
