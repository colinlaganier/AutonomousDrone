
#include <cstdio>
#include <cstring>   //strlen
#include <cstdlib>
#include <cerrno>
#include <unistd.h>   //close
#include <arpa/inet.h>    //close
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <ctime>
#include <thread>
#include <atomic>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdlib.h>

#define TRUE   1
#define FALSE  0
#define PORT 8888
#define MESSAGE_BUFFER 128

struct thread_message{
    std::atomic_bool send_flag;
    char send_message[MESSAGE_BUFFER];
    std::atomic_bool init;
};

struct target_drone{
    std::string id;
    int client_index;
};

// Function Prototypes
int communication_thread(thread_message *message);
[[noreturn]] void command_line_thread(thread_message *message);
void parse_function(thread_message *message, int target, std::string function, std::string value);
int get_client_id(std::string target_id);

int main(int argc, char *argv[]){

    thread_message message;
    message.send_flag = false;
    message.init = false;

//    std::thread communication(communication_thread, &message);
//    std::thread command_line(command_line_thread, &message);
//
//    command_line.join();
//    communication.join();

    command_line_thread(&message);

    return 0;
}

int communication_thread(thread_message *command_message){
    while (!command_message->init)
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    int opt = TRUE;
    int master_socket, addrlen, new_socket, client_socket[30], max_clients = 3, activity, i, valread, sd;
    int max_sd;
    struct sockaddr_in address;

    char buffer[1025];  //data buffer of 1K

    //set of socket descriptors
    fd_set readfds;

    //a message
    char *message = "ECHO Daemon v1.0 \r\n";

    //initialise all client_socket[] to 0 so not checked
    for (i = 0; i < max_clients; i++) {
        client_socket[i] = 0;
    }

    //create a master socket
    if ((master_socket = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    //set master socket to allow multiple connections
    if (setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *) &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    //type of socket created
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    //bind the socket to localhost port 8888
    if (bind(master_socket, (struct sockaddr *) &address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    printf("Listener on port %d \n", PORT);

    //try to specify maximum of 3 pending connections for the master socket
    if (listen(master_socket, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    //accept the incoming connection
    addrlen = sizeof(address);
    puts("Waiting for connections ...");

    while (TRUE) {
        //clear the socket set
        FD_ZERO(&readfds);

        //add master socket to set
        FD_SET(master_socket, &readfds);
        max_sd = master_socket;

        //add child sockets to set
        for (i = 0; i < max_clients; i++) {
            //socket descriptor
            sd = client_socket[i];

            //if valid socket descriptor then add to read list
            if (sd > 0)
                FD_SET(sd, &readfds);

            //highest file descriptor number, need it for the select function
            if (sd > max_sd)
                max_sd = sd;
        }

        //wait for an activity on one of the sockets , timeout is NULL ,
        //so wait indefinitely
        activity = select(max_sd + 1, &readfds, nullptr, nullptr, nullptr);

        if ((activity < 0) && (errno != EINTR)) {
            printf("select error");
        }

        //If something happened on the master socket ,
        //then it's an incoming connection
        if (FD_ISSET(master_socket, &readfds)) {
            if ((new_socket = accept(master_socket,
                                     (struct sockaddr *) &address, (socklen_t *) &addrlen)) < 0) {
                perror("accept");
                exit(EXIT_FAILURE);
            }

            //inform user of socket number - used in send and receive commands
            printf("New connection , socket fd is %d , ip is : %s , port : %d\n", new_socket,
                   inet_ntoa(address.sin_addr), ntohs(address.sin_port));

            //send new connection greeting message
            if (send(new_socket, message, strlen(message), 0) != strlen(message)) {
                perror("send");
            }

            puts("Welcome message sent successfully");

            //add new socket to array of sockets
            for (i = 0; i < max_clients; i++) {
                //if position is empty
                if (client_socket[i] == 0) {
                    client_socket[i] = new_socket;
                    printf("Adding to list of sockets as %d\n", i);

                    break;
                }
            }
        }

        //else it's some IO operation on some other socket
        for (i = 0; i < max_clients; i++) {
            sd = client_socket[i];

            if (FD_ISSET(sd, &readfds)) {
                //Check if it was for closing , and also read the
                //incoming message
                if ((valread = read(sd, buffer, 1024)) == 0) {
                    //Somebody disconnected , get his details and print
                    getpeername(sd, (struct sockaddr *) &address, (socklen_t *) &addrlen);
                    printf("Host disconnected , ip %s , port %d \n",
                           inet_ntoa(address.sin_addr), ntohs(address.sin_port));

                    //Close the socket and mark as 0 in list for reuse
                    close(sd);
                    client_socket[i] = 0;
                }

                    //Echo back the message that came in
                else {
                    //set the string terminating NULL byte on the end
                    //of the data read
                    buffer[valread] = '\0';
                    printf("Received message\n");
                    printf(buffer);
//                    send(sd , buffer , strlen(buffer) , 0 );
                }
            }
        }
    }
}

[[noreturn]] void command_line_thread(thread_message *message){
    bool reading = true;

    while(reading){
        std::string str;
        const char* delimiter = ":";
//        getline(std::cin, str);
        str = "drone001:takeoff:2";
        if (str == "exit")
        {
            exit(1);
        }
        else
        {
            // Split values from string
            std::string target_id;
            std::string function;
            std::string value;
            size_t str_len = str.length();

            int splitter_count = 0;
            for (int i = 0; i < str.size(); i++)
                if (str[i] == ':') splitter_count++;

            size_t splitter_1 = str.find(delimiter,0);
            target_id = str.substr(0,splitter_1);

            std::cout << splitter_1 << std::endl;

            if (splitter_count > 1) {
                std::cout << "With value" << std::endl;
                size_t splitter_2 = str.find(delimiter, splitter_1,1);
                std::cout << splitter_2 << std::endl;
                function = str.substr(splitter_1 + 1, splitter_2 + 1);

                value = str.substr(splitter_1 + splitter_2 + 3);
            }
            else {
                std::cout << "No value" << std::endl;
                function = str.substr(splitter_1 + 1);
                value = "0";
            }

            std::cout << "target: " << target_id << ", function: " << function << ", value: " << value << '\n';

            int client_id = get_client_id(target_id);

            parse_function(message,client_id,function,value);

            exit(2);
        }
    }
}


void parse_function(thread_message *message, int target, std::string function, std::string value){
    if (function == "takeoff")
    {

    }
    else if (function == "change_mode")
    {
        message->send_message =
    }
}

int get_client_id(std::string target_id)
{
    return 0;
}