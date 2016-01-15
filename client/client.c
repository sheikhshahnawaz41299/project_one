#include <netinet/in.h>// sockaddr_in
#include <sys/types.h> // socket
#include <sys/socket.h>// socket
#include <stdio.h>     // printf
#include <stdlib.h>    // exit
#include <string.h>    // bzero
#include <arpa/inet.h> // net
#include <pthread.h>   //线程
#define SERVER_PORT 3333
#define SERVER_IP "192.168.1.50"
    
/*
char pkg[] = {
    0xc0,
    0x01,
    0x00, 0x01,
    0, 0, 0, 0, 0, 0,
    0, 0x4,
    1, 2, 3, 4
};
*/
char *pkg;
/* after send cmd,need to free pkg 
 * make data cmd pkg,just client use
 */
char* cmd_pkg_make(char pkg_type, unsigned char cmd_type,
    unsigned int cmd, unsigned int data_len, char* data)
{
	unsigned int pkg_len;

    if(data == NULL) {
        data_len = 0;
    }

	if(0 == data_len) {
		pkg_len = 10;		
	} else {
		pkg_len = 12 + data_len; 
	}

	char* pkg = (char*)malloc(pkg_len);
    
	if(!pkg) {
		printf("[%s]malloc fail.\n", __func__);
		return NULL;
	}

	bzero(pkg, pkg_len);
	
	pkg[0] = 0x80;			//pkg type
	if(0 != data_len) {	
		pkg[0] |= 0x40;		//data exit?
	}
	
	pkg[1] = cmd_type;		//cmd type
	pkg[2] = (cmd>>8) & 0xff;	//cmd H
	pkg[3] = cmd & 0xff;		//cmd L

	if(0 != data_len) {
		pkg[10] = (data_len>>8) & 0xff;	//data len H
		pkg[11] = data_len & 0xff;		//data len L
		memcpy(&pkg[12], data, data_len);	//copy cmd data to pkg
		}

	return pkg;
}

int main()
{
    // 声明并初始化一个客户端的socket地址结构
    struct sockaddr_in client_addr;
    bzero(&client_addr, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = htons(INADDR_ANY);
    client_addr.sin_port = htons(0);
    // 创建socket，若成功，返回socket描述符
    int client_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket_fd < 0) {
        printf("Create Socket Failed:");
        exit(1);
    }
    // 绑定客户端的socket和客户端的socket地址结构 非必需
    if (-1 == (bind(client_socket_fd, (struct sockaddr*)&client_addr, sizeof(client_addr)))) {
        printf("Client Bind Failed:");
        exit(1);
    }
    // 声明一个服务器端的socket地址结构，并用服务器那边的IP地址及端口对其进行初始化，用于后面的连接
    struct sockaddr_in server_addr;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) == 0) {
        printf("Server IP Address Error:");
        exit(1);
    }
    server_addr.sin_port = htons(SERVER_PORT);
    socklen_t server_addr_length = sizeof(server_addr);
    // 向服务器发起连接，连接成功后client_socket_fd代表了客户端和服务器的一个socket连接
    if (connect(client_socket_fd, (struct sockaddr*)&server_addr, server_addr_length) < 0) {
        printf("Can Not Connect To Server IP:");
        exit(0);
    }

	pkg = cmd_pkg_make(1, 0, 0, 2, "hh");
	
    if(send(client_socket_fd, pkg, 14, 0) < 0) { 
    	printf("[%s]send cmd fail.", __func__); 
    	exit(1); 
    } 
	free(pkg);
	
    printf("[%s]client send done.\n", __func__);
    return 0;
}