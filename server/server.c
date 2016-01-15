#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string.h>
#include <sys/wait.h>
#include "cmd_manager.h"
#include "package_parse.h"
#define SERVER_PORT 3333
_Bool thread_flag = 1; // c not support bool

void shut_down_server(void) {
	thread_flag = 0;
}

/* 单线程服务器程序 */
int main(void)
{
    int serverSockFd;
    struct sockaddr_in local_addr;
    char buf[1024];
    
    /* cmd manager init */
    cmd_manager_init();
	
        //cmd_register(1, 1, cmd_1_0_exec, cmd_1_0_print);
    // 定义客户端的socket地址结构
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    /* init addr */
    bzero(&local_addr, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htons(INADDR_ANY);
    local_addr.sin_port = htons(SERVER_PORT);
    /* create socked */
    serverSockFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSockFd == -1) {
        printf("Err,create socket fail.\n");
        exit(1);
    }
    // 绑定socket和socket地址结构
    if (-1 == (bind(serverSockFd, (struct sockaddr*)&local_addr, sizeof(local_addr)))) {
        printf("Server Bind Failed:");
        exit(1);
    }
    // socket监听
    if (-1 == (listen(serverSockFd, 1))) {
        printf("Server Listen Failed:");
        exit(1);
    }
    while (thread_flag) {
        printf("[%s]wait accept...\n", __func__);
        // accept函数会把连接到的客户端信息写到client_addr中
        int newSockFd = accept(serverSockFd, (struct sockaddr*)&clientAddr, &clientAddrLen);
        if (newSockFd < 0) {
            printf("Server Accept Failed:");
            break;
        }
        printf("INFO_start recieve\n");
        if (recv(newSockFd, buf, 1024, 0) < 0) {
            printf("Server Recieve Data Failed:");
            break;
        }
        //printf("get data:%d %d %d\n", buf[0], buf[1], buf[2]);
        struct cmd_pkg_t* cpkg = pkg_parse(buf);
        if (cpkg) {
        exec_cmd_by_cmd_pkg(cpkg, NULL);
        cmd_pkg_free(cpkg);
        }
    }
    printf("[%s]cmd server exit.\n", __func__);
   
   cmd_manager_release();
	return 0;
}
