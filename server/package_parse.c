#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "cmd_manager.h"
#include "package_parse.h"
/*
0:bit7:数据包还是命令包
命令包格式 1
0:  bit6:是否存在控制数据
1:cmd_type
2:cmd h
3:cmd l
4～9:保留
10/11:控制数据长度H/L
...控制数据

数据包格式 0
1:

*/
#define PKG_TYPE_OFFSET 0
#define DATA_PKG_TYPE 0x00
#define CMD_PKG_TYPE 0X01
#define PKG_TYPE_BIT_OFFSET 0x07
#define CMD_PKG_CMD_DATA_EXIST_OFFSET 0
#define CMD_PKG_CMD_DATA_EXIST_BIT 0x06
#define CMD_PKG_CMD_TYPE_OFFSET 0x01
#define CMD_PKG_CMD_H_OFFSET 0x02
#define CMD_PKG_CMD_L_OFFSET 0x03
#define CMD_PKG_CMD_DATA_LEN_H_OFFSET 0x0a
#define CMD_PKG_CMD_DATA_LEN_L_OFFSET 0x0b
#define CMD_PKG_CMD_DATA_OFFSET 0x0c
int get_uint_bit(unsigned int data, int bit_offset)
{
    if ((bit_offset < 0) || (bit_offset > 31)) {
        printf("[%s]bit_offset:%d,invalid value.\n", __func__, bit_offset);
        return -1;
    }
    return ((data >> bit_offset) & 0x01);
}
static int get_package_type(char* pkg)
{
    return get_uint_bit(pkg[PKG_TYPE_OFFSET], PKG_TYPE_BIT_OFFSET);
}
struct cmd_pkg_t* pkg_parse(char* pkg)
{
    struct cmd_pkg_t* cmd_pkg;
    cmd_pkg = (struct cmd_pkg_t*)malloc(sizeof(struct cmd_pkg_t));
    if (!cmd_pkg) {
        printf("[%s]malloc fail.\n", __func__);
        return NULL;
    }
    if (CMD_PKG_TYPE == get_package_type(pkg)) {
        printf("[%s]parse cmd pkg.\n", __func__);
        cmd_pkg->cmd_type = pkg[CMD_PKG_CMD_TYPE_OFFSET];
        cmd_pkg->cmd = (pkg[CMD_PKG_CMD_H_OFFSET] << 8) | (pkg[CMD_PKG_CMD_L_OFFSET]);
        printf("[%s]cmd type:%d cmd:%d\n", __func__, cmd_pkg->cmd_type, cmd_pkg->cmd);
        if (get_uint_bit(pkg[CMD_PKG_CMD_DATA_EXIST_OFFSET], CMD_PKG_CMD_DATA_EXIST_BIT)) {
            cmd_pkg->data_len = (pkg[CMD_PKG_CMD_DATA_LEN_H_OFFSET] << 8) | (pkg[CMD_PKG_CMD_DATA_LEN_L_OFFSET]);
            printf("[%s]data len:%d\n", __func__, cmd_pkg->data_len);
            cmd_pkg->data = (char*)malloc(cmd_pkg->data_len);
            if (cmd_pkg->data == NULL) {
                free(cmd_pkg);
                return NULL;
            }
            memcpy(cmd_pkg->data, &pkg[CMD_PKG_CMD_DATA_OFFSET], cmd_pkg->data_len);
        }
        else {
            cmd_pkg->data_len = 0;
            cmd_pkg->data = NULL;
        }
    }
    else {
        printf("[%s]parse data pkg.\n", __func__);
    }
    return cmd_pkg;
}
void cmd_pkg_free(struct cmd_pkg_t* cpkg)
{
    if (cpkg->data_len) {
        free(cpkg->data);
    }
    free(cpkg);
}