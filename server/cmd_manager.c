#include <stdio.h>
#include <malloc.h>
#include "cmd_manager.h"
#include "package_parse.h"
#include "server_control_cmd.h"
#include "raspberry_ctl.h"

/* 命令管理器 */
static struct cmd_manager_t cmd_manager;
/* get cmd manager in other file */
struct cmd_manager_t *get_cmd_manager(void) {
	return &cmd_manager;
}

/* 命令管理器初始化 */
int cmd_manager_init(void)
{
    cmd_manager.cmd_num = 0;
    cmd_manager.cmd_parse_list = NULL;

	add_server_control_cmds();
	add_led_module_cmds(NULL);

	cmd_manager.is_manager_init_flag = 1;
    return 0;
}
/* 释放管理器 */
int cmd_manager_release(void)
{
    struct cmd_node_t* p_cmd;
    struct cmd_node_t* p_tmp;
    p_cmd = cmd_manager.cmd_parse_list;
    printf("[%s]All cmd num:%d\n", __func__, cmd_manager.cmd_num);
    while (p_cmd != NULL) {
        printf("[%s]release cmd:%d %d\n", __func__, p_cmd->cmd_type, p_cmd->cmd);
        p_tmp = p_cmd;
        p_cmd = p_cmd->next;
        free(p_tmp);
    }
    cmd_manager.cmd_parse_list = NULL;
    cmd_manager.cmd_num = 0;
	cmd_manager.is_manager_init_flag = 0;
	return 0;
}
/* 检查命令是否已经存在 */
static int is_cmd_exist(struct cmd_node_t* cn)
{
    struct cmd_node_t* p_cmd;
    p_cmd = cmd_manager.cmd_parse_list;
    while (p_cmd != NULL) {
        /*  分配的变量添加2次或者命令类型及其命令码都相同 */
        if ((p_cmd == cn) || ((p_cmd->cmd_type == cn->cmd_type) && (p_cmd->cmd == cn->cmd))) {
            return 1;
        }
        p_cmd = p_cmd->next;
    }
    return 0;
}
/* 添加命令 */
int cmd_add(struct cmd_node_t* cn)
{
    if (cmd_manager.cmd_num == 0) {
        cmd_manager.cmd_parse_list = cn;
        cmd_manager.cmd_parse_list->next = NULL;
        cmd_manager.cmd_parse_list->prev = NULL;
        cmd_manager.cmd_num = 1;
    }
    else if (0 == is_cmd_exist(cn)) {
        cmd_manager.cmd_parse_list->prev = cn;
        cn->next = cmd_manager.cmd_parse_list;
        cn->prev = NULL;
        cn->cnt_in_list = cmd_manager.cmd_num;
        cmd_manager.cmd_parse_list = cn;
        cmd_manager.cmd_num++;
    }
    return 0;
}
int cmd_register(int cmd_type, int cmd, void* exec_func, void* print_func)
{
    struct cmd_node_t* new_node;
    new_node = (struct cmd_node_t*)malloc(sizeof(struct cmd_node_t));
    new_node->cmd_type = cmd_type;
    new_node->cmd = cmd;
    new_node->exec_cmd = (int (*)(void* cmd_data, unsigned int, void*))exec_func;
    new_node->cmd_print = print_func; /* 不转换也行 */
    if (cmd_manager.cmd_num == 0) {
        cmd_manager.cmd_parse_list = new_node;
        cmd_manager.cmd_parse_list->next = NULL;
        cmd_manager.cmd_parse_list->prev = NULL;
        cmd_manager.cmd_num = 1;
    }
    else if (0 == is_cmd_exist(new_node)) {
        cmd_manager.cmd_parse_list->prev = new_node;
        new_node->next = cmd_manager.cmd_parse_list;
        new_node->prev = NULL;
        new_node->cnt_in_list = cmd_manager.cmd_num;
        cmd_manager.cmd_parse_list = new_node;
        cmd_manager.cmd_num++;
    }
    return 0;
}

/* 执行指定命令 
 * cmd_type:命令类型
 * cmd:命令号
 * cmd_data:命令所需数据
 * data_len:数据长度，单位:字节
 * p_ret:命令返回值
 *
 * return 0:成功 !0:失败
 **/
int exec_cmd(int cmd_type, int cmd, void* cmd_data, unsigned int data_len, void* p_ret)
{
    struct cmd_node_t* p_cmd;
    int is_cmd_exec = 0;
    int cmd_ret = 0;
    p_cmd = cmd_manager.cmd_parse_list;
    while (p_cmd != NULL) {
        if ((p_cmd->cmd_type == cmd_type) && (p_cmd->cmd == cmd)) {
            if (p_cmd->exec_cmd != NULL) {
                cmd_ret = p_cmd->exec_cmd(cmd_data, data_len, p_ret);
                is_cmd_exec = 1;
                break;
            }
        }
        p_cmd = p_cmd->next;
    }
    if (is_cmd_exec == 0) {
        printf("[%s]cmd(%d,%d)not find\n", __func__, cmd_type, cmd);
    }
    return cmd_ret;
}
/* 执行指定命令包
 * p_cmd_pkg:命令包
 * p_ret:命令返回值
 *
 * return 0:成功 !0:失败
 **/
int exec_cmd_by_cmd_pkg(struct cmd_pkg_t* p_cmd_pkg, void* p_ret)
{
    return exec_cmd(p_cmd_pkg->cmd_type,
        p_cmd_pkg->cmd, p_cmd_pkg->data,
        p_cmd_pkg->data_len, p_ret);
}
int exec_cmd_by_pkg(char* pkg, void* p_ret)
{
    int ret;
    struct cmd_pkg_t* cpkg = pkg_parse(pkg);
    if (cpkg) {
        ret = exec_cmd_by_cmd_pkg(cpkg, p_ret);
        cmd_pkg_free(cpkg);
    }
    else {
        ret = -1;
        p_ret = NULL;
        printf("[%s]pkg parse fail.\n", __func__);
    }
    return ret;
}
/* 测试用例 */
#if 0
void cmd_1_0_print(struct cmd_node_t* cn)
{
    printf("cmd num:%d\n", cn->cnt_in_list);
    printf("cmd type:%d\ncmd:%d\n", cn->cmd_type, cn->cmd);
    printf("cmd desc:第一个命令\n\n");
}
int cmd_1_0_exec(void* pdata, unsigned int data_len, void* p_ret)
{
    int i;
    printf("exec data_len:%d\n", data_len);
    printf("data:");
    for (i = 0; i < data_len; i++) {
        printf("%d ", ((char *)pdata)[i]);
    }
        printf("\n");
    return 0;
}
struct cmd_node_t* c, *d;
char pkg[] = {
    0xc0,
    0x01,
    0x00, 0x01,
    0, 0, 0, 0, 0, 0,
    0, 0x4,
    1, 2, 3, 4
};
    struct cmd_pkg_t * cpkg;
int main(void)
{
      cpkg = pkg_parse(pkg);
    c = (struct cmd_node_t*)malloc(sizeof(struct cmd_node_t));
    d = (struct cmd_node_t*)malloc(sizeof(struct cmd_node_t));
    c->cmd_type = 1;
    c->cmd = 0;
    c->cmd_print = cmd_1_0_print;
    c->exec_cmd = cmd_1_0_exec;
    d->cmd_type = 1;
    d->cmd = 1;
    d->cmd_print = cmd_1_0_print;
    d->exec_cmd = cmd_1_0_exec;
    /* 第1步 初始化管理器 */
    cmd_manager_init();
    /* 第2步 添加命令 */
    cmd_add(c);
    cmd_add(d);
    cmd_register(4, 4, cmd_1_0_exec, cmd_1_0_print);
    //ls_all_cmd(NULL);
    //exec_cmd(0, 0, NULL, 0, NULL);
    //exec_cmd(1, 0, NULL, 0, NULL);
    if (cpkg) {
        exec_cmd_by_cmd_pkg(cpkg, NULL);
            pkg_free(cpkg);
    }

    /* 程序退出必须释放空间 */
    printf("\n\n");
    cmd_manager_release();
}
#endif
