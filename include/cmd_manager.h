#ifndef _CMD_MANAGER_H_
#define _CMD_MANAGER_H_
#include "package_parse.h"  
/* 宏定义 */
#define CMD_MANAGER_CTL_TYPE 0x00
#define CM_SHOW_ALL 0x00  //现实管理器中所有命令

/* cmd manager define */
struct cmd_manager_t {
    struct cmd_node_t* cmd_parse_list;
    unsigned int cmd_num;

	/* 0:not init 1:init */
	char is_manager_init_flag;	
};

/* 命令 */
struct cmd_node_t {
    struct cmd_node_t* next;
    struct cmd_node_t* prev;
    int cmd_type;
    int cmd;
    int (*exec_cmd)(void* cmd_data, unsigned int data_len, void *p_ret);
    void (*cmd_print)(struct cmd_node_t* cn);
    unsigned int cnt_in_list;
};
extern struct cmd_manager_t *get_cmd_manager(void);
extern int cmd_add(struct cmd_node_t* cn);
extern int exec_cmd(int cmd_type, int cmd, void* cmd_data, unsigned int data_len, void* p_ret);
extern int exec_cmd_by_cmd_pkg(struct cmd_pkg_t* p_cmd_pkg, void* p_ret);
extern int exec_cmd_by_pkg(char* pkg, void* p_ret);
extern int cmd_manager_init(void);
extern int cmd_manager_release(void);
extern int cmd_register(int cmd_type, int cmd, void* exec_func, void* print_func);

#endif