#include <stdlib.h>
#include <stdio.h>

#include "cmd_manager.h"
#include "server.h"

/* cmd:0 0
 * desc:show all cmd info in list.
 * 
 **/
int show_cmd_manager_list(void* cmd_data, unsigned int data_len, void *p_ret) {
	struct cmd_manager_t *cmd_manager_p = get_cmd_manager();
    struct cmd_node_t* p_cmd;
	
    if (cmd_manager_p->cmd_num == 0) {
        printf("No cmd in list.\n");
        return 0;
    }
    p_cmd =  cmd_manager_p->cmd_parse_list;
    while (p_cmd != NULL) {
        if (p_cmd->cmd_print != NULL) {
            p_cmd->cmd_print(p_cmd);
        }
        p_cmd = p_cmd->next;
    }
    return 0;
}
void cmd_0_0_info(struct cmd_node_t *cn) {
    printf("cmd type:%d\ncmd:%d\n", cn->cmd_type, cn->cmd);
    printf("cmd num in list:%d\n", cn->cnt_in_list);
    printf("cmd desc:show cmd manager list.\n\n");	
}

/* close server
 * before close,release cmd manager.
 */
int close_server(void* cmd_data, unsigned int data_len, void *p_ret) {
	printf("[%s]cmd server shut down.\n", __func__);
	cmd_manager_release();
	shut_down_server();
	return 0;
}
void cmd_0_1_info(struct cmd_node_t *cn) {
    printf("cmd type:%d\ncmd:%d\n", cn->cmd_type, cn->cmd);
	printf("cmd num in list:%d\n", cn->cnt_in_list);
    printf("cmd desc:close server.\n\n");
}

/* add server control cmds to cmd manager list */
int add_server_control_cmds(void) {

	cmd_register(0, 0, show_cmd_manager_list, cmd_0_0_info);
	cmd_register(0, 1, close_server, cmd_0_1_info);

	return 0;
}