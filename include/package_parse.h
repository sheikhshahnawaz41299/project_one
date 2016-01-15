#ifndef _PACKAGE_PARSE_H_
#define _PACKAGE_PARSE_H_

struct cmd_pkg_t {
    int cmd_type;
    int cmd;
    int data_len;
    char* data;
};
extern struct cmd_pkg_t* pkg_parse(char* pkg);
extern void cmd_pkg_free(struct cmd_pkg_t* cpkg);

#endif
