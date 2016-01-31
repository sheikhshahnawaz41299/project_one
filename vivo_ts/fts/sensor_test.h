#ifndef _SENSOR_TEST_H_
#define _SENSOR_TEST_H_

extern ssize_t fts_production_test(struct fts_ts_info *info, char *buf, size_t count);
extern int fts_chip_initialization(struct fts_ts_info *info);
extern int fts_wait_controller_ready(struct fts_ts_info *info);

#endif