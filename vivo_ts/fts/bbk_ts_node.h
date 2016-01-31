#ifndef _BBK_TS_NODE_H_
#define _BBK_TS_NODE_H_
ssize_t fts_custom_gesture_template_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count);
ssize_t fts_custom_gesture_template_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);

ssize_t fts_custom_gesture_remove_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count);
ssize_t fts_custom_gesture_remove_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);

ssize_t fts_custom_gesture_valid_show(struct kobject *kobj, struct kobj_attribute *attr,  char *buf);
ssize_t fts_custom_gesture_valid_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count);

ssize_t touchscreen_user_defined_gesture_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
ssize_t touchscreen_user_defined_gesture_enable_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count);

extern int fts_gesture_switch_rewrite_tochip(struct fts_ts_info *info);
extern int fts_custom_gesture_template_rewrite_tochip(struct fts_ts_info *info);
extern int fts_glove_mode_switch_rewrite_tochip(struct fts_ts_info *info);
extern int bbk_rewrite_usb_charger_flag_tochip(struct fts_ts_info *info);
extern int bbk_rewrite_edge_suppress_switch_tochip(struct fts_ts_info *info);
extern ssize_t touchscreen_edge_suppress_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
extern ssize_t touchscreen_edge_suppress_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count);
#endif
