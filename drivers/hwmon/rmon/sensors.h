#ifndef INCLUDE_RMON_SENSORS_H_
#define INCLUDE_RMON_SENSORS_H_

struct rmon_sensor{
  int (*get)(struct rmon_data*, int*, int, int);
  unsigned char type;
  int get_which;
  int get_arg;
};

struct rmon_sensor_attribute {
  struct device_attribute dev_attr;
  struct rmon_sensor rmsensor;
};

#define to_rmon_sensor_attr(_dev_attr) \
  container_of(_dev_attr, struct rmon_sensor_attribute, dev_attr)

#define RMON_SENSOR_ATTR(_name, _mode, _show, _store, _decode, _type, _which, _arg) \
  { .dev_attr = __ATTR(_name, _mode, _show, _store), \
    .rmsensor = {_decode, _type, _which, _arg}}

enum rmon_sensor_type{RMON_VOLTAGE, RMON_CURRENT, RMON_POWER, RMON_TEMP, RMON_FAN, RMON_LOGICAL};
const char* rmon_sensor_unit[6] = {"mV", "mA", "mW", "C", "rpm", ""}; 

/* Acquire value function prototypes */
int rmon_get_aq_voltage(struct rmon_data* data, int *value, int which, int fullscale);
int rmon_get_aq_current(struct rmon_data* data, int *value, int which, int fullscale);
int rmon_get_aq_temp(struct rmon_data* data, int *value, int which, int unused);
int rmon_get_powergood(struct rmon_data* data, int *value, int which, int unused);
int rmon_get_fan(struct rmon_data* data, int *value, int which, int unused);

/* Sensor AQ offsets */
#define RMON_AQ_1V5AUX_V 0x0
#define RMON_AQ_1V0_V    0x10
#define RMON_AQ_1V5_V    0x16
#define RMON_AQ_1V8_V    0x19
#define RMON_AQ_2V5_V    0x1c
#define RMON_AQ_3V3_V    0xd
#define RMON_AQ_5V0_V    0xa
#define RMON_AQ_12V_V    0x7
#define RMON_AQ_1V0_I    0x11
#define RMON_AQ_1V5_I    0x17
#define RMON_AQ_1V8_I    0x1a
#define RMON_AQ_2V5_I    0x1d
#define RMON_AQ_3V3_I    0xe
#define RMON_AQ_5V0_I    0xb
#define RMON_AQ_12V_I    0x8
#define RMON_AQ_V5_T     0x3
#define RMON_AQ_PPC_T    0x9
#define RMON_AQ_MON_T    0x1f

/* PG bit offsets */
#define RMON_PM_3V3AUX_PG 0x0
#define RMON_PM_MGTPLL_PG 0x1
#define RMON_PM_MGTVTT_PG 0x2
#define RMON_PM_MGTVCC_PG 0x3

/* Fan status offsets */
#define RMON_FC_V5_FAN   0x0
#define RMON_FC_CASE_FAN 0x1

/* Defines for the full-scale of the voltages/currents */
#define RMON_FS_1V5AUX_V 2560
#define RMON_FS_1V0_V    2560
#define RMON_FS_1V5_V    2560
#define RMON_FS_1V8_V    2560
#define RMON_FS_2V5_V    2560
#define RMON_FS_3V3_V    4092
#define RMON_FS_5V0_V    8184
#define RMON_FS_12V_V    16368
#define RMON_FS_1V0_I    (256*1000)
#define RMON_FS_1V5_I    (256*250)
#define RMON_FS_1V8_I    (256*500)
#define RMON_FS_2V5_I    (256*100)
#define RMON_FS_3V3_I    (256*100)
#define RMON_FS_5V0_I    (256*250)

/* current measuring tends to be off by a fixed offset around 0x40 */
#define RMON_CUR_OFFSET  0x40

#endif

