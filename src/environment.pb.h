/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.4 */

#ifndef PB_ENVIRONMENT_PB_H_INCLUDED
#define PB_ENVIRONMENT_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _ActuatorMessage {
    bool has_light;
    bool light;
    bool has_window;
    bool window;
} ActuatorMessage;

typedef struct _alartMessage {
    bool alarm;
} alartMessage;

typedef struct _environmentMessage {
    bool has_lightLevel;
    int32_t lightLevel;
    bool has_temperature;
    float temperature;
    bool has_humidity;
    float humidity;
    bool has_airQuality;
    int32_t airQuality;
    bool has_presence;
    bool presence;
} environmentMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define environmentMessage_init_default          {false, 0, false, 0, false, 0, false, 0, false, 0}
#define alartMessage_init_default                {0}
#define ActuatorMessage_init_default             {false, 0, false, 0}
#define environmentMessage_init_zero             {false, 0, false, 0, false, 0, false, 0, false, 0}
#define alartMessage_init_zero                   {0}
#define ActuatorMessage_init_zero                {false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define ActuatorMessage_light_tag                1
#define ActuatorMessage_window_tag               2
#define alartMessage_alarm_tag                   1
#define environmentMessage_lightLevel_tag        1
#define environmentMessage_temperature_tag       2
#define environmentMessage_humidity_tag          3
#define environmentMessage_airQuality_tag        4
#define environmentMessage_presence_tag          5

/* Struct field encoding specification for nanopb */
#define environmentMessage_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    lightLevel,        1) \
X(a, STATIC,   OPTIONAL, FLOAT,    temperature,       2) \
X(a, STATIC,   OPTIONAL, FLOAT,    humidity,          3) \
X(a, STATIC,   OPTIONAL, INT32,    airQuality,        4) \
X(a, STATIC,   OPTIONAL, BOOL,     presence,          5)
#define environmentMessage_CALLBACK NULL
#define environmentMessage_DEFAULT NULL

#define alartMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, BOOL,     alarm,             1)
#define alartMessage_CALLBACK NULL
#define alartMessage_DEFAULT NULL

#define ActuatorMessage_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, BOOL,     light,             1) \
X(a, STATIC,   OPTIONAL, BOOL,     window,            2)
#define ActuatorMessage_CALLBACK NULL
#define ActuatorMessage_DEFAULT NULL

extern const pb_msgdesc_t environmentMessage_msg;
extern const pb_msgdesc_t alartMessage_msg;
extern const pb_msgdesc_t ActuatorMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define environmentMessage_fields &environmentMessage_msg
#define alartMessage_fields &alartMessage_msg
#define ActuatorMessage_fields &ActuatorMessage_msg

/* Maximum encoded size of messages (where known) */
#define environmentMessage_size                  34
#define alartMessage_size                        2
#define ActuatorMessage_size                     4

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
