#ifndef SHIRE_IPC_PROTOCOL_H
#define SHIRE_IPC_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define SHIRE_IPC_MAGIC 0x53484952U /* "SHIR" */
#define SHIRE_IPC_VERSION 2U
#define SHIRE_IPC_MAX_COMMANDS 64U

enum
{
    SHIRE_IPC_STATE = 1,
    SHIRE_IPC_COMMANDS = 2,
    SHIRE_IPC_ACK = 3
};

enum
{
    SHIRE_IPC_CMD_NONE = 0,
    SHIRE_IPC_CMD_MTB = 1,
    SHIRE_IPC_CMD_WHEEL = 2,
    SHIRE_IPC_CMD_THRUSTER = 3
};

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t payload_size;
    uint32_t reserved;
} shire_ipc_header_t;

typedef struct
{
    shire_ipc_header_t header;
    double sim_time;
    double qn[4];
    double wn[3];
    double pos_n[3];
    double vel_n[3];
    double sun_vector_body[3];
    double mag_field_body[3];
    double hvb[3];
    double mass;
    double cm[3];
    double inertia[3][3];
    int32_t eclipse;
    uint32_t reserved;
    double atmo_density;
} shire_ipc_state_t;

typedef struct
{
    uint32_t type;
    int32_t spacecraft_id;
    uint32_t enable_mask;
    uint32_t reserved;
    double values[6];
} shire_ipc_command_t;

typedef struct
{
    shire_ipc_header_t header;
    int32_t status;
    uint32_t reserved;
} shire_ipc_ack_t;

typedef struct
{
    shire_ipc_header_t header;
    uint32_t count;
    uint32_t reserved;
    shire_ipc_command_t commands[SHIRE_IPC_MAX_COMMANDS];
} shire_ipc_commands_t;

/* Command frames contain the header, count/reserved prefix, and only the
 * populated command records. The maximum-sized structure remains bounded
 * receive storage; it is no longer the size transmitted on every tick. */
#define SHIRE_IPC_COMMANDS_PREFIX_SIZE (sizeof(uint32_t) * 2U)
#define SHIRE_IPC_COMMANDS_PAYLOAD_SIZE(count) \
    (SHIRE_IPC_COMMANDS_PREFIX_SIZE + \
     ((size_t)(count) * sizeof(shire_ipc_command_t)))
#define SHIRE_IPC_COMMANDS_FRAME_SIZE(count) \
    (sizeof(shire_ipc_header_t) + SHIRE_IPC_COMMANDS_PAYLOAD_SIZE(count))

#if defined(__cplusplus)
static_assert(sizeof(shire_ipc_header_t) == 16, "unexpected SHIRE IPC header layout");
static_assert(sizeof(shire_ipc_state_t) == 320, "unexpected SHIRE IPC state layout");
static_assert(sizeof(shire_ipc_command_t) == 64, "unexpected SHIRE IPC command layout");
static_assert(sizeof(shire_ipc_ack_t) == 24, "unexpected SHIRE IPC ack layout");
static_assert(sizeof(shire_ipc_commands_t) == 4120, "unexpected SHIRE IPC batch layout");
#else
_Static_assert(sizeof(shire_ipc_header_t) == 16, "unexpected SHIRE IPC header layout");
_Static_assert(sizeof(shire_ipc_state_t) == 320, "unexpected SHIRE IPC state layout");
_Static_assert(sizeof(shire_ipc_command_t) == 64, "unexpected SHIRE IPC command layout");
_Static_assert(sizeof(shire_ipc_ack_t) == 24, "unexpected SHIRE IPC ack layout");
_Static_assert(sizeof(shire_ipc_commands_t) == 4120, "unexpected SHIRE IPC batch layout");
#endif

#endif
