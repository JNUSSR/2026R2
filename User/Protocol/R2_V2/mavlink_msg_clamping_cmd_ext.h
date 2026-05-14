#pragma once
// MESSAGE CLAMPING_CMD_EXT PACKING

#define MAVLINK_MSG_ID_CLAMPING_CMD_EXT 204


typedef struct __mavlink_clamping_cmd_ext_t {
 float angle; /*< [deg] Target angle for the clamping mechanism*/
} mavlink_clamping_cmd_ext_t;

#define MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN 4
#define MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN 4
#define MAVLINK_MSG_ID_204_LEN 4
#define MAVLINK_MSG_ID_204_MIN_LEN 4

#define MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC 153
#define MAVLINK_MSG_ID_204_CRC 153



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CLAMPING_CMD_EXT { \
    204, \
    "CLAMPING_CMD_EXT", \
    1, \
    {  { "angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_clamping_cmd_ext_t, angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CLAMPING_CMD_EXT { \
    "CLAMPING_CMD_EXT", \
    1, \
    {  { "angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_clamping_cmd_ext_t, angle) }, \
         } \
}
#endif

/**
 * @brief Pack a clamping_cmd_ext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param angle [deg] Target angle for the clamping mechanism
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_clamping_cmd_ext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN];
    _mav_put_float(buf, 0, angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
#else
    mavlink_clamping_cmd_ext_t packet;
    packet.angle = angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CLAMPING_CMD_EXT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
}

/**
 * @brief Pack a clamping_cmd_ext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param angle [deg] Target angle for the clamping mechanism
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_clamping_cmd_ext_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN];
    _mav_put_float(buf, 0, angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
#else
    mavlink_clamping_cmd_ext_t packet;
    packet.angle = angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CLAMPING_CMD_EXT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
#endif
}

/**
 * @brief Pack a clamping_cmd_ext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param angle [deg] Target angle for the clamping mechanism
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_clamping_cmd_ext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN];
    _mav_put_float(buf, 0, angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
#else
    mavlink_clamping_cmd_ext_t packet;
    packet.angle = angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CLAMPING_CMD_EXT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
}

/**
 * @brief Encode a clamping_cmd_ext struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param clamping_cmd_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_clamping_cmd_ext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_clamping_cmd_ext_t* clamping_cmd_ext)
{
    return mavlink_msg_clamping_cmd_ext_pack(system_id, component_id, msg, clamping_cmd_ext->angle);
}

/**
 * @brief Encode a clamping_cmd_ext struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param clamping_cmd_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_clamping_cmd_ext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_clamping_cmd_ext_t* clamping_cmd_ext)
{
    return mavlink_msg_clamping_cmd_ext_pack_chan(system_id, component_id, chan, msg, clamping_cmd_ext->angle);
}

/**
 * @brief Encode a clamping_cmd_ext struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param clamping_cmd_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_clamping_cmd_ext_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_clamping_cmd_ext_t* clamping_cmd_ext)
{
    return mavlink_msg_clamping_cmd_ext_pack_status(system_id, component_id, _status, msg,  clamping_cmd_ext->angle);
}

/**
 * @brief Send a clamping_cmd_ext message
 * @param chan MAVLink channel to send the message
 *
 * @param angle [deg] Target angle for the clamping mechanism
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_clamping_cmd_ext_send(mavlink_channel_t chan, float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN];
    _mav_put_float(buf, 0, angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLAMPING_CMD_EXT, buf, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
#else
    mavlink_clamping_cmd_ext_t packet;
    packet.angle = angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLAMPING_CMD_EXT, (const char *)&packet, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
#endif
}

/**
 * @brief Send a clamping_cmd_ext message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_clamping_cmd_ext_send_struct(mavlink_channel_t chan, const mavlink_clamping_cmd_ext_t* clamping_cmd_ext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_clamping_cmd_ext_send(chan, clamping_cmd_ext->angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLAMPING_CMD_EXT, (const char *)clamping_cmd_ext, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
#endif
}

#if MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_clamping_cmd_ext_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLAMPING_CMD_EXT, buf, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
#else
    mavlink_clamping_cmd_ext_t *packet = (mavlink_clamping_cmd_ext_t *)msgbuf;
    packet->angle = angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLAMPING_CMD_EXT, (const char *)packet, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_MIN_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_CRC);
#endif
}
#endif

#endif

// MESSAGE CLAMPING_CMD_EXT UNPACKING


/**
 * @brief Get field angle from clamping_cmd_ext message
 *
 * @return [deg] Target angle for the clamping mechanism
 */
static inline float mavlink_msg_clamping_cmd_ext_get_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a clamping_cmd_ext message into a struct
 *
 * @param msg The message to decode
 * @param clamping_cmd_ext C-struct to decode the message contents into
 */
static inline void mavlink_msg_clamping_cmd_ext_decode(const mavlink_message_t* msg, mavlink_clamping_cmd_ext_t* clamping_cmd_ext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    clamping_cmd_ext->angle = mavlink_msg_clamping_cmd_ext_get_angle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN? msg->len : MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN;
        memset(clamping_cmd_ext, 0, MAVLINK_MSG_ID_CLAMPING_CMD_EXT_LEN);
    memcpy(clamping_cmd_ext, _MAV_PAYLOAD(msg), len);
#endif
}
