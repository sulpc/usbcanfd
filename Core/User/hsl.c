#include "hsl.h"

#include "can.h"
#include "log.h"
#include "system.h"
#include "usbd_cdc_if.h"
#include "util_misc.h"
#include "util_ringbuffer.h"
#include "util_tfp.h"

#define HSL_FRAME_SIZE_MAX     256
#define HSL_PACK_SIZE          64   // FS: max package length = 64
#define HSL_RXDATA_BUFFER_SIZE 1024
#define HSL_TXDATA_BUFFER_SIZE 1024


// proto + mtype + stype + data

#define HSL_PROTO_CAN           0x01
#define HSL_MTYPE_CAN_CONFIG    0x01
#define HSL_MTYPE_CAN_CONNECT   0x02
#define HSL_MTYPE_CAN_UNCONNECT 0x03
#define HSL_MTYPE_CAN_TRANSFER  0x04
#define HSL_MTYPE_CAN_DOWNLINK  0x04
#define HSL_MTYPE_CAN_UPLINK    0x05
#define HSL_MTYPE_ACK(mtype)    (mtype | 0x80)

#define HSL_CAN_FRAME_IDE 0x01
#define HSL_CAN_FRAME_FDF 0x02
#define HSL_CAN_FRAME_BRS 0x04

static uint8_t hsl_canmsg_buffer[64 + 5 + 3];
#define get_canmsg_buffer()       (&hsl_canmsg_buffer[0])
#define get_canmsg_proto_buffer() (&hsl_canmsg_buffer[0])
#define get_canmsg_mtype_buffer() (&hsl_canmsg_buffer[1])
#define get_canmsg_stype_buffer() (&hsl_canmsg_buffer[2])
#define get_canmsg_mdata_buffer() (&hsl_canmsg_buffer[3])   // id(4) + type(1) + data(64)


static void hsl_send_frm_to_host(const uint8_t* data, uint16_t len);
static void hsl_recv_msg_from_host(const uint8_t* data, uint16_t len);


static util_ringbuffer_t buf_from_host;   // used in the user process and the usb isr
static util_ringbuffer_t buf_to_host;     // used in the user process only
static uint8_t           rxbuf_raw[HSL_RXDATA_BUFFER_SIZE];
static uint8_t           txbuf_raw[HSL_TXDATA_BUFFER_SIZE];
static uint8_t           rxmsgbuf[HSL_FRAME_SIZE_MAX];
static uint8_t           txfrmbuf[HSL_FRAME_SIZE_MAX];
static tfp_t             tfp = {
                .msg_cbk   = hsl_recv_msg_from_host,
                .frm_cbk   = hsl_send_frm_to_host,
                .msg_buf   = rxmsgbuf,
                .msg_bufsz = sizeof(rxmsgbuf),
                .frm_buf   = txfrmbuf,
                .frm_bufsz = sizeof(txfrmbuf),
};


static void hsl_can_proto_send(uint8_t mtype, uint8_t stype, const uint8_t* data, uint16_t len)
{
    if (data == get_canmsg_mdata_buffer() || data == nullptr) {
        *get_canmsg_proto_buffer() = HSL_PROTO_CAN;
        *get_canmsg_mtype_buffer() = mtype;
        *get_canmsg_stype_buffer() = stype;
        // log_printf("TXMSG: "), log_bytesdump(get_canmsg_buffer(), len + 3);
        tfp_make_frame(&tfp, get_canmsg_buffer(), len + 3);
    } else {
        // !
    }
}

static void hsl_can_proto_recv(uint8_t mtype, uint8_t stype, const uint8_t* data, uint16_t len)
{
    uint8_t ret = 0;

    switch (mtype) {
    case HSL_MTYPE_CAN_CONFIG:
        if (len != 10) {
            log_printf("invalid msg - mtype=%02x\n", mtype);
            break;
        }
        uint32_t nomi_bitrate = util_getbigendian4(data);
        uint8_t  nomi_sample  = data[4];
        uint32_t data_bitrate = util_getbigendian4(data + 5);
        uint8_t  data_sample  = data[9];

        if (can_config(nomi_bitrate, nomi_sample, data_bitrate, data_sample) != 0) {
            ret = 0xff;
            log_printf("can set config fail\n");
        }
        hsl_can_proto_send(HSL_MTYPE_ACK(HSL_MTYPE_CAN_CONFIG), ret, nullptr, 0);
        break;

    case HSL_MTYPE_CAN_CONNECT:
        if (len != 0) {
            log_printf("invalid msg - mtype=%02x\n", mtype);
            break;
        }

        if (can_connect() != 0) {
            log_printf("can connect fail\n");
        }
        hsl_can_proto_send(HSL_MTYPE_ACK(HSL_MTYPE_CAN_CONNECT), ret, nullptr, 0);
        break;

    case HSL_MTYPE_CAN_UNCONNECT:
        if (len != 0) {
            log_printf("invalid msg - mtype=%02x\n", mtype);
            break;
        }
        if (can_unconnect() != 0) {
            ret = 0xff;
            log_printf("can unconnect fail\n");
        }
        hsl_can_proto_send(HSL_MTYPE_ACK(HSL_MTYPE_CAN_UNCONNECT), ret, nullptr, 0);
        break;

    case HSL_MTYPE_CAN_TRANSFER:
        if (len < 5) {
            log_printf("invalid msg - mtype=%02x\n", mtype);
            break;
        }
        uint32_t id    = util_getbigendian4(data);
        uint8_t  flags = data[4];
        // log_printf("TXCAN %03X %d: ", id, len - 5), log_bytesdump(data + 5, len - 5);
        if (can_send(id, flags & HSL_CAN_FRAME_IDE, flags & HSL_CAN_FRAME_FDF, flags & HSL_CAN_FRAME_BRS, data + 5,
                     len - 5) != 0) {
            // can_send fail
            log_printf("can send fail\n");
            hsl_can_proto_send(HSL_MTYPE_ACK(HSL_MTYPE_CAN_TRANSFER), 0xff, nullptr, 0);
        }
        break;

    default:
        break;
    }
}

static void hsl_recv_msg_from_host(const uint8_t* data, uint16_t len)
{
    // msg: proto(1) + mtype(1) + stype(1) + data(*)
    // log_printf("RXMSG: "), log_bytesdump(data, len);

#if 0
    tfp_make_frame(&tfp, data, len);
#else
    if (len < 3) {
        return;
    }

    switch (data[0]) {
    case HSL_PROTO_CAN:
        hsl_can_proto_recv(data[1], data[2], data + 3, len - 3);
        break;

    default:
        break;
    }
#endif
}

static void hsl_send_frm_to_host(const uint8_t* data, uint16_t len)
{
    if (buf_to_host.cap < len) {
        // ! buffer overflow
    } else {
        util_ringbuffer_put(&buf_to_host, data, len);
    }
}

static void hsl_can_recv(uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data, uint8_t len)
{
    // log_printf("RXCAN %03X %d: ", id, len), log_bytesdump(data, len);

    uint8_t* candata = get_canmsg_mdata_buffer();

    util_setbigendian4(candata, id);
    candata[4] = (ide ? HSL_CAN_FRAME_IDE : 0) | (fdf ? HSL_CAN_FRAME_FDF : 0) | (brs ? HSL_CAN_FRAME_BRS : 0);
    memcpy(candata + 5, data, len);

    hsl_can_proto_send(HSL_MTYPE_CAN_TRANSFER, 0, candata, len + 5);
}

void hsl_usb_recv(uint8_t* data, uint16_t len)
{
    util_ringbuffer_put(&buf_from_host, data, len);
}

void hsl_init(void)
{
    util_ringbuffer_init(&buf_from_host, rxbuf_raw, sizeof(rxbuf_raw));
    util_ringbuffer_init(&buf_to_host, txbuf_raw, sizeof(txbuf_raw));
    can_set_nofity(hsl_can_recv);
}

void hsl_process(void)
{
    // rx from host
    {
        static uint8_t  rxpack[HSL_PACK_SIZE];
        static uint16_t rxpack_len = 0;

        system_irq_disable();
        rxpack_len += util_ringbuffer_get(&buf_from_host, rxpack + rxpack_len, HSL_PACK_SIZE - rxpack_len);
        system_irq_enable();

        if (rxpack_len != 0) {
            // log_printf("rxpack:"), log_bytesdump(rxpack, rxpack_len);

            uint16_t feed = tfp_stream_feed(&tfp, rxpack, rxpack_len);

            // pack buffer may remain some data
            memcpy(rxpack, rxpack + feed, rxpack_len - feed);
            rxpack_len -= feed;
        }
    }

    // tx to host
    {
        static uint8_t txpack[HSL_PACK_SIZE];

        if (CDC_TransmitReady_FS()) {
            uint32_t txpack_len = util_ringbuffer_get(&buf_to_host, txpack, HSL_PACK_SIZE);
            if (txpack_len != 0) {
                // log_printf("cdc tx %d\n", txpack_len);
                CDC_Transmit_FS(txpack, txpack_len);
            }
        }
    }
}
