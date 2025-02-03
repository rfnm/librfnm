#pragma once
#include <cstdint>

#if defined(__GNUC__)
#define RFNM_PACKED_STRUCT( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#elif defined(_MSC_VER)
#define RFNM_PACKED_STRUCT( __Declaration__ ) __pragma(pack(push,1)) __Declaration__ __pragma(pack(pop))
#endif

enum rfnm_ch_data_type {
	RFNM_CH_DATA_TYPE_COMPLEX,
	RFNM_CH_DATA_TYPE_REAL
};

enum rfnm_agc_type {
	RFNM_AGC_OFF,
	RFNM_AGC_DEFAULT
};

enum rfnm_bias_tee {
	RFNM_BIAS_TEE_OFF,
	RFNM_BIAS_TEE_ON
};

enum rfnm_fm_notch {
	RFNM_FM_NOTCH_AUTO,
	RFNM_FM_NOTCH_ON,
	RFNM_FM_NOTCH_OFF
};

enum rfnm_rf_path {
	RFNM_PATH_SMA_A,
	RFNM_PATH_SMA_B,
	RFNM_PATH_SMA_C,
	RFNM_PATH_SMA_D,
	RFNM_PATH_SMA_E,
	RFNM_PATH_SMA_F,
	RFNM_PATH_SMA_G,
	RFNM_PATH_SMA_H,
	RFNM_PATH_EMBED_ANT,
	RFNM_PATH_LOOPBACK,
	RFNM_PATH_NULL
};

enum rfnm_ch_enable {
	RFNM_CH_OFF,
	RFNM_CH_ON,
	RFNM_CH_ON_TDD
};

enum rfnm_ch_stream {
	RFNM_CH_STREAM_AUTO,
	RFNM_CH_STREAM_OFF,
	RFNM_CH_STREAM_ON
};

RFNM_PACKED_STRUCT(
	struct rfnm_range_8b {
	int8_t min;
	uint8_t max;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_api_tx_ch {
	int8_t abs_id;
	int8_t dgb_ch_id;
	int8_t dgb_id;
	int8_t dac_id;
	int64_t freq_min;
	int64_t freq_max;
	int64_t freq;
	int16_t rfic_lpf_bw;
	/*int16_t samp_freq_div_m;
	int16_t samp_freq_div_n;*/
	int8_t avail;
	int8_t power;
	struct rfnm_range_8b power_range;
	enum rfnm_ch_enable enable;
	enum rfnm_ch_stream stream;
	enum rfnm_bias_tee bias_tee;
	enum rfnm_rf_path path;
	enum rfnm_rf_path path_preferred;
	enum rfnm_rf_path path_possible[10];
	enum rfnm_ch_data_type data_type;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_api_rx_ch {
	int8_t abs_id;
	int8_t dgb_ch_id;
	int8_t dgb_id;
	int8_t adc_id;
	int64_t freq_min;
	int64_t freq_max;
	int64_t freq;
	int16_t rfic_lpf_bw;
	/*int16_t samp_freq_div_m;
	int16_t samp_freq_div_n;*/
	int8_t avail;
	int8_t gain;
	struct rfnm_range_8b gain_range;
	int16_t rfic_dc_q;
	int16_t rfic_dc_i;
	enum rfnm_ch_enable enable;
	enum rfnm_ch_stream stream;
	enum rfnm_agc_type agc;
	enum rfnm_bias_tee bias_tee;
	enum rfnm_fm_notch fm_notch;
	enum rfnm_rf_path path;
	enum rfnm_rf_path path_preferred;
	enum rfnm_rf_path path_possible[10];
	enum rfnm_ch_data_type data_type;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_hwinfo_bit {
	uint8_t board_id;
	uint8_t board_revision_id;
	uint8_t serial_number[9];
	char user_readable_name[30];
	uint8_t mac_addr[6];
	uint8_t tx_ch_cnt;
	uint8_t rx_ch_cnt;
	int16_t temperature;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_hwinfo_clockgen {
	uint64_t dcs_clk;
	uint64_t dcs_clk_min;
	uint64_t dcs_clk_max;
	uint32_t dcs_clk_step;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_hwinfo {
	uint32_t protocol_version;
	struct rfnm_dev_hwinfo_bit motherboard;
	struct rfnm_dev_hwinfo_bit daughterboard[2];
	struct rfnm_dev_hwinfo_clockgen clock;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_tx_ch_list {
	//	int cnt;
	struct rfnm_api_tx_ch ch[8];
	uint8_t apply;
	uint32_t cc;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_rx_ch_list {
	//	int cnt;
	struct rfnm_api_rx_ch ch[8];
	uint8_t apply;
	uint32_t cc;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_get_set_result {
	uint32_t cc_tx;
	uint32_t cc_rx;
	int32_t tx_ecodes[8];
	int32_t rx_ecodes[8];
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_m7_status {
	uint32_t tx_buf_id;
	uint32_t rx_head;
	uint32_t kernel_cache_flush_tail; // this variable shouldn't be here, but it's already mapped ...
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_stream_stats {
	uint64_t usb_tx_ok[4];
	uint64_t usb_tx_error[4];
	uint64_t usb_rx_ok[4];
	uint64_t usb_rx_error[4];
	uint64_t usb_rx_bytes[4];
	uint64_t usb_tx_bytes[4];

	uint64_t local_tx_ok[4];
	uint64_t local_tx_error[4];
	uint64_t local_rx_ok[4];
	uint64_t local_rx_error[4];
	uint64_t local_rx_bytes[4];
	uint64_t local_tx_bytes[4];

	uint64_t eth_tx_ok[4];
	uint64_t eth_tx_error[4];
	uint64_t eth_rx_ok[4];
	uint64_t eth_rx_error[4];
	uint64_t eth_rx_bytes[4];
	uint64_t eth_tx_bytes[4];

	uint64_t la_adc_ok[4];
	uint64_t la_adc_error[4];
	uint64_t la_dac_ok[4];
	uint64_t la_dac_error[4];
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_local_transport_meminfo {
	uint64_t local_rx_memaddr;
	uint64_t local_rx_bufsize;
	uint64_t local_tx_memaddr;
	uint64_t local_tx_bufsize;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_status {
	uint64_t usb_dac_last_dqbuf;
	struct rfnm_stream_stats stream_stats;
	//struct rfnm_m7_status m7_status;

}
);

enum rfnm_control_ep {
	RFNM_GET_DEV_HWINFO = 0xf00,
	RFNM_GET_TX_CH_LIST,
	RFNM_SET_TX_CH_LIST,
	RFNM_GET_RX_CH_LIST,
	RFNM_SET_RX_CH_LIST,
	RFNM_GET_SET_RESULT,
	RFNM_GET_DEV_STATUS,
	RFNM_GET_SM_RESET,
	RFNM_GET_LOCAL_MEMINFO,
	RFNM_SET_DCS,
};

typedef enum {
	RFNM_API_OK = 0,
	RFNM_API_PROBE_FAIL = 1,
	RFNM_API_TUNE_FAIL = 2,
	RFNM_API_GAIN_FAIL = 3,
	RFNM_API_TIMEOUT = 4,
	RFNM_API_USB_FAIL = 5,
	RFNM_API_DQBUF_OVERFLOW = 6,
	RFNM_API_NOT_SUPPORTED = 7,
	RFNM_API_SW_UPGRADE_REQUIRED = 8,
	RFNM_API_DQBUF_NO_DATA = 9,
	RFNM_API_MIN_QBUF_CNT_NOT_SATIFIED = 10,
	RFNM_API_MIN_QBUF_QUEUE_FULL = 11,
	//RFNM_API_DQBUF_UNDERRUN = 7,
} rfnm_api_failcode;

#define RFNM_RX_USB_BUF_MULTI 80
#define RFNM_RX_USB_BUF_SIZE 80
#define RFNM_TX_USB_BUF_MULTI 80

#define RFNM_LA9310_DMA_RX_SIZE		(256)
#define LA_RX_BASE_BUFSIZE (4*RFNM_LA9310_DMA_RX_SIZE)
#define LA_RX_BASE_BUFSIZE_12 ((LA_RX_BASE_BUFSIZE * 3) / 4)

#define RFNM_RX_USB_BUF_SIZE_ELEMS (RFNM_LA9310_DMA_RX_SIZE * RFNM_RX_USB_BUF_MULTI)

#define RFNM_LA9310_DMA_TX_SIZE		(256)
#define LA_TX_BASE_BUFSIZE (4*RFNM_LA9310_DMA_TX_SIZE)
#define LA_TX_BASE_BUFSIZE_12 ((LA_TX_BASE_BUFSIZE * 3) / 4)

RFNM_PACKED_STRUCT(
	struct rfnm_rx_usb_buf {
	uint32_t magic;
	uint32_t adc_id;
	uint32_t phytimer;
	uint32_t dropped;
	uint32_t adc_cc;
	uint64_t usb_cc;
	uint32_t padding[1];
	uint8_t buf[LA_RX_BASE_BUFSIZE_12 * RFNM_RX_USB_BUF_MULTI];
}
);
// local transport assumes tx/rx are the same size
RFNM_PACKED_STRUCT(
	struct rfnm_tx_usb_buf {
	uint32_t magic;
	uint32_t dac_id;
	uint32_t phytimer;
	uint32_t dropped;
	uint32_t dac_cc;
	uint64_t usb_cc;
	uint32_t padding[1];
	uint8_t buf[LA_TX_BASE_BUFSIZE_12 * RFNM_TX_USB_BUF_MULTI];
}
);

#define RFNM_USB_TX_PACKET_SIZE sizeof (struct rfnm_tx_usb_buf)
#define RFNM_USB_TX_PACKET_HEAD_SIZE (RFNM_USB_TX_PACKET_SIZE - (LA_TX_BASE_BUFSIZE_12 * RFNM_TX_USB_BUF_MULTI))
#define RFNM_USB_TX_PACKET_DATA_SIZE (RFNM_USB_TX_PACKET_SIZE - RFNM_USB_TX_PACKET_HEAD_SIZE)
#define RFNM_USB_TX_PACKET_ELEM_CNT (RFNM_USB_TX_PACKET_DATA_SIZE / 3)

#define RFNM_USB_RX_PACKET_SIZE sizeof (struct rfnm_rx_usb_buf)
#define RFNM_USB_RX_PACKET_HEAD_SIZE (RFNM_USB_RX_PACKET_SIZE - (LA_RX_BASE_BUFSIZE_12 * RFNM_RX_USB_BUF_MULTI))
#define RFNM_USB_RX_PACKET_DATA_SIZE (RFNM_USB_RX_PACKET_SIZE - RFNM_USB_RX_PACKET_HEAD_SIZE)
#define RFNM_USB_RX_PACKET_ELEM_CNT (RFNM_USB_RX_PACKET_DATA_SIZE / 3)

#define RFNM_SYSCTL_TRANSFER_SIZE 1152

#define RFNM_IOCTL_BASE  ( ((3U) << 30) | (('R') << 8) | ( 0U << 0 ) | (RFNM_SYSCTL_TRANSFER_SIZE << 16) )
#define RFNM_IOCTL_BASE_DATA  ( ((3U) << 30) | (('R') << 8) | ( 0U << 0 ) | (RFNM_USB_TX_PACKET_SIZE << 16) )

#define RFNM_PROTOCOL_VERSION 2
#define RFNM_UDP_CTRL_PORT 28285
#define RFNM_UDP_DATA_PORT 28286

