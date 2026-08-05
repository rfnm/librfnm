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

/* RX front-end leg policy. AUTO: banked filters (GNSS/cellular SAW legs on boards that
 * have them) engage when the whole capture fits their passband - best NF/selectivity
 * for narrowband listening. FLAT: hold the broadband leg across the band - hop levels
 * stay comparable for panorama consumers (RTSA/sweep), where a leg change mid-span
 * reads as a fake noise-floor shelf. */
enum rfnm_rx_fe_mode {
	RFNM_RX_FE_AUTO,
	RFNM_RX_FE_FLAT,
	/* CALIB-FROZEN: byte-exact replica of the FE banking the factory calibration
	 * line ran (la93xx_host_sw_calib_works_may_2025) - the per-board loop rows in
	 * rfnm.hardware are only comparable against THIS configuration. Never retune. */
	RFNM_RX_FE_CALIB
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
	RFNM_PATH_NULL,        /* sentinel: terminates path_possible lists, "no selection" */
	RFNM_PATH_TERMINATED   /* RX input parked into the FE's resistive termination (no antenna,
	                        * no TX): the receiver-only floor for flatness/cal work */
};

enum rfnm_ch_enable {
	RFNM_CH_RF_OFF,
	RFNM_CH_RF_ON,
	RFNM_CH_RF_ON_TDD
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
	int8_t avail;
	int8_t gain;
	struct rfnm_range_8b gain_range;
	int16_t rfic_dc_q;
	int16_t rfic_dc_i;
	enum rfnm_ch_enable enable;
	enum rfnm_ch_stream stream;
	enum rfnm_agc_type agc;
	enum rfnm_bias_tee bias_tee;
	/* the old 4-byte fm_notch enum slot, split into byte fields so the struct CANNOT
	 * grow (sizeof and every offset unchanged; little-endian legacy writers put the
	 * fm_notch value in byte 0 and zeros here, which reads as defaults) */
	uint8_t fm_notch;       /* enum rfnm_fm_notch */
	uint8_t fe_mode;        /* enum rfnm_rx_fe_mode (#39) */
	uint8_t ch_rsvd0;       /* future per-channel flags; zero = default */
	uint8_t ch_rsvd1;
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
	struct rfnm_dev_hwinfo_clock {
	uint64_t samp_rate;
	uint64_t samp_rate_min;
	uint64_t samp_rate_max;
	uint32_t samp_rate_step;
	uint64_t dcs_clk;	// raw DCS PLL frequency; the converters run at dcs_clk >> *_dcs_div
	// programmed stream chain state, decoded from the last stream command. The effective ADC
	// rate is dcs_clk >> rx_dcs_div and the user rate is that >> rx_decim_log2. All zero until
	// a stream has programmed the chain; reprogrammed on every channel list apply.
	uint8_t rx_dcs_div;	// 1 = hardware divide-by-2 engaged between DCS and ADC
	uint8_t tx_dcs_div;
	uint8_t rx_decim_log2;	// VSPA decimation factor 2^k
	uint8_t tx_interp_log2;
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_hwinfo {
	uint32_t protocol_version;
	struct rfnm_dev_hwinfo_bit motherboard;
	struct rfnm_dev_hwinfo_bit daughterboard[2];
	struct rfnm_dev_hwinfo_clock clock;
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
	uint32_t cc_samp_rate;
	int32_t samp_rate_ecode;
}
);

// v4: which channel-config field a validation rejection was about (device-side
// mirror: la9310rfnm rfnm_status_ext.h - keep in sync). librfnm composes the human
// message from (field, ecode) + the requested value it already holds + the
// advertised ranges; transient device conditions are never represented here.
enum rfnm_reject_field {
	RFNM_REJ_NONE = 0,
	RFNM_REJ_FREQ,
	RFNM_REJ_LPF_BW,
	RFNM_REJ_POWER,
	RFNM_REJ_GAIN,
	RFNM_REJ_DC_TRIM,
	RFNM_REJ_ENABLE,
	RFNM_REJ_STREAM,
	RFNM_REJ_AGC,
	RFNM_REJ_BIAS_TEE,
	RFNM_REJ_FM_NOTCH,
	RFNM_REJ_PATH,
	RFNM_REJ_RATE,
	RFNM_REJ_CH_MISSING,
	RFNM_REJ_DEVICE,
	RFNM_REJ_BOOT,		// radio bring-up incomplete: the board refused the session
				// verb until its boot script signals the driver stack is up
};

RFNM_PACKED_STRUCT(
	struct rfnm_dev_get_set_result_ext {
	struct rfnm_dev_get_set_result base;
	uint8_t tx_reject_field[8];	// enum rfnm_reject_field per channel, 0 = none
	uint8_t rx_reject_field[8];
	uint8_t samp_rate_reject_field;
	// ---- protocol v5 (2026-07-13): the apply timing handle (device-side mirror:
	// la9310rfnm rfnm_status_ext.h - keep in sync). Break bits + an AT-APPLY (pre-send)
	// wire-epoch snapshot; settle = "every broken direction's epoch advanced PAST the
	// snapshot" (wrap-safe s32 diff > 0). No predicted epoch, no dynamic pending state -
	// liveness rides the status poll. A direction the apply turned OFF gets no bit:
	// that timeline ended, there is nothing to wait on.
	uint8_t timing_break;		// bit0 rx break, bit1 tx break, bit2 dcs_freq reclock
	uint32_t rx_epoch_at_apply;	// wire (u32) epochs when the apply was processed,
	uint32_t tx_epoch_at_apply;	// snapshotted BEFORE the device-side work fires
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_set_samp_rate {
	uint64_t freq;
	uint32_t cc;
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

	// phytimer phase 1: computed-stamp chain breaks seen by the kernel (a sub's stamp
	// disagreed with the previous sub's stamp + size x R without a DISCONT/epoch resync).
	// Loud by design: DSP data loss must never be transparent at the transport layer.
	uint64_t la_phytimer_error[4];
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
	uint64_t usb_dac_last_dqbuf[4];
	uint64_t usb_adc_last_qbuf[4];
	struct rfnm_stream_stats stream_stats;
	//struct rfnm_m7_status m7_status;

	// the local/TCP transport keeps its own RX packet counter: variable-size USB packets make the
	// usb ring cc advance faster than the local ring cc, so TCP/local consumers must check this one
	uint64_t local_adc_last_qbuf[4];

	// TX ring protocol (fw-published, heartbeat-fresh): read position, state
	// (bit0 = tx active, bits 4:1 = upsampling log2), fw arm/disarm sequence, and
	// the driver's write head - occupancy = (head - read_ptr) mod ring
	uint32_t tx_ring_read_ptr;
	uint32_t tx_state;
	uint32_t tx_stream_seq;
	uint32_t tx_ring_head;

	// phytimer phase 1: RX timing anchor (the stream ack), fw-published on stream start.
	// rx_t0 = tick of the first sample of the current epoch; ticks per output sample
	// R = 2^rx_r_shift / 2 (exact); rx_regate_cnt = lanes parked after an AXIQ overrun
	// this epoch (the kernel re-gates on seeing it - a new epoch). This is the host's
	// timebase anchor - the running RX stream's stamps ARE the clock for scheduling.
	uint32_t rx_t0;
	uint32_t rx_epoch;
	uint32_t rx_r_shift;
	uint32_t rx_regate_cnt;

	// phytimer phase 3: TX timing anchor. tx_t0 = tick the DAC gate opens (the SAME
	// minted tick as rx_t0 when both directions apply together - one timeline); ring
	// slot n airs at tx_t0 + n*256*R ticks, R = 2^tx_r_shift / 2 exact. tx_epoch
	// increments per TX stream start. Health: fw DAC starvation + kernel-side timed
	// placement rejects (misaligned / too-late / too-far / rewind).
	uint32_t tx_t0;
	uint32_t tx_epoch;
	uint32_t tx_r_shift;
	uint32_t tx_underrun_cnt;
	uint32_t tx_timed_reject_cnt;

	// phytimer: current tick, captured (C21 spare channel) at every status populate.
	// Lets any client read board time WITHOUT running an RX stream (an RX apply on a
	// TX board steals the shared FE port - never require one just to read the clock).
	// Freshness = one status-fetch round trip; unwrap host-side (32-bit, ~70 s wrap).
	uint32_t phytimer_now;

	// the feed contract (exactness plan): what the kernel pump actually enforces, so
	// clients size their lead from the device instead of folklore constants. Mirrors
	// the ksw module's rfnm_status_ext.h - keep the two in sync.
	uint32_t tx_feed_lead_ticks;	// TX pump feed lead in phytimer ticks (rate-aware)
	uint32_t rx_flush_deadline_us;	// partial RX packet flush deadline

	// #19 pump-event telemetry (module-side cumulative counters since insmod, never
	// reset - diff across a session): tx_pace_rolls = mispaced-start regate REQUESTS
	// from the kernel TX pace check; tx_arm_repairs = self-heal stream re-applies that
	// carried TX - each one re-mints tx_t0/tx_epoch behind the client's back and hands
	// the fw one GO-detect arm-repair opportunity (the actual fw repair firing is only
	// visible on the M4 console; this is the kernel-visible proxy). A mid-session
	// quality collapse that correlates with a bump here was a pump event, not RF.
	uint32_t tx_pace_rolls;
	uint32_t tx_arm_repairs;

	// ---- protocol v3 bundle (r6-5b, 2026-07-11; mirrors ksw rfnm_status_ext.h) ----
	// POS placement outcomes, cumulative since module load (diff across a session):
	// the client-side answer to "my counters read clean while the kernel dropped 92%"
	// (the OAI UL night). Any nonzero delta of late/stale/misaligned during YOUR
	// session is YOUR feed - loud in dmesg board-side, now visible here too.
	uint32_t tx_pos_placed;
	uint32_t tx_pos_late;
	uint32_t tx_pos_stale;
	uint32_t tx_pos_misaligned;
	// honest usable write-lead minimum in phytimer ticks: the 16-slot placement
	// guard + a 1 ms publish/staleness allowance (r6's measured 1.0-1.2 ms floor).
	// tx_feed_lead_ticks above stays the historical (conservative) advisory.
	uint32_t tx_feed_min_lead_ticks;
	// the anchor congruence step anchors mint on (384 << max(rx_dcs, tx_dcs)),
	// computed device-side from the APPLIED stream word - consume this, never
	// re-derive from cached hwinfo (the r4b anchor_at_probe folklore).
	uint32_t sched_anchor_step_ticks;
	// RESERVED (read 0) until their producers land: AXIQ-vs-DDR-RD underrun split
	// (needs the VSPA DMEM aperture proven) and the kernel slip engine (#23).
	uint32_t tx_underrun_axiq;
	uint32_t tx_underrun_ddr_rd;
	uint32_t tx_slip_ticks;
	// ---- protocol v4 (2026-07-12) ----
	// cc of the most recent late/stale POS placement: attribute a late event to a
	// specific burst (last-event + counters, not a per-burst ledger)
	uint64_t tx_last_late_usb_cc;
	// ---- time unification step 1 (~/r/docs/time-unification-plan-2026-07-17.md) ----
	// THE one anchor, absolute: phytimer_now64 = (wrap_count << 32) | hw_low32,
	// kernel-owned extension, SAMPLED AT REQUEST TIME (never a cached copy). phy_gen
	// = LA9310 boot generation: ticks compare only within one gen; a gen move voids
	// every outstanding time promise (TIME_RESET) - reopen and re-read (gen, now64).
	uint64_t phytimer_now64;
	uint32_t phy_gen;
	uint32_t phy_rsvd;
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
	RFNM_SET_SAMP_RATE,
	RFNM_SET_RX_REPEAT,
	RFNM_HARD_RESET_LA9310,
	RFNM_GET_HARD_RESET_STATUS,
	// phytimer phase 2: configure the M4 TDD scheduler (gates + FE flips on one tick
	// grid, sample-exact stamps). Payload = struct rfnm_dev_tdd; both zero stops.
	RFNM_SET_TDD,
	// v3 phase 1: the absolute-time request ring (schedule-native contract).
	// Payload = struct rfnm_dev_txn; op 0 resets the ring (generation bump), op 1
	// pushes one entry. Entries carry ABSOLUTE phytimer ticks, monotonic, written
	// ahead of a per-kind minimum lead; late = rejected loudly. LOCAL v1.
	RFNM_SET_TXN,
};

RFNM_PACKED_STRUCT(
	struct rfnm_dev_tdd {
	uint32_t period_chunks;	// pattern in CHUNKS (768 ADC samples): alignment is structural
	uint32_t duty_chunks;	// RX-window length; rest of the period = TX/FE-A profile
}
);

RFNM_PACKED_STRUCT(
	struct rfnm_dev_txn {
	uint32_t op;	// 0 = reset ring, 1 = push entry
	uint32_t tick;	// absolute phytimer tick (the entry's identity)
	uint8_t kind;	// 1 = RX_WINDOW, 2 = TX_SLOT, 3 = FE, 4 = ANCHOR (kernel-side sticky
			// requested stream anchor, flags bit0 = valid - never enters the M4 ring)
	uint8_t flags;
	uint16_t type;	// slot-type tag / FE profile index
	uint32_t len;	// samples
	uint32_t bind;	// TX: source DAC-ring slot
}
);

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

	// scheduling rejects (client-side only, never on the wire). Synchronous on the
	// LOCAL transport; USB/TCP schedule SETs are fire-and-forget - their rejects
	// surface through dev_status counters / tx_health and the board-side dmesg.
	RFNM_API_SCHED_NO_ANCHOR = 12,	// no TX epoch published yet - stream not started
	RFNM_API_SCHED_MISALIGNED = 13,	// tick not on the slot grid (t0 + n*ticks_per_slot)
	RFNM_API_SCHED_LATE = 14,	// below the producer lead floor (kernel ETIME)
	RFNM_API_SCHED_FULL = 15,	// request ring full (kernel ENOSPC)
	RFNM_API_SCHED_ORDER = 16,	// non-monotonic tick (kernel EINVAL)
	RFNM_API_SCHED_NOT_RESET = 17,	// ring never schedule_reset() (kernel ENXIO)
	// defect #18: the USB RX pipe is unrecoverably dead for this session (board-side
	// producer parked; SET_INTERFACE resyncs exhausted their budget without progress).
	// Host-generated, never on the wire. The client must reopen the device - retrying
	// rx_dqbuf will keep returning this until the stream is torn down and restarted.
	RFNM_API_RX_PIPE_DEAD = 18,
	// P3/D1: a positional-intent write (the session seeked) that cannot be stamped
	// (no TX anchor latched and none published). Pre-P3 this silently demoted to
	// free-run and the burst vanished with no counter - the worst behavior the
	// library ever had. Host-generated, never on the wire.
	RFNM_API_TX_NOT_ANCHORED = 19,
} rfnm_api_failcode;

#define RFNM_RX_USB_BUF_MULTI 80
#define RFNM_RX_USB_BUF_SIZE 80
#define RFNM_TX_USB_BUF_MULTI 80	// MAX slots per TX packet; the header's multi field says how many are present (variable-size packets: small for latency, full for throughput)

#define RFNM_LA9310_DMA_RX_SIZE		(256)
#define LA_RX_BASE_BUFSIZE (4*RFNM_LA9310_DMA_RX_SIZE)
#define LA_RX_BASE_BUFSIZE_12 ((LA_RX_BASE_BUFSIZE * 3) / 4)

#define RFNM_RX_USB_BUF_SIZE_ELEMS (RFNM_LA9310_DMA_RX_SIZE * RFNM_RX_USB_BUF_MULTI)

#define RFNM_LA9310_DMA_TX_SIZE		(256)
#define LA_TX_BASE_BUFSIZE (4*RFNM_LA9310_DMA_TX_SIZE)
#define LA_TX_BASE_BUFSIZE_12 ((LA_TX_BASE_BUFSIZE * 3) / 4)

// phytimer phase 1: the two provably-dead cc passthrough head fields are repurposed as
// flags words - same offset, same size, zero-init = contiguous/no-event. Intent is always
// a flag, never a stamp sentinel (phytimer = 0 is a legal tick on a wrapping counter).
// phytimer is the exact tick of the packet's first sample; a packet never contains a
// stamp discontinuity inside its payload (the kernel flushes before a DISCONT sub).
#define RFNM_RX_FLAG_DISCONT		(1u << 0)	// device self-heal re-gate happened right before this packet
#define RFNM_TX_FLAG_TIME_VALID		(1u << 0)	// gate at exactly phytimer (phase 3)
#define RFNM_TX_FLAG_EOB		(1u << 1)	// gate closes after this packet (phase 3)
// positional free-run (defect #20): phytimer = exact tick of the packet's first sample
// (same semantic as TIME_VALID, but the stream free-runs - the device PLACES the
// payload at the tick's ring slot instead of gating a window). Stamped automatically
// by tx_qbuf against the last apply's anchor; epoch rides [15:8]. Late/stale/off-grid
// packets are dropped device-side, counted in the rfnm_tx_stat_pos_* module params.
#define RFNM_TX_FLAG_POS_VALID		(1u << 2)
#define RFNM_STREAM_FLAG_EPOCH(f)	(((f) >> 8) & 0xFF)	// RX stream generation of the stamps in this packet

RFNM_PACKED_STRUCT(
	struct rfnm_rx_usb_buf {
	uint32_t magic;
	uint32_t adc_id;
	uint32_t phytimer;
	uint32_t fmt;	// RFNM_PACKET_FMT_*: payload encoding (was unused "dropped")
	uint32_t rx_flags;	// bit0 DISCONT, [15:8] epoch (was the dead adc_cc passthrough)
	uint64_t usb_cc;
	uint32_t elem_cnt;	// samples in buf (variable-size packets: latency-deadline partial flush)
	uint8_t buf[LA_RX_BASE_BUFSIZE_12 * RFNM_RX_USB_BUF_MULTI];
}
);
// local transport assumes tx/rx are the same size
RFNM_PACKED_STRUCT(
	struct rfnm_tx_usb_buf {
	uint32_t magic;
	uint32_t dac_id;
	uint32_t phytimer;
	uint32_t fmt;	// RFNM_PACKET_FMT_*: payload encoding (was unused "dropped")
	uint32_t tx_flags;	// bit0 TIME_VALID, bit1 EOB, [15:8] epoch (was the never-populated dac_cc)
	uint64_t usb_cc;
	// base bufs (256 samples each) present in this packet, 1..RFNM_TX_USB_BUF_MULTI;
	// wire length = RFNM_USB_TX_PACKET_HEAD_SIZE + multi * LA_TX_BASE_BUFSIZE_12
	uint32_t multi;
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

// payload encoding carried in the packet head fmt field. USB always uses
// PACKED12 (wire bandwidth); the local transport uses CS16 so neither side ever
// packs or unpacks on the same SoC. TCP ships whatever the packet declares:
// PACKED12 when a remote session owns the radio, CS16 while a local session
// does - so a remote listener keeps receiving across a local takeover instead
// of silently starving (defect #96). Consumers branch per packet.
#define RFNM_PACKET_FMT_PACKED12 0
#define RFNM_PACKET_FMT_CS16 1

// local transport ring elements: identical heads, payload grown so a CS16 packet
// carries the same elem/multi capacity as a PACKED12 wire packet. The packed structs
// keep buf and ext contiguous; address payloads via RFNM_USB_*_PACKET_HEAD_SIZE.
RFNM_PACKED_STRUCT(
	struct rfnm_local_rx_pkt {
	struct rfnm_rx_usb_buf p;
	uint8_t ext[(LA_RX_BASE_BUFSIZE - LA_RX_BASE_BUFSIZE_12) * RFNM_RX_USB_BUF_MULTI];
}
);
RFNM_PACKED_STRUCT(
	struct rfnm_local_tx_pkt {
	struct rfnm_tx_usb_buf p;
	uint8_t ext[(LA_TX_BASE_BUFSIZE - LA_TX_BASE_BUFSIZE_12) * RFNM_TX_USB_BUF_MULTI];
}
);
#define RFNM_LOCAL_RX_PACKET_SIZE (sizeof(struct rfnm_local_rx_pkt))
#define RFNM_LOCAL_TX_PACKET_SIZE (sizeof(struct rfnm_local_tx_pkt))

#define RFNM_SYSCTL_TRANSFER_SIZE 1152

#define RFNM_IOCTL_BASE  ( ((3U) << 30) | (('R') << 8) | ( 0U << 0 ) | (RFNM_SYSCTL_TRANSFER_SIZE << 16) )
#define RFNM_IOCTL_BASE_DATA  ( ((3U) << 30) | (('R') << 8) | ( 0U << 0 ) | (RFNM_USB_TX_PACKET_SIZE << 16) )

// /dev/rfnm_data_ep zero-copy protocol - values mirrored in la93xx_host_sw
// la9310_rfnm.h, keep the two in sync. The RX/TX packet pools are mmap'd from the
// chardev at the offsets below; the ioctls move pool indices only, never payload.
#define RFNM_LOCAL_RX_GET 2	// writes u32 pool index of the next filled RX packet to *arg
#define RFNM_LOCAL_RX_PUT 3	// arg = index: return the packet to the free pool
#define RFNM_LOCAL_TX_ACQ 4	// writes u32 pool index of a claimed free TX slot to *arg
#define RFNM_LOCAL_TX_SUB 5	// arg = index: submit the filled slot for transmission
#define RFNM_LOCAL_MMAP_RX_OFFSET 0x0UL
#define RFNM_LOCAL_MMAP_TX_OFFSET 0x10000000UL

#define RFNM_PROTOCOL_VERSION 5
#define RFNM_UDP_CTRL_PORT 28285
#define RFNM_UDP_DATA_PORT 28286
#define RFNM_TCP_CTRL_PORT 28287
#define RFNM_TCP_DATA_PORT 28288

RFNM_PACKED_STRUCT(
struct rfnm_tcp_ctrl_header {
	uint8_t cmd;
	uint16_t size;
}
);
