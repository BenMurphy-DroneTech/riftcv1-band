/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "connection.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/sys/crc.h>

#include "esb.h"
#include "rift_ts.h"
#include "rift_airtime.h"

uint8_t last_reset = 0;
//const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
bool esb_state = false;
bool timer_state = false;
bool send_data = false;
uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

uint32_t tx_errors = 0;
int64_t last_tx_success = 0;
int64_t last_tx_fail = 0;

// Link statistics for the stats command
static uint32_t stats_written = 0;
static uint32_t stats_success = 0;
static uint32_t stats_failed = 0;
static uint32_t stats_flushed = 0;
static uint32_t stats_stalls = 0;
static int64_t stats_start = 0;

// ESB busy this long with no TX event is stuck (four attempts take under 3 ms)
#define STALL_MS 20
static int64_t last_tx_event = 0;

// The radio and ESB's timer (TIMER2) at the last stall, for stats
static struct
{
	bool valid, timer_running;
	uint32_t radio_state, radio_shorts, radio_int, ev_ready, ev_address, ev_end, ev_disabled;
	uint32_t timer_shorts, cc[3], count, bitmode, ppi_chen;
} stall;

static void stall_snapshot(void)
{
	stall.radio_state = NRF_RADIO->STATE;
	stall.radio_shorts = NRF_RADIO->SHORTS;
	stall.radio_int = NRF_RADIO->INTENSET;
	stall.ev_ready = NRF_RADIO->EVENTS_READY;
	stall.ev_address = NRF_RADIO->EVENTS_ADDRESS;
	stall.ev_end = NRF_RADIO->EVENTS_END;
	stall.ev_disabled = NRF_RADIO->EVENTS_DISABLED;
	stall.timer_shorts = NRF_TIMER2->SHORTS;
	for (int i = 0; i < 3; i++)
		stall.cc[i] = NRF_TIMER2->CC[i];
	NRF_TIMER2->TASKS_CAPTURE[3] = 1; // (CC3 is free: ERRATA 216 only)
	uint32_t t = NRF_TIMER2->CC[3];
	stall.count = t;
	stall.bitmode = NRF_TIMER2->BITMODE;
	k_busy_wait(20);
	NRF_TIMER2->TASKS_CAPTURE[3] = 1;
	stall.timer_running = NRF_TIMER2->CC[3] != t;
	stall.ppi_chen = NRF_PPI->CHEN;
	stall.valid = true;
}

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 30
#define TX_ERROR_MAX 100
#define TX_ERROR_CLEAR_RATE 10

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, ESB_THREAD_PRIORITY, 0, 0);

uint64_t pairing_packets = 0;

//|type    |description
//|TX  CRC8|pairing
//|RX  CRC8|pairing

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|TX  CRC8|ack     |device_addr                                          |
//|RX  CRC8|ack     |recv_addr                                            |

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		rift_airtime_done(true, event->tx_attempts);
		last_tx_event = k_uptime_get();
		stats_success++;
		if (!paired_addr[0]) // zero, not paired
			pairing_packets++; // keep track of pairing state
		if (tx_errors >= TX_ERROR_THRESHOLD && tx_errors < TX_ERROR_THRESHOLD + TX_ERROR_CLEAR_RATE && last_tx_fail == 0)
		{
			last_tx_success = 0; // reset last_tx_success on threshold reached
			last_tx_fail = k_uptime_get();
		}
		if (tx_errors > TX_ERROR_CLEAR_RATE)
			tx_errors -= TX_ERROR_CLEAR_RATE;
		else
			tx_errors = 0;
		LOG_DBG("TX SUCCESS");
		if (esb_paired)
			clocks_stop();
		break;
	case ESB_EVENT_TX_FAILED:
		rift_airtime_done(false, event->tx_attempts);
		last_tx_event = k_uptime_get();
		stats_failed++;
		if (tx_errors < TX_ERROR_MAX)
			tx_errors++;
		if (tx_errors == TX_ERROR_THRESHOLD && last_tx_success == 0) // consecutive failure to transmit
		{
			last_tx_fail = 0; // reset last_tx_fail on threshold reached
			last_tx_success = k_uptime_get();
		}
		LOG_DBG("TX FAILED");
		if (esb_paired)
			clocks_stop();
		break;
	case ESB_EVENT_RX_RECEIVED:
		// TODO: have to read rx until -ENODATA (or -EACCES/-EINVAL)
		if (!esb_read_rx_payload(&rx_payload)) // zero, rx success
		{
			LOG_DBG("rx len: %d, pipe: %d", rx_payload.length, rx_payload.pipe);
			if (!paired_addr[0]) // zero, not paired
			{
				LOG_DBG("tx: %16llX rx: %16llX", *(uint64_t *)tx_payload_pair.data, *(uint64_t *)rx_payload.data);
				if (rx_payload.length == 8 && tx_payload_pair.data[1] == 1) // ack to second packet in pairing burst
					memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
			}
			else
			{
				if (rx_payload.length == 4)
				{
					// TODO: Device should never receive packets if it is already paired, why is this packet received?
					// This may be part of acknowledge
//					if (!nrfx_timer_init_check(&m_timer))
					{
						LOG_WRN("Timer not initialized");
						break;
					}
					if (timer_state == false)
					{
//						nrfx_timer_resume(&m_timer);
						timer_state = true;
					}
//					nrfx_timer_clear(&m_timer);
					last_reset = 0;
					led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1]; // sync led flashes :)
					led_clock_offset = 0;
					LOG_DBG("RX, timer reset");
				}
				else if (rx_payload.length >= RIFT_ACK_LEN) // the receiver's timestamp: this packet's arrival on the beacon clock (and maybe a command)
				{
					rift_ts_ack(rx_payload.data, rx_payload.length, event->tx_attempts);
				}
			}
		}
		break;
	}
}

bool clock_status = false;

#if defined(CONFIG_CLOCK_CONTROL_NRF)
static struct onoff_manager *clk_mgr;

static int clocks_init(void)
{
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENOTSUP;
	}

	return 0;
}

SYS_INIT(clocks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int clocks_start(void)
{
	if (clock_status)
		return 0;
	int err;
	int res;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		k_usleep(100);
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10) {
			LOG_WRN_ONCE("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	LOG_DBG("HF clock started");
	clock_status = true;
	return 0;
}

void clocks_stop(void)
{
	if (!clock_status)
		return;
	clock_status = false;

	onoff_release(clk_mgr);

	LOG_DBG("HF clock stop request");
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

static struct k_thread clocks_thread_id;
static K_THREAD_STACK_DEFINE(clocks_thread_id_stack, 128);

void clocks_request_start(uint32_t delay_us)
{
	k_thread_create(&clocks_thread_id, clocks_thread_id_stack, K_THREAD_STACK_SIZEOF(clocks_thread_id_stack), (k_thread_entry_t)clocks_start, NULL, NULL, NULL, CLOCKS_START_THREAD_PRIORITY, 0, K_USEC(delay_us));
}

static struct k_thread clocks_stop_thread_id;
static K_THREAD_STACK_DEFINE(clocks_stop_thread_id_stack, 128);

void clocks_request_stop(uint32_t delay_us)
{
	k_thread_create(&clocks_stop_thread_id, clocks_stop_thread_id_stack, K_THREAD_STACK_SIZEOF(clocks_stop_thread_id), (k_thread_entry_t)clocks_stop, NULL, NULL, NULL, CLOCKS_STOP_THREAD_PRIORITY, 0, K_USEC(delay_us));
}

// this was randomly generated
// TODO: I have no idea?
// TODO: see esb information, check CONFIG_ESB_PIPE_COUNT
/*
base_addr_p0: Base address for pipe 0, in big endian.
base_addr_p1: Base address for pipe 1-7, in big endian.
pipe_prefixes: Address prefix for pipe 0 to 7.
*/
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8}; // TODO: not used
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

int esb_initialize(bool tx)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_2_SETTINGS_READ(CONFIG_2_RADIO_TX_POWER);
		// config.retransmit_delay = 600;
		//config.retransmit_count = 0;
		//config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true; // TODO: while pairing, should be set to false
		config.use_fast_ramp_up = IS_ENABLED(CONFIG_RIFT_FAST_RAMP_UP); // the receiver must match
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_2_SETTINGS_READ(CONFIG_2_RADIO_TX_POWER);
		// config.retransmit_delay = 600;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = IS_ENABLED(CONFIG_RIFT_FAST_RAMP_UP);
	}

	err = esb_init(&config);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

void esb_deinitialize(void)
{
	if (esb_initialized)
	{
		esb_initialized = false;
		k_msleep(10); // wait for pending transmissions
		esb_disable();
	}
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Recreate receiver address
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = paired_addr[7] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
//	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
//	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

void esb_set_pair(uint64_t addr)
{
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, device_addr, 6);
	uint8_t checksum = crc8_ccitt(0x07, buf, 6);
	if (checksum == 0)
		checksum = 8;
	if ((addr & 0xFF) != checksum)
	{
		LOG_INF("Incorrect checksum");
		return;
	}
	esb_reset_pair();
	memcpy(paired_addr, &addr, sizeof(paired_addr));
	LOG_INF("Paired");
	sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
}

void esb_pair(void)
{
	if (get_status(SYS_STATUS_CONNECTION_ERROR))
		set_status(SYS_STATUS_CONNECTION_ERROR, false);
	tx_errors = 0;
	if (!paired_addr[0]) // zero, no receiver paired
	{
		LOG_INF("Pairing");
		esb_set_addr_discovery();
		esb_initialize(true);
//		timer_init(); // TODO: shouldn't be here!!!
		tx_payload_pair.pipe = 0;
		tx_payload_pair.noack = false;
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		memcpy(&tx_payload_pair.data[2], addr, 6);
		LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
		uint8_t checksum = crc8_ccitt(0x07, &tx_payload_pair.data[2], 6);
		if (checksum == 0)
			checksum = 8;
		LOG_INF("Checksum: %02X", checksum);
		tx_payload_pair.data[0] = checksum; // Use checksum to make sure packet is for this device
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_PAIR);
		while (paired_addr[0] != checksum)
		{
			int64_t time_begin = k_uptime_get();

			if (!esb_initialized)
			{
				esb_set_addr_discovery();
				esb_initialize(true);
			}

			if (!clock_status)
				clocks_start();

			if (paired_addr[0])
			{
				LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0; // Packet not for this device
			}

//			esb_flush_rx();
//			esb_flush_tx();

			pairing_packets = 0;

			// send pairing request
			tx_payload_pair.data[1] = 0;
			esb_write_payload(&tx_payload_pair);
			k_usleep(100);
			while (!esb_is_idle() && (k_uptime_get() < (time_begin + 10)))
				k_usleep(1);

			// receive ack data
			tx_payload_pair.data[1] = 1;
			if ((pairing_packets == 1) && (k_uptime_get() < (time_begin + 10)))
				esb_write_payload(&tx_payload_pair);
			k_usleep(100);
			while (!esb_is_idle() && (k_uptime_get() < (time_begin + 10)))
				k_usleep(1);

			if (pairing_packets == 2)
				LOG_INF("Pairing request received");

			if (clock_status)
				clocks_stop();

			int64_t time_delta = k_uptime_get() - time_begin;
			if (time_delta > 1000)
				k_yield();
			else
				k_msleep(1000 - time_delta);
		}
		set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_PAIR);
		LOG_INF("Paired");
		sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		esb_deinitialize();
		k_msleep(1600); // wait for led pattern
	}
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Receiver address: %012llX", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);

	connection_set_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_paired = true;
	clocks_stop();
}

void esb_reset_pair(void)
{
	if (paired_addr[0] || esb_paired)
	{
		esb_deinitialize(); // make sure esb is off
		esb_paired = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		LOG_INF("Pairing requested");
	}
}

void esb_clear_pair(void)
{
	esb_reset_pair();
	sys_write(PAIRED_ID, &retained->paired_addr, paired_addr, sizeof(paired_addr)); // write zeroes
	LOG_INF("Pairing data reset");
}

void esb_write(uint8_t *data)
{
	esb_write_len(data, 21);
}

// The connection thread and the console (airtime b) both write
static K_MUTEX_DEFINE(write_lock);

static void esb_write_common(const uint8_t *data, uint8_t length, bool noack)
{
	if (!esb_initialized || !esb_paired)
		return;
	if (!clock_status)
		clocks_start();
	k_mutex_lock(&write_lock, K_FOREVER);
	tx_payload.pipe = 1; // using base address 1
#if defined(NRF54L15_XXAA) // TODO: esb halts with ack and tx fail
	tx_payload.noack = true;
#else
	tx_payload.noack = noack;
#endif
	tx_payload.length = length;
	memcpy(tx_payload.data, data, length);
	// The connection thread writes some packets back to back: give the last one time for its
	// retries (3 at 600 us, a beacon slot can cost one) before flushing it
	int64_t wait_until = k_uptime_get() + 3;
	while (!esb_is_idle() && k_uptime_get() < wait_until)
		k_usleep(50);
	bool busy = !esb_is_idle();
	if (busy && k_uptime_get() - last_tx_event > STALL_MS)
	{
		// Busy with no TX event for so long: a transfer ESB can't finish (a lost ACK
		// with no timeout running, see esb/esb.c). Start ESB again
		stats_stalls++;
		stall_snapshot();
		esb_initialize(true);
		last_tx_event = k_uptime_get();
		busy = !esb_is_idle();
	}
	if (busy)
		stats_flushed++;
	esb_flush_tx(); // this will clear all transmissions even if they did not complete
	rift_airtime_start(length, tx_payload.noack, busy);
	esb_write_payload(&tx_payload); // Add transmission to queue
	stats_written++;
	send_data = true;
	k_mutex_unlock(&write_lock);
}

void esb_write_len(const uint8_t *data, uint8_t length)
{
	esb_write_common(data, length, false);
}

void esb_write_noack(const uint8_t *data, uint8_t length)
{
	esb_write_common(data, length, true);
}

bool esb_ready(void)
{
	return esb_initialized && esb_paired;
}

void esb_print_stats(void)
{
	int64_t now = k_uptime_get();
	int64_t ms = now - stats_start;
	printk("Over %lld ms: %u written, %u acked, %u failed, %u flushed while busy, %u stalls restarted\n",
		   ms, stats_written, stats_success, stats_failed, stats_flushed, stats_stalls);
	stats_written = 0;
	stats_success = 0;
	stats_failed = 0;
	stats_flushed = 0;
	stats_stalls = 0;
	stats_start = now;
	if (stall.valid)
		printk("Last stall: radio state %u shorts %03x int %03x events ready %u address %u end %u disabled %u; "
			   "TIMER2 %s at %u (bitmode %u) shorts %04x cc0 %u cc1 %u cc2 %u; PPI %08x\n",
			   stall.radio_state, stall.radio_shorts, stall.radio_int, stall.ev_ready, stall.ev_address,
			   stall.ev_end, stall.ev_disabled, stall.timer_running ? "running" : "stopped", stall.count,
			   stall.bitmode, stall.timer_shorts, stall.cc[0], stall.cc[1], stall.cc[2], stall.ppi_chen);
}

static void esb_thread(void)
{
	bool use_hid = CONFIG_0_SETTINGS_READ(CONFIG_0_CONNECTION_OVER_HID);
	bool use_shutdown = CONFIG_0_SETTINGS_READ(CONFIG_0_USER_SHUTDOWN);
	int64_t start_time = k_uptime_get();

	// Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	while (1)
	{
		if (!esb_paired && (!use_hid || paired_addr[0] || (!get_status(SYS_STATUS_USB_CONNECTED) && k_uptime_get() - 750 > start_time))) // only automatically enter pairing while not potentially communicating by usb, however allow esb if already paired
		{
			esb_pair();
			esb_initialize(true);
		}
		if (tx_errors >= TX_ERROR_THRESHOLD)
		{
			if (!get_status(SYS_STATUS_CONNECTION_ERROR) && (!use_hid || !get_status(SYS_STATUS_USB_CONNECTED))) // only raise error while not potentially communicating by usb
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
			if (use_shutdown && k_uptime_get() - last_tx_success > CONFIG_3_SETTINGS_READ(CONFIG_3_CONNECTION_TIMEOUT_DELAY)) // shutdown if receiver is not detected // TODO: is shutdown necessary if usb is connected at the time?
			{
				LOG_WRN("No response from receiver in %dm", CONFIG_3_SETTINGS_READ(CONFIG_3_CONNECTION_TIMEOUT_DELAY) / 60000);
				sys_request_system_off(false);
			}
		}
		else if (tx_errors < TX_ERROR_THRESHOLD && get_status(SYS_STATUS_CONNECTION_ERROR) && k_uptime_get() - last_tx_fail > 3000) // TODO: there is possibly some race condition causing tx_error to potentially be above zero more often than not, so the check is more lenient; tx_error under threshold and last errors above threshold was not recent
		{
			set_status(SYS_STATUS_CONNECTION_ERROR, false);
		}
		k_msleep(100);
	}
}
