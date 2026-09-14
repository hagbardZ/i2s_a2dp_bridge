#include <Arduino.h>
#include <driver/i2s.h>
#include <BluetoothA2DPSource.h>
#include <freertos/stream_buffer.h>
#include <Adafruit_NeoPixel.h>
#include <nvs.h>
#include <nvs_flash.h>

// ── M5 Echo I2S RX pin mapping (retro-go input) ──────────────
#define PIN_RX_BCK      19
#define PIN_RX_WS       33
#define PIN_RX_DATA     22

#define PIN_BTN         39  // press this button to pair a new device
#define PIN_LED         27      // SK6812 RGB LED (NEOPIXEL)
#define PIN_STATUS_LED  12      // plain status LED (flash rate = BT state)
// PLAIN LED STATE
// connected = ON
// connecting = medium blink 250ms
// Disconnecting = slow blink 500ms
// Disconneced= fast blink 100ms



// ── Bluetooth ─────────────────────────────────────────────────
#define BT_DEVICE_NAME  "MARTENDO32-BLUE"

// ── Fixed sample rate (optional) ──────────────────────────────
// Uncomment and set to lock the software to a single sample rate
// (e.g. 44100, 48000). When defined, the input-rate scanner is
// disabled and the software only works at the given rate.
// When commented out, the software auto-detects the input rate.
//#define FIXED_SAMPLE_RATE 44100

// ── Tuning ────────────────────────────────────────────────────
#define A2DP_BUF_SIZE       16384
#define I2S_READ_BYTES      2048

// ── Globals ───────────────────────────────────────────────────
static BluetoothA2DPSource a2dp_source;
static StreamBufferHandle_t a2dp_buf;
static Adafruit_NeoPixel   led(1, PIN_LED, NEO_GRB + NEO_KHZ800);

#ifdef FIXED_SAMPLE_RATE
static volatile uint32_t input_hz            = FIXED_SAMPLE_RATE;
#define SINK_TARGET_HZ  ((float)FIXED_SAMPLE_RATE)
#else
static volatile uint32_t input_hz            = 44100;   // live-measured input rate (smoothed)
#define SINK_TARGET_HZ  44100.0f                          // nominal sink rate
#endif
static volatile bool     audio_detected      = false;
static volatile bool     user_disconnect_req  = false;  // set when button forces a switch
static esp_bd_addr_t     current_peer;                  // MAC of the sink we are paired to
static volatile esp_a2d_connection_state_t bt_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
static volatile uint32_t measured_rate = 0;
static volatile uint32_t sink_frames_consumed = 0;      // total frames pulled by sink callback
static volatile uint32_t input_frames_since_print = 0;
static int16_t           read_buf[I2S_READ_BYTES / 2];
static int16_t           resampled[4096];
static float             resample_step;                 // input frames per output frame (adaptive)
static float             src_pos = 0.0f;                // resampler phase (input frames)
static float             servo_scale = 1.0f;            // buffer-level rate trim (1.0 = nominal)

// ── Last-device persistence ───────────────────────────────────
// Persist the current sink's address to NVS under the exact namespace/key the
// ESP32-A2DP library reads on boot ("connected_bda"/"src_bda"). This is needed
// because a fresh-discovery connection (button) never writes NVS itself, so
// "last_connection" stays stale across reboots (e.g. M5 restarts straight back
// to an old speaker instead of the headphones you last used).
static void persist_last_connection(const esp_bd_addr_t &bda) {
    nvs_handle h;
    if (nvs_open("connected_bda", NVS_READWRITE, &h) == ESP_OK) {
        if (nvs_set_blob(h, "src_bda", bda, ESP_BD_ADDR_LEN) == ESP_OK)
            nvs_commit(h);
        nvs_close(h);
    }
}

// ── LED status ────────────────────────────────────────────────
static void led_set(uint8_t r, uint8_t g, uint8_t b) {
    led.setPixelColor(0, led.Color(r, g, b));
    led.show();
}

static void led_update(void) {
    // Fading pulse for "searching" states
    static uint8_t pulse = 0;
    static int8_t  pulse_dir = 4;
    pulse += pulse_dir;
    if (pulse >= 128 || pulse == 0) pulse_dir = -pulse_dir;

    switch (bt_state) {
        case ESP_A2D_CONNECTION_STATE_CONNECTED:
            if (audio_detected)
                led_set(0, 64, 0);                         // green — streaming
            else
                led_set(0, 0, 64);                         // blue — connected, no audio
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            led_set(pulse, pulse / 2, 0);                  // orange pulse — connecting
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
            led_set(pulse / 2, 0, pulse / 2);              // purple pulse — disconnecting
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
        default:
            led_set(pulse, 0, 0);                          // red pulse — searching
            break;
    }
}

// ── Plain status LED (GPIO13) ────────────────────────────────
// Connected = steady ON; otherwise the flash rate encodes the state:
//   connecting   -> medium blink  (250 ms)
//   disconnecting-> slow blink    (500 ms)
//   disconnected -> fast blink    (100 ms)
// Called every loop iteration; millis()-based so it never blocks audio.
static void status_led_update(void) {
    static uint32_t last_toggle = 0;
    static bool     on = false;
    bool     steady_on = false;
    uint32_t period_ms = 0;

    switch (bt_state) {
        case ESP_A2D_CONNECTION_STATE_CONNECTED:
            steady_on = true;
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            period_ms = 250;
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
            period_ms = 500;
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
        default:
            period_ms = 100;
            break;
    }

    if (steady_on) {
        if (!on) {
            on = true;
            digitalWrite(PIN_STATUS_LED, HIGH);
        }
    } else if (millis() - last_toggle >= period_ms) {
        last_toggle = millis();
        on = !on;
        digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
    }
}

// ── A2DP source callback (runs on BT task, core 0) ───────────
static int32_t a2dp_callback(uint8_t *data, int32_t length)
{
    // The A2DP stack may invoke this with a NULL/length-0 request (e.g. on
    // media stop or during reconfiguration). xStreamBufferReceive asserts on
    // a NULL destination, which crashed the M5 mid-stream — guard it here.
    if (data == nullptr || length <= 0)
        return 0;
    int32_t got = xStreamBufferReceive(a2dp_buf, data, length, pdMS_TO_TICKS(5));
    if (got < 0) got = 0;
    // Always hand the SBC encoder a COMPLETE PCM buffer. A short read would
    // become a truncated SBC frame -> audible clicks. Pad the remainder with
    // silence so the media task can never underrun or glitch.
    if (got < length)
        memset(data + got, 0, length - got);
    sink_frames_consumed += length / 4;
    return length;
}

// ── Bluetooth connection callbacks ────────────────────────────
static void on_bt_state_changed(esp_a2d_connection_state_t state, void*) {
    bt_state = state;
    Serial.printf("[BT] State: %s\n",
                  state == ESP_A2D_CONNECTION_STATE_CONNECTED    ? "Connected" :
                  state == ESP_A2D_CONNECTION_STATE_DISCONNECTED ? "Disconnected" :
                  state == ESP_A2D_CONNECTION_STATE_CONNECTING   ? "Connecting" :
                  state == ESP_A2D_CONNECTION_STATE_DISCONNECTING? "Disconnecting" : "?");
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        Serial.printf("[BT] Negotiated sink sample rate: %d Hz\n",
                      a2dp_source.get_source_sample_rate());
        // Re-enable auto-reconnect now that we are paired with a real sink, so
        // a future drop (or reboot) healthily reconnects to this device. This
        // is deliberately NOT done in the button handler (see there for why).
        a2dp_source.set_auto_reconnect(true);
        // persist the current sink so a reboot resumes THIS device instead of
        // whatever stale address the library kept in NVS (boot auto-reconnect
        // reads exactly this blob).
        persist_last_connection(current_peer);
        // A fresh connection completes a user-initiated device switch, so we
        // can safely arm the drop-reconnect logic again.
        user_disconnect_req = false;
        // Flush stale pre-connect audio so the buffer starts clean
        uint8_t b;
        while (xStreamBufferReceive(a2dp_buf, &b, 1, 0) != 0) {}
    } else if (state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
        // The button's fresh discovery is now actively trying a new sink; the
        // user-switch intent is satisfied, so subsequent drops are real drops
        // and should auto-reconnect.
        user_disconnect_req = false;
    } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        // A connection dropped. If this was NOT a user-initiated switch (button
        // press), kick off the library's native reconnect to the last device.
        // Discovery-initiated connections never arm the library's internal
        // is_autoreconnect_allowed flag, so without this the M5 would sit in
        // "waiting" forever after e.g. a Philips power-cycle. reconnect() arms
        // that flag and retries connect_to(last_connection), which is exactly
        // the path that already works for the MS425.
        if (!user_disconnect_req) {
            Serial.println("[BT] Sink dropped - reconnecting to last device...");
            a2dp_source.reconnect();
        }
    }
}

// ── Bluetooth discovery filter ────────────────────────────────
// Called for every A2DP-compatible device found during inquiry. The library
// already filters out incompatible classes-of-device before invoking this, so
// returning true just accepts the first suitable sink (headphones, speaker...).
static bool on_ssid(const char *ssid, esp_bd_addr_t address, int rssi) {
    char mac[18];
    snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x",
             address[0], address[1], address[2],
             address[3], address[4], address[5]);
    Serial.printf("[SSID] Found sink: \"%s\" (%s) RSSI=%d\n", ssid, mac, rssi);
    memcpy(current_peer, address, ESP_BD_ADDR_LEN);
    return true;
}

// ── Setup ─────────────────────────────────────────────────────
void setup(void)
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== M5 Echo  I2S -> BT Speaker ===");
    Serial.printf("RX pins: BCK=%d  WS=%d  DATA=%d\n",
                  PIN_RX_BCK, PIN_RX_WS, PIN_RX_DATA);

    pinMode(PIN_BTN, INPUT_PULLUP);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    led.begin();
    led.setBrightness(32);
    led_set(64, 0, 0);  // red while booting

    a2dp_buf = xStreamBufferCreate(A2DP_BUF_SIZE, 256);

    // I2S0 RX — slave mode (retro-go provides BCK & WS)
    i2s_config_t rx_cfg = {};
    rx_cfg.mode                 = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
#ifdef FIXED_SAMPLE_RATE
    rx_cfg.sample_rate          = FIXED_SAMPLE_RATE;
#else
    rx_cfg.sample_rate          = 44100;
#endif
    rx_cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    rx_cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    rx_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    rx_cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    rx_cfg.dma_buf_count        = 8;
    rx_cfg.dma_buf_len          = 512;
    rx_cfg.use_apll             = false;
    rx_cfg.tx_desc_auto_clear   = false;
    rx_cfg.fixed_mclk           = 0;

    i2s_pin_config_t rx_pins = {};
    rx_pins.mck_io_num     = I2S_PIN_NO_CHANGE;
    rx_pins.bck_io_num     = PIN_RX_BCK;
    rx_pins.ws_io_num      = PIN_RX_WS;
    rx_pins.data_out_num   = I2S_PIN_NO_CHANGE;
    rx_pins.data_in_num    = PIN_RX_DATA;

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &rx_cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &rx_pins));

    // Bluetooth A2DP Source
    a2dp_source.set_data_callback(a2dp_callback);
    a2dp_source.set_on_connection_state_changed(on_bt_state_changed);
    a2dp_source.set_ssid_callback(on_ssid);
    a2dp_source.set_auto_reconnect(true);
    a2dp_source.start(BT_DEVICE_NAME);

    Serial.println("[Ready] Waiting for retro-go I2S audio...\n");
}

// ── Main loop (core 1) ───────────────────────────────────────
void loop(void)
{
    // Read PCM from retro-go via I2S RX
    size_t  bytes_read = 0;
    int in_frames = 0, out_frames = 0;
    i2s_read(I2S_NUM_0, read_buf, sizeof(read_buf), &bytes_read, pdMS_TO_TICKS(10));

    if (bytes_read > 0) {
        in_frames = bytes_read / 4;
        input_frames_since_print += in_frames;

#ifndef FIXED_SAMPLE_RATE
        // Live input-rate measurement (1 s window), low-pass filtered to stop
        // per-second quantization jitter from pitching the audio. retro-go
        // switches format (48k / 44.1k / lower), so we adapt to what's present.
        static uint32_t rate_win_start = 0;
        static uint32_t rate_win_frames = 0;
        if (rate_win_start == 0) rate_win_start = millis();
        rate_win_frames += in_frames;
        audio_detected = true;
        if (millis() - rate_win_start >= 1000) {
            uint32_t hz = rate_win_frames * 1000UL / (millis() - rate_win_start);
            if (hz >= 8000 && hz <= 96000) {
                bool big_jump =
                    (input_hz > hz ? input_hz - hz : hz - input_hz) > 2000;
                if (big_jump) {
                    // Format switch: snap instantly and reset the resampler
                    // phase. Keep the buffer level (flushing would starve the
                    // sink, since input and sink run at the same rate once the
                    // new format is active -> empty buffer never rebuilds).
                    input_hz = hz;
                    src_pos = 0.0f;
                } else {
                    input_hz = (uint32_t)(0.5f * input_hz + 0.5f * hz);
                }
            }
            rate_win_frames = 0;
            measured_rate = hz;
            rate_win_start = millis();
        }
#else
        audio_detected = true;
#endif

        // Step: input frames per output frame, to reach the nominal sink rate,
        // trimmed by the buffer-level servo (servo_scale) so the buffer stays
        // near target without clicky drop/duplicate bursts.
        resample_step = (float)input_hz / SINK_TARGET_HZ * servo_scale;

        // Phase-continuous linear-interpolation resampler. src_pos is a global
        // fractional input position carried across blocks for continuity.
        const int max_out = 2047;
        int out_frames = 0;
        while (out_frames < max_out) {
            int idx = (int)src_pos;
            if (idx >= in_frames - 1) break;
            float frac = src_pos - idx;
            resampled[out_frames*2]   = (int16_t)(read_buf[idx*2]   * (1.0f - frac) +
                                                  read_buf[(idx+1)*2] * frac);
            resampled[out_frames*2+1] = (int16_t)(read_buf[idx*2+1] * (1.0f - frac) +
                                                  read_buf[(idx+1)*2+1] * frac);
            out_frames++;
            src_pos += resample_step;
        }
        src_pos -= in_frames;                // carry remainder into next input block

        if (out_frames > 0) {
            // Small timeout instead of 0: if the buffer is momentarily full we
            // wait for the sink to drain a little rather than throwing away an
            // entire block (which caused ~46ms audio gaps).
            xStreamBufferSend(a2dp_buf, resampled, out_frames * 4, pdMS_TO_TICKS(20));
        }
    }

    // Rate servo — replaces the old drop/duplicate burst guard. It nudges the
    // resampler ratio very slightly to keep the buffer near target. Correcting
    // 128-frame bursts repeatedly produces clicks; a slow (<0.1%) change in the
    // interpolation ratio is inaudible.
    static float servo_integr = 0.0f;
    static bool  servo_inited = false;
    static uint32_t last_servo = 0;
    if (millis() - last_servo >= 200) {
        int fill = (int)xStreamBufferBytesAvailable(a2dp_buf);
        int target = (int)(A2DP_BUF_SIZE * 70 / 100);          // 70% cushion
        float error = (float)(fill - target) / (float)A2DP_BUF_SIZE; // -1..+1
        if (!servo_inited) {
            servo_integr = 0.0f;
            servo_inited = true;
        }
        servo_integr += 0.00004f * error;                      // slow integral
        if (servo_integr >  0.02f) servo_integr =  0.02f;
        if (servo_integr < -0.02f) servo_integr = -0.02f;
        float factor = 1.0f + servo_integr + 0.0008f * error;  // small proportional
        if (factor > 1.05f) factor = 1.05f;
        if (factor < 0.95f) factor = 0.95f;
        servo_scale = factor;
        last_servo = millis();
    }

    // Button → reconnect BT: force a fresh discovery (instead of always
    // reconnecting to the last-paired device). auto-reconnect must be off
    // during the scan so start() performs an inquiry instead of connecting
    // straight to the stored address. Re-enable it once a sink is found.
    if (digitalRead(PIN_BTN) == LOW) {
        delay(50);
        if (digitalRead(PIN_BTN) == LOW) {
            Serial.println("[BTN] Reconnecting BT (fresh discovery)...");
            // Mark this as a user-initiated switch so the DISCONNECTED callback
            // doesn't immediately auto-reconnect to the device we are leaving.
            // Cleared below once the new discovery connection is established.
            user_disconnect_req = true;
            // Do a clean full restart of the A2DP source. A bare
            // disconnect()+start() can leave the library's internal state
            // machine stuck in DISCOVERING when the old sink completes its
            // connection mid-scan, so the CONNECTED event gets dropped and
            // the media stream never starts (sinkHz stays 0). end(false)
            // tears down and resets A2DP/state without turning off the
            // Bluetooth controller, and clears last_connection so start()
            // does a true inquiry instead of bouncing back to a saved device.
            a2dp_source.set_auto_reconnect(false);
            a2dp_source.end(false);
            delay(500);
            a2dp_source.start(BT_DEVICE_NAME);
            // NOTE: user_disconnect_req stays true through discovery and is only
            // cleared once a new sink reports CONNECTED (in on_bt_state_changed),
            // so any stale DISCONNECTED events from the teardown won't trigger an
            // unwanted reconnect() to the device we just left.
            while (digitalRead(PIN_BTN) == LOW) delay(10);
        }
    }

    // Periodic status + LED refresh (moved out of the hot loop so the
    // NeoPixel/RMT update doesn't interfere with the audio pipeline)
    static uint32_t last_print = 0;
    static uint32_t last_sink  = 0;
    if (millis() - last_print > 1000) {
        led_update();
        uint32_t dt = millis() - last_print;
        uint32_t sinkHz = (sink_frames_consumed - last_sink) * 1000UL / dt;
        uint32_t inHz   = input_frames_since_print * 1000UL / dt;
        Serial.printf("inHz=%u sinkHz=%u | buf: %d/%d | sv=%d.%04d | BT: %s\n",
                      inHz, sinkHz,
                      (int)xStreamBufferBytesAvailable(a2dp_buf),
                      (int)A2DP_BUF_SIZE,
                      (int)servo_scale,
                      (int)(servo_scale * 10000) % 10000,
                      bt_state == ESP_A2D_CONNECTION_STATE_CONNECTED ? "Connected" : "waiting");
        input_frames_since_print = 0;
        last_sink  = sink_frames_consumed;
        last_print = millis();
    }

    delay(1);
    status_led_update();
}
