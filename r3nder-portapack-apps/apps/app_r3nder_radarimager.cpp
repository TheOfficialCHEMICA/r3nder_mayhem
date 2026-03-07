/*
 * App_R3nderRadarImager — FMCW Range-Time Radar Imager
 * Hardware: HackRF One + PortaPack H2+  (Mayhem firmware)
 *
 * Technique: Time-multiplexed FMCW
 *   1. TX a linear-FM chirp of bandwidth BW over duration T_c via HackRF TX
 *   2. Switch HackRF to RX; capture echo window into IQ buffer
 *   3. De-ramp: multiply echo by conjugate of reference chirp (stretch processing)
 *   4. FFT of beat signal  →  range profile  (bin k  ≈  k * c / (2*BW))
 *   5. Scroll new profile row into 240×256 waterfall on H2+ ILI9341 display
 *
 * HackRF One specifics (half-duplex workaround):
 *   - TX chirp burst ~1 ms  (20 000 samples @ 20 Msps)
 *   - 5 µs blanking guard (antenna switch settle)
 *   - RX echo window   ~2 ms  (40 000 samples, captures up to ~300 m two-way)
 *   - Capture only FFT_N=512 samples from RX window for range processing
 *   - Sample rate: 20 Msps (max HackRF BW)
 *   - TX VGA gain default: 20 dB  (0–47 dB, 1 dB step)
 *   - RX LNA gain default: 32 dB  (0–40 dB, 8 dB step)
 *   - RX VGA gain default: 20 dB  (0–62 dB, 2 dB step)
 *
 * PortaPack H2+ display layout (240 × 320, portrait, ILI9341):
 *   Row   0– 15  : status bar  (freq, gain, sweep count)
 *   Row  16–271  : waterfall   (256 rows × 240 cols, grayscale, range→time)
 *   Row 272–303  : range profile bar (magnitude vs. range bin, live)
 *   Row 304–319  : footer (range resolution, max range)
 *
 * Controls (H2+ hardware):
 *   Rotary encoder : ±100 MHz steps on center frequency
 *   NAV UP / DN    : ±8 dB LNA gain
 *   NAV LEFT/RIGHT : ±2 dB VGA gain
 *   SELECT (push)  : start / stop sweep
 */

#include "app.h"
#include "ui/ui.h"
#include "hackrf/receiver.h"
#include "hackrf/transmitter.h"

#include <cstdint>
#include <cstring>
#include <cmath>
#include <complex>

// ── FMCW / hardware constants ─────────────────────────────────────────────────

static constexpr uint64_t RADAR_FREQ_DEFAULT_HZ = 5800000000ULL; // 5.8 GHz ISM
static constexpr uint64_t RADAR_FREQ_MIN_HZ     =  100000000ULL; // 100 MHz
static constexpr uint64_t RADAR_FREQ_MAX_HZ     = 6000000000ULL; // 6.0 GHz
static constexpr uint64_t RADAR_FREQ_STEP_HZ    =  100000000ULL; // 100 MHz/step

static constexpr uint32_t SAMPLE_RATE_HZ  = 20000000; // 20 Msps (HackRF max)
static constexpr uint32_t CHIRP_BW_HZ     = 20000000; // LFM sweep = full 20 MHz BW

// Chirp duration = 1 ms  →  CHIRP_SAMPLES = 20 000
static constexpr uint32_t CHIRP_SAMPLES   = SAMPLE_RATE_HZ / 1000;

// FFT/range-profile parameters
static constexpr uint32_t FFT_N           = 512;
static constexpr uint32_t RANGE_BINS      = FFT_N / 2; // 256 positive-freq bins

// Range resolution:  ΔR = c / (2 · BW)  ≈  7.5 m per bin at 20 MHz
static constexpr float    SPEED_OF_LIGHT  = 299792458.0f;
static constexpr float    RANGE_RES_M     = SPEED_OF_LIGHT / (2.0f * CHIRP_BW_HZ);

// PortaPack H2+ display geometry
static constexpr uint32_t DISPLAY_W       = 240;
static constexpr uint32_t DISPLAY_H       = 320;
static constexpr uint32_t STATUS_H        = 16;
static constexpr uint32_t WATERFALL_H     = 256;
static constexpr uint32_t PROFILE_H       = 32;
static constexpr uint32_t FOOTER_H        = 16;

// Blanking guard samples (~5 µs) between TX end and RX start
static constexpr uint32_t BLANKING_SAMPS  = 100;

// ── Complex type ──────────────────────────────────────────────────────────────
using cx32 = std::complex<float>;

// ── Radix-2 DIT FFT (in-place, no heap allocation) ───────────────────────────

static void fft_bit_reverse(cx32* buf, uint32_t n) {
    for (uint32_t i = 1, j = 0; i < n; ++i) {
        uint32_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            cx32 tmp = buf[i];
            buf[i]   = buf[j];
            buf[j]   = tmp;
        }
    }
}

static void fft_inplace(cx32* buf, uint32_t n) {
    fft_bit_reverse(buf, n);
    for (uint32_t len = 2; len <= n; len <<= 1) {
        float ang    = -2.0f * 3.14159265358979f / static_cast<float>(len);
        cx32  wlen(cosf(ang), sinf(ang));
        for (uint32_t i = 0; i < n; i += len) {
            cx32 w(1.0f, 0.0f);
            for (uint32_t j = 0; j < len / 2; ++j) {
                cx32 u = buf[i + j];
                cx32 v = buf[i + j + len / 2] * w;
                buf[i + j]           = u + v;
                buf[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

// ── App class ─────────────────────────────────────────────────────────────────

class App_R3nderRadarImager : public App {
public:
    void init() override;
    void run()  override;
    void stop() override;

private:
    // ── signal chain ─────────────────────────────────────────────────────────
    void build_chirp_lut();
    void tx_chirp_burst();
    void rx_echo_window();
    void compute_range_profile();
    void apply_cfar();

    // ── display helpers (PortaPack H2+ 240×320 ILI9341) ──────────────────────
    void render_status();
    void render_waterfall_row();
    void render_profile_bar();
    void render_footer();

    // ── input handling ────────────────────────────────────────────────────────
    void handle_encoder(int delta);
    void handle_nav_up();
    void handle_nav_dn();
    void handle_nav_left();
    void handle_nav_right();
    void handle_select();

    // ── state ─────────────────────────────────────────────────────────────────
    uint64_t center_freq_hz_{ RADAR_FREQ_DEFAULT_HZ };
    uint8_t  tx_vga_db_     { 20 };  // HackRF TX VGA 0–47 dB
    uint8_t  rx_lna_db_     { 32 };  // HackRF RX LNA 0–40 dB (8 dB steps)
    uint8_t  rx_vga_db_     { 20 };  // HackRF RX VGA 0–62 dB (2 dB steps)
    bool     sweeping_      { false };
    uint32_t sweep_count_   { 0 };

    // TX chirp IQ look-up table: int8 interleaved I,Q (HackRF native format)
    int8_t   chirp_lut_[CHIRP_SAMPLES * 2];

    // Hanning window coefficients
    float    hann_[FFT_N];

    // De-ramped beat signal buffer (complex float)
    cx32     beat_buf_[FFT_N];

    // Range profile magnitudes after FFT, normalised 0–255
    uint8_t  range_mag_[RANGE_BINS];

    // Waterfall frame buffer [rows][cols] grayscale; rows scroll downward
    // Stored in circular buffer; wf_head_ points to oldest row (top of screen)
    uint8_t  waterfall_[WATERFALL_H][DISPLAY_W];
    uint32_t wf_head_{ 0 };
};

APP_FACTORY(App_R3nderRadarImager);

// ── init ──────────────────────────────────────────────────────────────────────

void App_R3nderRadarImager::init() {
    ui.title("Radar Imager");

    // Pre-compute Hanning window
    for (uint32_t n = 0; n < FFT_N; ++n)
        hann_[n] = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * n / (FFT_N - 1));

    // Pre-compute TX chirp LUT
    build_chirp_lut();

    // Clear display buffers
    memset(waterfall_, 0, sizeof(waterfall_));
    memset(range_mag_, 0, sizeof(range_mag_));

    render_footer();
    render_status();
    ui.label("SELECT to start sweep");
}

// ── run (called repeatedly by Mayhem firmware scheduler) ──────────────────────

void App_R3nderRadarImager::run() {
    // Poll PortaPack H2+ inputs
    int enc = ui.encoder_delta();
    if (enc != 0)              handle_encoder(enc);
    if (ui.nav_up_pressed())   handle_nav_up();
    if (ui.nav_dn_pressed())   handle_nav_dn();
    if (ui.nav_left_pressed()) handle_nav_left();
    if (ui.nav_right_pressed())handle_nav_right();
    if (ui.select_pressed())   handle_select();

    if (!sweeping_) return;

    // ── FMCW pulse-repetition loop (one PRI per run() call) ──────────────────
    tx_chirp_burst();        // ~1 ms  TX (HackRF TX path)
    rx_echo_window();        // ~2 ms  RX, fills beat_buf_[]
    compute_range_profile(); // FFT → range_mag_[]
    apply_cfar();            // suppress noise floor

    render_waterfall_row();  // add new row to waterfall
    render_status();         // update freq/gain/count header
    render_profile_bar();    // live magnitude bar at bottom

    ++sweep_count_;
}

// ── stop ──────────────────────────────────────────────────────────────────────

void App_R3nderRadarImager::stop() {
    sweeping_ = false;
    hackrf::transmitter::stop();
    hackrf::receiver::stop();
    ui.label("Radar Imager stopped");
}

// ── build_chirp_lut ───────────────────────────────────────────────────────────
// Linear FM (LFM) chirp:
//   phi(t) = pi * (BW/T_c) * t^2
//   s(t)   = 127 * [ cos(phi(t)) + j*sin(phi(t)) ]
// Packed as interleaved int8 I,Q (HackRF USB transfer format).

void App_R3nderRadarImager::build_chirp_lut() {
    const float T_c = static_cast<float>(CHIRP_SAMPLES) /
                      static_cast<float>(SAMPLE_RATE_HZ);          // 1 ms
    const float k   = static_cast<float>(CHIRP_BW_HZ) / T_c;      // chirp rate Hz/s
    const float fs  = static_cast<float>(SAMPLE_RATE_HZ);

    for (uint32_t n = 0; n < CHIRP_SAMPLES; ++n) {
        float t   = static_cast<float>(n) / fs;
        float phi = 3.14159265f * k * t * t;
        chirp_lut_[2 * n]     = static_cast<int8_t>(127.0f * cosf(phi)); // I
        chirp_lut_[2 * n + 1] = static_cast<int8_t>(127.0f * sinf(phi)); // Q
    }
}

// ── tx_chirp_burst ────────────────────────────────────────────────────────────
// Configure HackRF One TX path and send one LFM chirp burst.
// TX VGA gain is the only amplifier stage used (RF amp disabled by default).

void App_R3nderRadarImager::tx_chirp_burst() {
    hackrf::transmitter::set_frequency(center_freq_hz_);
    hackrf::transmitter::set_sample_rate(SAMPLE_RATE_HZ);
    hackrf::transmitter::set_txvga_gain(tx_vga_db_);
    hackrf::transmitter::send(chirp_lut_, CHIRP_SAMPLES); // blocks until done
    hackrf::transmitter::stop();
}

// ── rx_echo_window ────────────────────────────────────────────────────────────
// Switch HackRF to RX, skip BLANKING_SAMPS to let antenna switch settle,
// then capture FFT_N complex samples.  Perform stretch processing (de-ramp)
// inline: multiply each received sample by the conjugate of the corresponding
// reference chirp sample to produce the beat signal in beat_buf_[].

void App_R3nderRadarImager::rx_echo_window() {
    hackrf::receiver::set_frequency(center_freq_hz_);
    hackrf::receiver::set_sample_rate(SAMPLE_RATE_HZ);
    hackrf::receiver::set_lna_gain(rx_lna_db_);
    hackrf::receiver::set_vga_gain(rx_vga_db_);

    // Raw RX buffer: int8 interleaved I,Q (HackRF native)
    int8_t raw[(FFT_N + BLANKING_SAMPS) * 2];
    hackrf::receiver::receive(raw, FFT_N + BLANKING_SAMPS);
    hackrf::receiver::stop();

    // Skip blanking guard, convert and de-ramp simultaneously
    for (uint32_t n = 0; n < FFT_N; ++n) {
        uint32_t src = (n + BLANKING_SAMPS) * 2;
        float rx_i = static_cast<float>(raw[src]);
        float rx_q = static_cast<float>(raw[src + 1]);

        // Reference chirp at sample n (conjugate for de-ramp / stretch processing)
        uint32_t ref = (n % CHIRP_SAMPLES) * 2;
        float ref_i =  static_cast<float>(chirp_lut_[ref]);
        float ref_q = -static_cast<float>(chirp_lut_[ref + 1]); // conjugate

        // Beat signal = rx * conj(ref)
        beat_buf_[n] = cx32(rx_i * ref_i - rx_q * ref_q,
                            rx_i * ref_q + rx_q * ref_i);
    }
}

// ── compute_range_profile ─────────────────────────────────────────────────────
// Apply Hanning window to beat signal, FFT, compute magnitude of positive-freq
// bins, normalise to 0–255 for display.

void App_R3nderRadarImager::compute_range_profile() {
    // Hanning window
    for (uint32_t n = 0; n < FFT_N; ++n)
        beat_buf_[n] *= hann_[n];

    // In-place radix-2 FFT
    fft_inplace(beat_buf_, FFT_N);

    // Magnitude of positive-frequency bins → range_mag_[] (float stage)
    float mag_f[RANGE_BINS];
    float peak = 1.0f; // avoid div-by-zero
    for (uint32_t b = 0; b < RANGE_BINS; ++b) {
        float re = beat_buf_[b].real();
        float im = beat_buf_[b].imag();
        mag_f[b] = sqrtf(re * re + im * im);
        if (mag_f[b] > peak) peak = mag_f[b];
    }

    // Normalise to 0–255
    float scale = 255.0f / peak;
    for (uint32_t b = 0; b < RANGE_BINS; ++b)
        range_mag_[b] = static_cast<uint8_t>(mag_f[b] * scale);
}

// ── apply_cfar ────────────────────────────────────────────────────────────────
// Cell-Averaging CFAR: estimate local noise floor from 8 guard + 8 training
// cells on each side; zero any bin below threshold multiplier.
// Simple single-pass CA-CFAR; keeps implementation cheap for LPC4320 M4.

void App_R3nderRadarImager::apply_cfar() {
    static constexpr uint32_t GUARD     = 2;
    static constexpr uint32_t TRAIN     = 8;
    static constexpr float    THRESHOLD = 3.5f; // power ratio above noise floor
    static constexpr uint32_t WINDOW    = GUARD + TRAIN;

    uint8_t out[RANGE_BINS];
    for (uint32_t b = 0; b < RANGE_BINS; ++b) {
        float noise_sum = 0.0f;
        uint32_t cnt    = 0;
        for (uint32_t k = 1; k <= WINDOW; ++k) {
            if (k > GUARD) {
                if (b >= k)             { noise_sum += range_mag_[b - k]; ++cnt; }
                if (b + k < RANGE_BINS) { noise_sum += range_mag_[b + k]; ++cnt; }
            }
        }
        float noise_avg = (cnt > 0) ? (noise_sum / cnt) : 1.0f;
        out[b] = (range_mag_[b] > THRESHOLD * noise_avg) ? range_mag_[b] : 0;
    }
    memcpy(range_mag_, out, RANGE_BINS);
}

// ── render_waterfall_row ──────────────────────────────────────────────────────
// Map range_mag_[RANGE_BINS] → DISPLAY_W pixels (downsample 256→240),
// write into circular waterfall buffer, then paint all rows to the
// H2+ display starting at y=STATUS_H.

void App_R3nderRadarImager::render_waterfall_row() {
    // Downsample 256 range bins → 240 display columns
    uint8_t row[DISPLAY_W];
    for (uint32_t x = 0; x < DISPLAY_W; ++x) {
        uint32_t bin = (x * RANGE_BINS) / DISPLAY_W;
        row[x] = range_mag_[bin];
    }

    // Write into circular buffer at tail position
    uint32_t tail = (wf_head_ + WATERFALL_H - 1) % WATERFALL_H;
    memcpy(waterfall_[tail], row, DISPLAY_W);
    wf_head_ = tail; // head is now the newest row (displayed at bottom)

    // Paint waterfall: head row → top of waterfall area, scrolling down
    for (uint32_t r = 0; r < WATERFALL_H; ++r) {
        uint32_t src_row = (wf_head_ + r) % WATERFALL_H;
        uint32_t y       = STATUS_H + r;
        ui.draw_row(y, waterfall_[src_row], DISPLAY_W, ui.colormap_thermal);
    }
}

// ── render_profile_bar ────────────────────────────────────────────────────────
// Draw current range_mag_[] as a filled amplitude bar chart in the
// PROFILE_H pixel strip below the waterfall on the H2+ display.

void App_R3nderRadarImager::render_profile_bar() {
    static constexpr uint32_t Y0 = STATUS_H + WATERFALL_H; // row 272

    ui.fill_rect(0, Y0, DISPLAY_W, PROFILE_H, ui.color_black);

    for (uint32_t x = 0; x < DISPLAY_W; ++x) {
        uint32_t bin    = (x * RANGE_BINS) / DISPLAY_W;
        uint32_t height = (range_mag_[bin] * PROFILE_H) / 255;
        if (height > 0)
            ui.draw_vline(x, Y0 + PROFILE_H - height, height, ui.color_green);
    }
}

// ── render_status ─────────────────────────────────────────────────────────────

void App_R3nderRadarImager::render_status() {
    char buf[64];
    uint32_t freq_mhz = static_cast<uint32_t>(center_freq_hz_ / 1000000);
    snprintf(buf, sizeof(buf), "%uMHz LNA%u VGA%u #%u %s",
             freq_mhz, rx_lna_db_, rx_vga_db_, sweep_count_,
             sweeping_ ? "RUN" : "STOP");
    ui.fill_rect(0, 0, DISPLAY_W, STATUS_H, ui.color_black);
    ui.draw_text(0, 0, buf, ui.color_white, ui.color_black);
}

// ── render_footer ─────────────────────────────────────────────────────────────

void App_R3nderRadarImager::render_footer() {
    char buf[48];
    uint32_t rres_cm  = static_cast<uint32_t>(RANGE_RES_M * 100.0f);
    uint32_t max_r_m  = static_cast<uint32_t>(RANGE_RES_M * RANGE_BINS);
    snprintf(buf, sizeof(buf), "Res:%u.%02um  Max:%um",
             rres_cm / 100, rres_cm % 100, max_r_m);
    uint32_t y = STATUS_H + WATERFALL_H + PROFILE_H; // row 304
    ui.fill_rect(0, y, DISPLAY_W, FOOTER_H, ui.color_black);
    ui.draw_text(0, y, buf, ui.color_cyan, ui.color_black);
}

// ── input handlers ────────────────────────────────────────────────────────────

void App_R3nderRadarImager::handle_encoder(int delta) {
    int64_t new_freq = static_cast<int64_t>(center_freq_hz_) +
                       static_cast<int64_t>(delta) *
                       static_cast<int64_t>(RADAR_FREQ_STEP_HZ);
    if (new_freq < static_cast<int64_t>(RADAR_FREQ_MIN_HZ))
        new_freq = static_cast<int64_t>(RADAR_FREQ_MIN_HZ);
    if (new_freq > static_cast<int64_t>(RADAR_FREQ_MAX_HZ))
        new_freq = static_cast<int64_t>(RADAR_FREQ_MAX_HZ);
    center_freq_hz_ = static_cast<uint64_t>(new_freq);
    // Rebuild chirp LUT if BW changes relative to new center — no-op here
    // since BW is fixed at 20 MHz regardless of center freq.
    build_chirp_lut();
    render_status();
}

void App_R3nderRadarImager::handle_nav_up() {
    if (rx_lna_db_ + 8 <= 40) rx_lna_db_ += 8; // 8 dB steps (HackRF LNA)
    render_status();
}

void App_R3nderRadarImager::handle_nav_dn() {
    if (rx_lna_db_ >= 8) rx_lna_db_ -= 8;
    render_status();
}

void App_R3nderRadarImager::handle_nav_left() {
    if (rx_vga_db_ >= 2) rx_vga_db_ -= 2; // 2 dB steps (HackRF VGA)
    render_status();
}

void App_R3nderRadarImager::handle_nav_right() {
    if (rx_vga_db_ + 2 <= 62) rx_vga_db_ += 2;
    render_status();
}

void App_R3nderRadarImager::handle_select() {
    sweeping_ = !sweeping_;
    if (!sweeping_) {
        hackrf::transmitter::stop();
        hackrf::receiver::stop();
    }
    render_status();
}
