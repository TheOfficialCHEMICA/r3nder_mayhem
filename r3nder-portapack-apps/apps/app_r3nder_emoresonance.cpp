#include <hackrf.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <unistd.h>

static hackrf_device* device = nullptr;
static const uint32_t SAMPLE_RATE = 2000000; // 2 MHz
static uint32_t tone_phase = 0;
static uint32_t frequency_hz = 435000000; // default frequency
static bool keep_running = true;

static int tx_callback(hackrf_transfer* transfer) {
    int8_t* buf = reinterpret_cast<int8_t*>(transfer->buffer);
    const float step = 2.0f * M_PI * frequency_hz / SAMPLE_RATE;
    for (unsigned int i = 0; i < transfer->valid_length; i += 2) {
        float s = sinf(step * tone_phase);
        int8_t sample = static_cast<int8_t>(s * 127);
        buf[i] = sample;
        buf[i+1] = sample;
        tone_phase++;
    }
    return 0;
}

static void handle_sigint(int) {
    keep_running = false;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <mood 0-100>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int mood = std::atoi(argv[1]);
    if (mood < 0 || mood > 100) {
        fprintf(stderr, "Mood must be in range 0-100\n");
        return EXIT_FAILURE;
    }

    frequency_hz = 430000000 + (mood * 1000000); // map mood to 430-530 MHz

    if (hackrf_init() != HACKRF_SUCCESS) {
        fprintf(stderr, "hackrf_init failed\n");
        return EXIT_FAILURE;
    }

    if (hackrf_open(&device) != HACKRF_SUCCESS) {
        fprintf(stderr, "hackrf_open failed\n");
        hackrf_exit();
        return EXIT_FAILURE;
    }

    hackrf_set_txvga_gain(device, 20);
    hackrf_set_sample_rate(device, SAMPLE_RATE);
    hackrf_set_baseband_filter_bandwidth(device, SAMPLE_RATE);
    hackrf_set_freq(device, frequency_hz);

    signal(SIGINT, handle_sigint);

    if (hackrf_start_tx(device, tx_callback, nullptr) != HACKRF_SUCCESS) {
        fprintf(stderr, "hackrf_start_tx failed\n");
        hackrf_close(device);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    printf("Transmitting mood frequency at %u Hz. Press Ctrl+C to stop...\n", frequency_hz);
    while (keep_running) {
        sleep(1);
    }

    hackrf_stop_tx(device);
    hackrf_close(device);
    hackrf_exit();
    return EXIT_SUCCESS;
}
