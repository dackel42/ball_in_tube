// VL53L1X – erster Verbindungstest fuer Raspberry Pi Pico 2 (RP2350)
//
// Verdrahtung:
//   Sensor VIN -> Pico 3V3(OUT)  (Pin 36)
//   Sensor GND -> Pico GND       (Pin 38)
//   Sensor SDA -> Pico GP4       (Pin 6)
//   Sensor SCL -> Pico GP5       (Pin 7)
//   XSHUT / GPIO1 vorerst offen lassen
//
// Erwartete Ausgabe im seriellen Monitor:
//   Geraet gefunden auf 0x29
//   Boot-Status: 1, Sensor-ID: 0xEACC

#include <stdio.h>
#include <cstdint>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "blink.pio.h"
#include "vl53l1x.h"

static i2c_inst_t* const I2C_PORT = i2c0;
constexpr uint SDA_PIN = 4;
constexpr uint SCL_PIN = 5;

constexpr uint16_t TIMING_BUDGET_MS = 20;   // Messdauer pro Wert
constexpr uint16_t PERIOD_MS        = 20;   // Abstand der Messungen -> ~50 Hz

// PIO Blinking example 
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

int main() {
    stdio_init_all();
    sleep_ms(3000);  // Zeit, um den seriellen Monitor zu oeffnen

    i2c_init(I2C_PORT, 400 * 1000);           // 400 kHz Fast Mode
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);                    // schwache interne Pull-ups;
    gpio_pull_up(SCL_PIN);                    // die meisten Module haben eigene

    stdio_init_all();
    sleep_ms(3000);  // Zeit, um den seriellen Monitor zu oeffnen

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    VL53L1X sensor(I2C_PORT);

    // Initialisierung; bei Fehler jede Sekunde Meldung ausgeben statt still zu haengen
    while (!sensor.init()) {
        printf("Sensor-Initialisierung fehlgeschlagen - Verdrahtung pruefen\n");
        sleep_ms(1000);
    }

    bool ok = sensor.setDistanceMode(VL53L1X::DistanceMode::Short)
           && sensor.setTimingBudgetMs(TIMING_BUDGET_MS)
           && sensor.setInterMeasurementMs(PERIOD_MS)
           && sensor.startRanging();
    if (!ok) {
        while (true) {
            printf("Sensor-Konfiguration fehlgeschlagen\n");
            sleep_ms(1000);
        }
    }

    printf("VL53L1X bereit: Short-Modus, %u ms Timing-Budget\n", TIMING_BUDGET_MS);

    uint32_t count = 0;
    absolute_time_t window_start = get_absolute_time();

    while (true) {
        
        // PIO Blinking example
        // PIO pio = pio0;
        // uint offset = pio_add_program(pio, &blink_program);
        // printf("Loaded program at %d\n", offset);
        // #ifdef PICO_DEFAULT_LED_PIN
        // blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
        // #else
        // blink_pin_forever(pio, 0, offset, 6, 10);
        // #endif

        bool ready = false;
        if (!sensor.dataReady(ready)) {
            printf("I2C-Fehler beim Abfragen\n");
            sleep_ms(100);
            continue;
        }
        if (!ready) {
            sleep_us(500);   // kurz warten, nicht ununterbrochen den Bus belasten
            continue;
        }

        VL53L1X::Measurement m{};
        if (!sensor.readMeasurement(m)) {
            printf("I2C-Fehler beim Lesen\n");
            continue;
        }

        uint32_t t_ms = to_ms_since_boot(get_absolute_time());
        printf("%8lu ms  %5u mm  Status %3u (%s)\n",
               static_cast<unsigned long>(t_ms), m.distance_mm, m.range_status,
               VL53L1X::statusText(m.range_status));

        // Einmal pro Sekunde die tatsaechliche Messrate ausgeben
        ++count;
        int64_t dt_us = absolute_time_diff_us(window_start, get_absolute_time());
        if (dt_us >= 1000000) {
            printf("--- Messrate: %.1f Hz ---\n", count * 1e6 / static_cast<double>(dt_us));
            count = 0;
            window_start = get_absolute_time();
        }
    }
}
