// vl53l1x.h – schlanker VL53L1X-Treiber fuer den Raspberry Pi Pico (Pico SDK)
//
// Registeradressen, Standardkonfiguration und Timing-Werte stammen aus dem
// "VL53L1X Ultra Lite Driver" von STMicroelectronics (BSD-3-Clause, siehe
// vl53l1x.cpp). Die I2C-Anbindung und die Klassenstruktur sind neu geschrieben.
//
// Typische Verwendung:
//   VL53L1X sensor(i2c0);
//   sensor.init();
//   sensor.setDistanceMode(VL53L1X::DistanceMode::Short);
//   sensor.setTimingBudgetMs(20);
//   sensor.setInterMeasurementMs(20);
//   sensor.startRanging();
//   ... dataReady() abfragen, dann readMeasurement()

#pragma once

#include <cstdint>
#include <cstddef>
#include "hardware/i2c.h"

class VL53L1X {
public:
    // Short: bis ca. 1,3 m, robust gegen Umgebungslicht
    // Long:  bis ca. 4 m, empfindlicher gegen Licht
    enum class DistanceMode : uint8_t { Short = 1, Long = 2 };

    struct Measurement {
        uint16_t distance_mm;   // gemessene Entfernung in Millimetern
        uint8_t  range_status;  // 0 = gueltig, alles andere: siehe statusText()
    };

    explicit VL53L1X(i2c_inst_t* i2c, uint8_t address = 0x29);

    // Wartet auf den Sensor-Boot, prueft die ID und laedt die Standardkonfiguration.
    bool init();

    bool setDistanceMode(DistanceMode mode);
    // Erlaubt: 15 (nur Short), 20, 33, 50, 100, 200, 500 ms
    bool setTimingBudgetMs(uint16_t budget_ms);
    // Abstand zwischen zwei Messungen; muss >= Timing-Budget sein
    bool setInterMeasurementMs(uint16_t period_ms);

    bool startRanging();
    bool stopRanging();

    // ready wird true, sobald eine neue Messung abholbereit ist
    bool dataReady(bool& ready);
    // Liest Entfernung und Status und gibt den Sensor fuer die naechste Messung frei
    bool readMeasurement(Measurement& m);

    bool readSensorId(uint16_t& id);

    static const char* statusText(uint8_t range_status);

private:
    // Low-Level-Zugriff: der VL53L1X nutzt 16-Bit-Registeradressen, Daten big-endian
    bool writeReg(uint16_t reg, const uint8_t* data, size_t len);
    bool readReg(uint16_t reg, uint8_t* data, size_t len);
    bool writeByte(uint16_t reg, uint8_t value);
    bool writeWord(uint16_t reg, uint16_t value);
    bool writeDWord(uint16_t reg, uint32_t value);
    bool readByte(uint16_t reg, uint8_t& value);
    bool readWord(uint16_t reg, uint16_t& value);

    bool clearInterrupt();

    i2c_inst_t*  i2c_;
    uint8_t      address_;
    DistanceMode mode_ = DistanceMode::Long;  // Zustand nach der Standardkonfiguration
    uint16_t     timing_budget_ms_ = 0;       // 0 = noch nicht gesetzt
    uint8_t      int_polarity_ = 1;           // Pegel, der "Daten bereit" bedeutet
};
