// vl53l1x.cpp – schlanker VL53L1X-Treiber fuer den Raspberry Pi Pico (Pico SDK)
//
// Basiert auf dem VL53L1X Ultra Lite Driver (ULD) von STMicroelectronics:
//
//   COPYRIGHT(c) 2018 STMicroelectronics
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//   1. Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//   2. Redistributions in binary form must reproduce the above copyright notice,
//      this list of conditions and the following disclaimer in the documentation
//      and/or other materials provided with the distribution.
//   3. Neither the name of STMicroelectronics nor the names of its contributors
//      may be used to endorse or promote products derived from this software
//      without specific prior written permission.
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//   AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. (Vollstaendiger Text:
//   BSD-3-Clause.)

#include "vl53l1x.h"
#include "pico/stdlib.h"

namespace {

// --- Registeradressen (aus dem ST ULD) --------------------------------------
constexpr uint16_t VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
constexpr uint16_t VHV_CONFIG__INIT                      = 0x000B;
constexpr uint16_t GPIO_HV_MUX__CTRL                     = 0x0030;
constexpr uint16_t GPIO__TIO_HV_STATUS                   = 0x0031;
constexpr uint16_t PHASECAL_CONFIG__TIMEOUT_MACROP       = 0x004B;
constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_A_HI     = 0x005E;
constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_A          = 0x0060;
constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_B_HI     = 0x0061;
constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_B          = 0x0063;
constexpr uint16_t RANGE_CONFIG__VALID_PHASE_HIGH        = 0x0069;
constexpr uint16_t SYSTEM__INTERMEASUREMENT_PERIOD       = 0x006C;
constexpr uint16_t SD_CONFIG__WOI_SD0                    = 0x0078;
constexpr uint16_t SD_CONFIG__INITIAL_PHASE_SD0          = 0x007A;
constexpr uint16_t SYSTEM__INTERRUPT_CLEAR               = 0x0086;
constexpr uint16_t SYSTEM__MODE_START                    = 0x0087;
constexpr uint16_t RESULT__RANGE_STATUS                  = 0x0089;
constexpr uint16_t RESULT__FINAL_RANGE_MM_SD0            = 0x0096;
constexpr uint16_t RESULT__OSC_CALIBRATE_VAL             = 0x00DE;
constexpr uint16_t FIRMWARE__SYSTEM_STATUS               = 0x00E5;
constexpr uint16_t IDENTIFICATION__MODEL_ID              = 0x010F;

constexpr uint16_t EXPECTED_MODEL_ID = 0xEACC;
constexpr uint32_t I2C_TIMEOUT_US    = 10000;

// --- Standardkonfiguration fuer die Register 0x2D bis 0x87 (aus dem ST ULD) --
// Diese Werte stellen u. a. Interrupt-Verhalten, Signal- und Sigma-Grenzen,
// das Messfenster (ROI) und interne Kalibrierparameter ein.
constexpr uint16_t CONFIG_START = 0x2D;
constexpr uint8_t DEFAULT_CONFIGURATION[] = {
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, // 0x2D - 0x34
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, // 0x35 - 0x3C
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, // 0x3D - 0x44
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, // 0x45 - 0x4C
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, // 0x4D - 0x54
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, // 0x55 - 0x5C
    0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, // 0x5D - 0x64
    0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00, // 0x65 - 0x6C
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x6D - 0x74
    0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00, // 0x75 - 0x7C
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, // 0x7D - 0x84
    0x01, 0x00, 0x00                                // 0x85 - 0x87
};
static_assert(sizeof(DEFAULT_CONFIGURATION) == 0x87 - 0x2D + 1,
              "Konfiguration muss genau die Register 0x2D..0x87 abdecken");

// --- Timing-Budget-Tabellen (aus dem ST ULD) --------------------------------
struct TimingEntry { uint16_t budget_ms; uint16_t macrop_a; uint16_t macrop_b; };

constexpr TimingEntry TIMING_SHORT[] = {
    { 15, 0x001D, 0x0027}, { 20, 0x0051, 0x006E}, { 33, 0x00D6, 0x006E},
    { 50, 0x01AE, 0x01E8}, {100, 0x02E1, 0x0388}, {200, 0x03E1, 0x0496},
    {500, 0x0591, 0x05C1},
};
constexpr TimingEntry TIMING_LONG[] = {
    { 20, 0x001E, 0x0022}, { 33, 0x0060, 0x006E}, { 50, 0x00AD, 0x00C6},
    {100, 0x01CC, 0x01EA}, {200, 0x02D9, 0x02F8}, {500, 0x048F, 0x04A4},
};

} // namespace

VL53L1X::VL53L1X(i2c_inst_t* i2c, uint8_t address)
    : i2c_(i2c), address_(address) {}

// ============================================================================
// Low-Level-I2C
// ============================================================================

bool VL53L1X::writeReg(uint16_t reg, const uint8_t* data, size_t len) {
    // Ein Schreibzugriff ist: [Reg-High][Reg-Low][Daten...] in einer Transaktion
    uint8_t buf[2 + 4];
    if (len > 4) return false;
    buf[0] = static_cast<uint8_t>(reg >> 8);
    buf[1] = static_cast<uint8_t>(reg & 0xFF);
    for (size_t i = 0; i < len; ++i) buf[2 + i] = data[i];
    int n = i2c_write_timeout_us(i2c_, address_, buf, 2 + len, false, I2C_TIMEOUT_US);
    return n == static_cast<int>(2 + len);
}

bool VL53L1X::readReg(uint16_t reg, uint8_t* data, size_t len) {
    // Erst Registeradresse schreiben (ohne Stop = "repeated start"), dann lesen
    uint8_t addr[2] = { static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF) };
    if (i2c_write_timeout_us(i2c_, address_, addr, 2, true, I2C_TIMEOUT_US) != 2)
        return false;
    int n = i2c_read_timeout_us(i2c_, address_, data, len, false, I2C_TIMEOUT_US);
    return n == static_cast<int>(len);
}

bool VL53L1X::writeByte(uint16_t reg, uint8_t value) {
    return writeReg(reg, &value, 1);
}

bool VL53L1X::writeWord(uint16_t reg, uint16_t value) {
    uint8_t b[2] = { static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value) };
    return writeReg(reg, b, 2);
}

bool VL53L1X::writeDWord(uint16_t reg, uint32_t value) {
    uint8_t b[4] = { static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
                     static_cast<uint8_t>(value >> 8),  static_cast<uint8_t>(value) };
    return writeReg(reg, b, 4);
}

bool VL53L1X::readByte(uint16_t reg, uint8_t& value) {
    return readReg(reg, &value, 1);
}

bool VL53L1X::readWord(uint16_t reg, uint16_t& value) {
    uint8_t b[2];
    if (!readReg(reg, b, 2)) return false;
    value = static_cast<uint16_t>((b[0] << 8) | b[1]);
    return true;
}

// ============================================================================
// Initialisierung
// ============================================================================

bool VL53L1X::readSensorId(uint16_t& id) {
    return readWord(IDENTIFICATION__MODEL_ID, id);
}

bool VL53L1X::init() {
    // 1) Warten, bis die interne Firmware hochgefahren ist (Bit 0 = 1)
    uint8_t boot = 0;
    for (int i = 0; i < 100; ++i) {
        if (readByte(FIRMWARE__SYSTEM_STATUS, boot) && (boot & 0x01)) break;
        sleep_ms(10);
    }
    if (!(boot & 0x01)) return false;

    // 2) Pruefen, ob wirklich ein VL53L1X antwortet
    uint16_t id = 0;
    if (!readSensorId(id) || id != EXPECTED_MODEL_ID) return false;

    // 3) Standardkonfiguration Register fuer Register schreiben (wie im ST ULD)
    for (uint16_t i = 0; i < sizeof(DEFAULT_CONFIGURATION); ++i) {
        if (!writeByte(CONFIG_START + i, DEFAULT_CONFIGURATION[i])) return false;
    }

    // Polaritaet des "Daten bereit"-Signals einmal auslesen und merken
    uint8_t mux = 0;
    if (!readByte(GPIO_HV_MUX__CTRL, mux)) return false;
    int_polarity_ = ((mux & 0x10) >> 4) ? 0 : 1;

    // 4) Eine Probemessung durchfuehren: Dabei kalibriert der Sensor intern
    //    seine Versorgungsspannungsregelung (VHV) auf die aktuelle Temperatur.
    if (!startRanging()) return false;
    bool ready = false;
    for (int i = 0; i < 500 && !ready; ++i) {   // Standardperiode ~103 ms, max. 500 ms warten
        if (!dataReady(ready)) return false;
        if (!ready) sleep_ms(1);
    }
    if (!ready) return false;
    if (!clearInterrupt() || !stopRanging()) return false;

    // 5) VHV-Kalibrierung fuer spaetere Messungen beschleunigen (aus dem ST ULD)
    if (!writeByte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09)) return false;
    if (!writeByte(VHV_CONFIG__INIT, 0x00)) return false;

    mode_ = DistanceMode::Long;   // Zustand der Standardkonfiguration
    timing_budget_ms_ = 0;
    return true;
}

// ============================================================================
// Konfiguration
// ============================================================================

bool VL53L1X::setDistanceMode(DistanceMode mode) {
    bool ok = true;
    if (mode == DistanceMode::Short) {
        ok &= writeByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
        ok &= writeByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
        ok &= writeByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
        ok &= writeByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
        ok &= writeWord(SD_CONFIG__WOI_SD0, 0x0705);
        ok &= writeWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
    } else {
        ok &= writeByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
        ok &= writeByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
        ok &= writeByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
        ok &= writeByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
        ok &= writeWord(SD_CONFIG__WOI_SD0, 0x0F0D);
        ok &= writeWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
    }
    if (!ok) return false;
    mode_ = mode;

    // Die Timing-Register haengen vom Modus ab -> ein schon gesetztes Budget neu anwenden
    if (timing_budget_ms_ != 0) return setTimingBudgetMs(timing_budget_ms_);
    return true;
}

bool VL53L1X::setTimingBudgetMs(uint16_t budget_ms) {
    const TimingEntry* table = (mode_ == DistanceMode::Short) ? TIMING_SHORT : TIMING_LONG;
    size_t count = (mode_ == DistanceMode::Short)
                       ? sizeof(TIMING_SHORT) / sizeof(TimingEntry)
                       : sizeof(TIMING_LONG) / sizeof(TimingEntry);

    for (size_t i = 0; i < count; ++i) {
        if (table[i].budget_ms == budget_ms) {
            if (!writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, table[i].macrop_a)) return false;
            if (!writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, table[i].macrop_b)) return false;
            timing_budget_ms_ = budget_ms;
            return true;
        }
    }
    return false;   // Wert nicht in der Tabelle (z. B. 15 ms im Long-Modus)
}

bool VL53L1X::setInterMeasurementMs(uint16_t period_ms) {
    // Der Sensor zaehlt in Takten seines internen Oszillators. Dessen Frequenz
    // steht im Register OSC_CALIBRATE_VAL; Faktor 1.075 laut ST ULD.
    uint16_t clock_pll = 0;
    if (!readWord(RESULT__OSC_CALIBRATE_VAL, clock_pll)) return false;
    clock_pll &= 0x3FF;
    uint32_t ticks = static_cast<uint32_t>(clock_pll * period_ms * 1.075f);
    return writeDWord(SYSTEM__INTERMEASUREMENT_PERIOD, ticks);
}

// ============================================================================
// Messen
// ============================================================================

bool VL53L1X::startRanging() {
    if (!clearInterrupt()) return false;
    return writeByte(SYSTEM__MODE_START, 0x40);   // 0x40 = kontinuierliche Messung
}

bool VL53L1X::stopRanging() {
    return writeByte(SYSTEM__MODE_START, 0x00);
}

bool VL53L1X::clearInterrupt() {
    return writeByte(SYSTEM__INTERRUPT_CLEAR, 0x01);
}

bool VL53L1X::dataReady(bool& ready) {
    uint8_t status = 0;
    if (!readByte(GPIO__TIO_HV_STATUS, status)) return false;
    ready = ((status & 0x01) == int_polarity_);
    return true;
}

bool VL53L1X::readMeasurement(Measurement& m) {
    // Status (0x89) und Entfernung (0x96/0x97) liegen im selben Ergebnisblock.
    // Ein einziger Lesezugriff ueber 15 Bytes ist schneller als mehrere einzelne.
    uint8_t buf[RESULT__FINAL_RANGE_MM_SD0 + 2 - RESULT__RANGE_STATUS];
    if (!readReg(RESULT__RANGE_STATUS, buf, sizeof(buf))) return false;

    // Interner Statuscode -> ST-Statuscode (Tabelle aus dem ST ULD)
    static constexpr uint8_t STATUS_MAP[24] = {
        255, 255, 255,   5,   2,   4,   1,   7,   3,   0,
        255, 255,   9,  13, 255, 255, 255, 255,  10,   6,
        255, 255,  11,  12
    };
    uint8_t raw = buf[0] & 0x1F;
    m.range_status = (raw < sizeof(STATUS_MAP)) ? STATUS_MAP[raw] : 255;

    size_t off = RESULT__FINAL_RANGE_MM_SD0 - RESULT__RANGE_STATUS;
    m.distance_mm = static_cast<uint16_t>((buf[off] << 8) | buf[off + 1]);

    // Sensor mitteilen, dass die Daten abgeholt wurden -> naechste Messung
    return clearInterrupt();
}

const char* VL53L1X::statusText(uint8_t s) {
    switch (s) {
        case 0:  return "OK";
        case 1:  return "Sigma zu gross (unsicher)";
        case 2:  return "Signal zu schwach";
        case 4:  return "Ausserhalb des Messbereichs";
        case 7:  return "Wrap-around (Ziel zu weit)";
        default: return "anderer Fehler";
    }
}
