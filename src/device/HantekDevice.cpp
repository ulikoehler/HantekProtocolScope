#include "HantekDevice.hpp"

#include "../util/IntelHex.hpp"

#include <chrono>
#include <libusb-1.0/libusb.h>
#include <stdexcept>
#include <thread>

namespace ucan {

namespace {

// gain index -> (EEPROM table index, ADC codes per volt)
struct GainEntry { int tableIndex; double codesPerVolt; };
GainEntry gainEntry(int gain) {
    switch (gain) {
        case 1:  return {14, 24.75};
        case 2:  return {8, 49.50};
        case 5:  return {6, 126.25};
        case 10: return {0, 250.0};
        default: throw std::runtime_error("Unbekannter Hantek-Verstärkungswert");
    }
}

} // namespace

HantekDevice::HantekDevice(std::string firmwarePath, int rateMHz,
                           std::array<int, 2> probes, std::array<int, 2> gains)
    : firmwarePath_(std::move(firmwarePath)), rateMHz_(rateMHz),
      probes_(probes), gains_(gains) {
    if (rateMHz_ != 10 && rateMHz_ != 12 && rateMHz_ != 15)
        throw std::runtime_error("Hantek-Abtastrate muss 10, 12 oder 15 MS/s sein");
    for (int channel = 0; channel < 2; ++channel) {
        gainEntry(gains_[channel]);
        if (probes_[channel] != 1 && probes_[channel] != 10)
            throw std::runtime_error("Tastkopfaktor muss 1 oder 10 sein");
    }
}

HantekDevice::~HantekDevice() { close(); }

void HantekDevice::check(int status, const char* what) const {
    if (status < 0)
        throw std::runtime_error(std::string(what) + ": " +
                                 libusb_strerror(static_cast<libusb_error>(status)));
}

void HantekDevice::close() {
    if (handle_) {
        releaseInterface();
        libusb_close(handle_);
        handle_ = nullptr;
    }
    if (context_) {
        libusb_exit(context_);
        context_ = nullptr;
    }
}

bool HantekDevice::connect(uint16_t vendor, std::string* error) {
    libusb_device_handle* handle =
        libusb_open_device_with_vid_pid(context_, vendor, kProduct);
    if (!handle) return false;
    const int status = libusb_claim_interface(handle, 0);
    if (status != LIBUSB_SUCCESS) {
        if (error)
            *error = libusb_strerror(static_cast<libusb_error>(status));
        libusb_close(handle);
        return false;
    }
    handle_ = handle;
    return true;
}

void HantekDevice::open() {
    check(libusb_init(&context_), "USB initialisieren");
    try {
        if (connect(kRunningVid)) return;
        if (!connect(kLoaderVid))
            throw std::runtime_error(
                "Hantek 6022BL nicht über libusb erreichbar. Oszi-Modus, "
                "Treiberbindung und andere Oszi-Programme prüfen.");
        auto blocks = loadFirmware(firmwarePath_);
        const uint8_t start = 1;
        controlWrite(0xA0, &start, 1, 0xE600);
        for (const auto& [address, data] : blocks)
            controlWrite(0xA0, data.data(), data.size(), address);
        const uint8_t stop = 0;
        controlWrite(0xA0, &stop, 1, 0xE600, true);
        libusb_release_interface(handle_, 0);
        libusb_close(handle_);
        handle_ = nullptr;

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(15);
        std::string runningConnectError;
        while (std::chrono::steady_clock::now() < deadline) {
            if (connect(kRunningVid, &runningConnectError)) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::string message =
            "Hantek nach RAM-Start unter 04b5:602a nicht erreichbar.";
        if (!runningConnectError.empty())
            message += " USB-Interface: " + runningConnectError;
        throw std::runtime_error(message);
    } catch (...) {
        close();
        throw;
    }
}

void HantekDevice::configure() {
    const uint8_t stop = 0, mode = 2;
    const uint8_t rate = static_cast<uint8_t>(rateMHz_);
    controlWrite(0xE3, &stop, 1);
    controlWrite(0xE4, &mode, 1);
    controlWrite(0xE2, &rate, 1);
    for (int channel = 0; channel < 2; ++channel) {
        const uint8_t gain = static_cast<uint8_t>(gains_[channel]);
        controlWrite(channel == 0 ? 0xE0 : 0xE1, &gain, 1);
    }
    auto eeprom = controlRead(0xA2, 8, 80);
    eeprom.resize(80, 0xFF);
    settings_.rateHz = rateMHz_ * 1e6;
    settings_.probeFactors = probes_;
    settings_.gains = gains_;
    for (int channel = 0; channel < 2; ++channel) {
        const auto entry = gainEntry(gains_[channel]);
        const size_t index = entry.tableIndex + channel;
        const uint8_t coarse = eeprom[index];
        const uint8_t fine = eeprom[index + 48];
        const uint8_t correction = eeprom[index + 32];
        double offset = (coarse >= 80 && coarse <= 175)
                            ? static_cast<double>(coarse) : 128.0;
        if (offset == coarse && fine > 0 && fine < 255)
            offset += (static_cast<double>(fine) - 128.0) / 250.0;
        double factor = (correction > 0 && correction < 255)
                            ? 1.0 + (static_cast<double>(correction) - 128.0) / 500.0
                            : 1.0;
        settings_.offsetCodes[channel] = offset;
        settings_.voltsPerCode[channel] =
            probes_[channel] * factor / entry.codesPerVolt;
    }
}

void HantekDevice::claimInterface() {
    if (!handle_) throw std::runtime_error("Hantek-Gerät nicht geöffnet");
    check(libusb_claim_interface(handle_, 0), "USB-Interface öffnen");
}

void HantekDevice::releaseInterface() {
    if (handle_) libusb_release_interface(handle_, 0);
}

void HantekDevice::setAltSetting(int alternate) {
    check(libusb_set_interface_alt_setting(handle_, 0, alternate),
          "Hantek-USB-Modus auswählen");
}

void HantekDevice::controlWrite(uint8_t request, const void* data,
                                size_t length, uint16_t address,
                                bool disconnectOk) {
    int status = libusb_control_transfer(
        handle_, 0x40, request, address, 0,
        const_cast<uint8_t*>(static_cast<const uint8_t*>(data)),
        static_cast<uint16_t>(length), 2000);
    if (disconnectOk && status == LIBUSB_ERROR_NO_DEVICE) return;
    check(status, "Hantek-Steuerbefehl");
    if (static_cast<size_t>(status) != length)
        throw std::runtime_error("Unvollständiger Hantek-Steuertransfer");
}

std::vector<uint8_t> HantekDevice::controlRead(uint8_t request,
                                               uint16_t value,
                                               size_t length) {
    std::vector<uint8_t> buffer(length);
    int status = libusb_control_transfer(handle_, 0xC0, request, value, 0,
                                         buffer.data(),
                                         static_cast<uint16_t>(length), 2000);
    check(status, "Hantek-Steuerlesung");
    buffer.resize(static_cast<size_t>(status));
    return buffer;
}

double HantekDevice::voltage(uint8_t adc, int channel) const noexcept {
    return (static_cast<double>(adc) - settings_.offsetCodes[channel]) *
           settings_.voltsPerCode[channel];
}

} // namespace ucan
