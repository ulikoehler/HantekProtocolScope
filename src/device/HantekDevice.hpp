#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct libusb_context;
struct libusb_device_handle;

namespace ucan {

/// Hantek DSO-6022BL over libusb-1.0: loader-mode firmware upload,
/// ADC configuration and EEPROM calibration. Port of hantek.py.
class HantekDevice {
public:
    static constexpr uint16_t kLoaderVid = 0x04B4;
    static constexpr uint16_t kRunningVid = 0x04B5;
    static constexpr uint16_t kProduct = 0x602A;

    struct Settings {
        double rateHz = 0;
        std::array<double, 2> offsetCodes{};
        std::array<double, 2> voltsPerCode{};
        std::array<int, 2> probeFactors{1, 1};
        std::array<int, 2> gains{1, 1};
    };

    /// rateMHz: 10, 12 or 15. probes/gains: per-channel {1|10} / {1|2|5|10}.
    HantekDevice(std::string firmwarePath, int rateMHz,
                 std::array<int, 2> probes, std::array<int, 2> gains);
    ~HantekDevice();

    HantekDevice(const HantekDevice&) = delete;
    HantekDevice& operator=(const HantekDevice&) = delete;

    /// Open the device, uploading the RAM firmware when in loader mode.
    void open();
    /// Program channels and read the EEPROM calibration.
    void configure();
    /// Release interface and close the device handle.
    void close();

    [[nodiscard]] libusb_device_handle* handle() const noexcept { return handle_; }
    [[nodiscard]] libusb_context* context() const noexcept { return context_; }
    [[nodiscard]] const Settings& settings() const noexcept { return settings_; }

    void claimInterface();
    void releaseInterface();
    void setAltSetting(int alternate);
    /// Vendor control write (bmRequestType 0x40). disconnectOk swallows
    /// LIBUSB_ERROR_NO_DEVICE like the Python loader path.
    void controlWrite(uint8_t request, const void* data, size_t length,
                      uint16_t address = 0, bool disconnectOk = false);
    /// Vendor control read (bmRequestType 0xC0).
    std::vector<uint8_t> controlRead(uint8_t request, uint16_t value,
                                     size_t length);

    /// Convert one raw ADC code to volts for channel `channel` (0|1).
    [[nodiscard]] double voltage(uint8_t adc, int channel) const noexcept;

private:
    bool connect(uint16_t vendor);
    void check(int status, const char* what) const;

    libusb_context* context_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    std::string firmwarePath_;
    int rateMHz_;
    std::array<int, 2> probes_;
    std::array<int, 2> gains_;
    Settings settings_;
};

} // namespace ucan
