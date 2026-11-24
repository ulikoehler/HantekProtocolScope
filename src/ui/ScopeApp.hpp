#pragma once

#include "../can/LiveDecoder.hpp"
#include "../can/VirtualTrigger.hpp"
#include "../device/HantekDevice.hpp"
#include "../device/IsoStream.hpp"
#include "../file/FileStreamer.hpp"
#include "../log/Loggers.hpp"
#include "../util/SpscQueue.hpp"
#include "DisplayBuffer.hpp"
#include "MessageListWidget.hpp"

#include <volcano/backend/Backend.hpp>
#include <volcano/plot/Plot.hpp>
#include <volcano/plot/plots/LinePlot.hpp>
#include <volcano/render/Renderer.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ucan {

struct AppConfig {
    std::string firmwarePath;
    int rateMHz = 10;
    std::array<int, 2> probes{1, 1};
    std::array<int, 2> gains{1, 1};
    uint32_t bitrate = 1'000'000;
    float thresholdV = 1.1f;
    double holdoffUs = 1000.0;
    can::VirtualTrigger::Mode triggerMode = can::VirtualTrigger::Mode::Error;
    double windowMs = 20.0;
    size_t displayCapacity = 0; // 0 → derived from rateHz in run()
    uint32_t width = 1600;
    uint32_t height = 950;
    bool stats = false;                 // periodic stderr statistics
    // --- capture / logging ---
    bool noGui = false;                  // headless: no VolcanoPlot window
    std::string logCanPath;              // JSONL frames+triggers
    std::string logRawPath;              // raw ADC .bin for `view`
    std::string viewFile;                // non-empty → replay, not capture
};

/// Live CAN scope: Hantek isochronous stream -> LiveDecoder +
/// VirtualTrigger -> VolcanoPlot scope + message table.
class ScopeApp {
public:
    explicit ScopeApp(AppConfig config);
    ~ScopeApp();

    int run();

private:
    void analyzerMain();
    void processSegment(const uint8_t* bytes, size_t sampleCount,
                        int64_t origin);
    void uiFrame();
    void updateScopePlots();
    void updateWidgets();
    int runHeadless();
    void shutdown();

    AppConfig config_;

    // Device / transport
    std::unique_ptr<HantekDevice> device_;
    std::unique_ptr<IsoStream> stream_;
    std::unique_ptr<FileStreamer> fileStream_;
    SpscQueue<std::unique_ptr<IsoBatch>, 128> batchQueue_;
    std::thread analyzerThread_;
    std::atomic<bool> running_{true};
    std::unique_ptr<CanLogger> canLogger_;
    std::unique_ptr<RawLogger> rawLogger_;
    double calVPC_[2] = {1.0, 1.0};  // V per ADC code (EEPROM/file header)
    double calOC_[2] = {0.0, 0.0};   // ADC-code offsets
    double rateHz_ = 0.0;            // samples/s (device or file header)
    size_t windowSamples_ = 200'000;

    // Analysis (analyzer thread only; created in run() once the device
    // rate is known)
    std::unique_ptr<can::LiveDecoder> decoder_;
    std::unique_ptr<can::VirtualTrigger> trigger_;
    // Channel display buffers: 0 = CANH, 1 = CANL, 2 = differential.
    std::array<std::unique_ptr<DisplayBuffer>, 3> chanBuffers_;
    std::mutex displayMutex_;
    std::vector<float> canhScratch_, canlScratch_, diffScratch_;

    // UI event queue (analyzer -> UI)
    struct UiEvent {
        enum class Kind : uint8_t { Frame, Trigger } kind;
        can::FrameEvent frame;
        can::TriggerEvent trigger;
    };
    SpscQueue<UiEvent, 2048> uiQueue_;
    std::deque<can::TriggerEvent> triggerMarks_;
    MessageListWidget* messageList_ = nullptr;

    // Shared UI-adjustable settings (written on UI thread, synced on
    // analyzer thread at segment start).
    std::atomic<float> threshold_{1.1f};
    std::atomic<uint32_t> bitrate_{1'000'000};
    std::atomic<double> holdoffUs_{1000.0};
    std::atomic<int> triggerMode_{0};
    std::atomic<bool> follow_{true};
    // Newest holdoff-accepted trigger candidate (absolute sample) —
    // written by the analyzer thread, read by the UI for centering.
    // Holdoff gates ONLY this view anchor; trigger events are unfiltered.
    std::atomic<int64_t> latestAccepted_{-1};

    // VolcanoPlot
    std::unique_ptr<volcano::backend::IBackend> backend_;
    std::unique_ptr<volcano::render::Renderer> renderer_;
    std::unique_ptr<volcano::plot::Figure> figure_;
    volcano::plot::Axes* axesTop_ = nullptr;  // CANH + CANL traces
    volcano::plot::Axes* axesDiff_ = nullptr; // CANH − CANL + trigger marks
    volcano::plot::LinePlot *canhMinLine_ = nullptr, *canhMaxLine_ = nullptr,
        *canlMinLine_ = nullptr, *canlMaxLine_ = nullptr;
    volcano::plot::LinePlot *minLine_ = nullptr, *maxLine_ = nullptr,
        *thrLine_ = nullptr, *limLoLine_ = nullptr, *limHiLine_ = nullptr,
        *markerLine_ = nullptr, *markerTopLine_ = nullptr,
        *trigLineTop_ = nullptr, *trigLineDiff_ = nullptr;
    volcano::plot::Slider* thrSlider_ = nullptr;
    volcano::plot::Slider* holdoffSlider_ = nullptr;
    volcano::plot::Button* bitrateButton_ = nullptr;
    volcano::plot::Button* modeButton_ = nullptr;
    volcano::plot::Button* followButton_ = nullptr;
    volcano::plot::Button* yautoButton_ = nullptr;
    volcano::plot::Button* clearButton_ = nullptr;

    // UI-side state
    std::atomic<bool> yAuto_{true};
    bool lastAutoValid_ = false;
    double lastAutoX0_ = 0, lastAutoX1_ = 0;
    // X viewports of both axes are kept equal; syncedX tracks the last
    // common value so manual nav on either axes can be mirrored.
    bool syncedXValid_ = false;
    double syncedX0_ = 0, syncedX1_ = 0;
    bool lastAutoYTopValid_ = false, lastAutoYDiffValid_ = false;
    double lastAutoYTop0_ = 0, lastAutoYTop1_ = 0;
    double lastAutoYDiff0_ = 0, lastAutoYDiff1_ = 0;
    std::vector<float> window_, xs_, guideX_;
    struct Decimated {
        std::vector<float> ymin, ymax;
    };
    std::array<Decimated, 3> decCh_; // CANH, CANL, diff
    int64_t bufferOrigin_ = 0, bufferHead_ = 0;
    // Decimation cache: skip re-decimating a static historical view.
    int64_t decVF_ = -1, decVL_ = -1, decOrigin_ = 0, decHead_ = 0;
    uint64_t framesSeen_ = 0, triggersSeen_ = 0;
    int frameCounter_ = 0;
    int bitrateIndex_ = 3;
    std::string cursorText_;
};

} // namespace ucan
