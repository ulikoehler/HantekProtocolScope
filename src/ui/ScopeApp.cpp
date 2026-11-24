#include "ScopeApp.hpp"

#include <volcano/plot/Interaction.hpp>
#include <volcano/plot/Style.hpp>
#include <volcano/plot/Widgets.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <format>
#include <iostream>
#include <limits>
#include <thread>

namespace ucan {

namespace vp = volcano::plot;

namespace {
std::atomic<bool> g_sigStop{false};
void handleSignal(int) noexcept { g_sigStop.store(true); }
void installSignalHandlers() {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
}
} // namespace

namespace {
constexpr int kListHeight = 170;
constexpr int kControlHeight = 66;
constexpr std::array<uint32_t, 4> kBitrates = {125'000, 250'000, 500'000,
                                               1'000'000};
constexpr std::array<const char*, 4> kBitrateLabels = {
    "Bitrate: 125k", "Bitrate: 250k", "Bitrate: 500k", "Bitrate: 1M"};

/// "1 µs" / "250 µs" / "10 ms" / "1 s" style formatting for the
/// holdoff slider label.
std::string formatUs(double us) {
    if (us >= 1e6) return std::format("{:.4g} s", us / 1e6);
    if (us >= 1e3) return std::format("{:.4g} ms", us / 1e3);
    return std::format("{:.4g} µs", us);
}

/// Short overlay label for a decoded frame — raw "123#01 02 …" or the
/// Klipper CAN protocol interpretation (docs/CANBUS_protocol.md).
std::string formatPacketLabel(const can::FrameEvent& fe,
                              PacketFormat fmt) {
    const auto& f = fe.frame;
    const auto hex = [&f](int i0, int i1) {
        std::string s;
        for (int i = i0; i < i1 && i < f.dlc; ++i) {
            if (i != i0) s += ' ';
            s += std::format("{:02X}", static_cast<unsigned>(f.data[i]));
        }
        return s;
    };
    if (fmt == PacketFormat::Klipper) {
        if (f.identifier == 0x3f0) { // host → mcu admin (broadcast)
            if (f.dlc >= 1 && f.data[0] == 0x00)
                return "adm> QUERY_UNASSIGNED";
            if (f.dlc >= 8 && f.data[0] == 0x01)
                return std::format("adm> SET_NODEID n{} {}", f.data[7],
                                   hex(1, 7));
            return "adm> " + hex(0, f.dlc);
        }
        if (f.identifier == 0x3f1) { // mcu → host admin
            if (f.dlc >= 8 && f.data[0] == 0x20)
                return "adm< NEED_NODEID " + hex(1, 7);
            return "adm< " + hex(0, f.dlc);
        }
        if (f.identifier >= 256) { // node data channel
            const int node = static_cast<int>(f.identifier - 256) / 2;
            const bool resp = (f.identifier - 256) & 1;
            return std::format("n{} {} {}", node, resp ? "resp" : "cmd",
                               fe.decoded.empty() ? hex(0, f.dlc)
                                                  : fe.decoded);
        }
        return std::format("0x{:03X} {}", f.identifier, hex(0, f.dlc));
    }
    return std::format("{:03X}#{}", f.identifier, hex(0, f.dlc));
}
} // namespace

ScopeApp::ScopeApp(AppConfig config) : config_(std::move(config)) {
    bitrate_.store(config_.bitrate);
    threshold_.store(config_.thresholdV);
    holdoffUs_.store(config_.holdoffUs);
    triggerMode_.store(static_cast<int>(config_.triggerMode));
}

ScopeApp::~ScopeApp() {
    shutdown();
    // VolcanoPlot's ~ScreenBackend crashes on Wayland: glfwTerminate()
    // runs before the Vulkan device is destroyed, so RADV then issues
    // Wayland calls on a dead connection. Leak the GPU objects instead —
    // the kernel reclaims them when the process exits.
    figure_.release();
    renderer_.release();
    backend_.release();
}

void ScopeApp::shutdown() {
    running_.store(false, std::memory_order_relaxed);
    if (analyzerThread_.joinable()) analyzerThread_.join();
    if (stream_) stream_->stop();
    if (fileStream_) fileStream_->stop();
    if (device_) {
        try {
            device_->releaseInterface();
            device_->close();
        } catch (...) {
        }
    }
}

int ScopeApp::run() {
    installSignalHandlers();
    rateHz_ = 0.0;

    if (!config_.viewFile.empty()) {
        // Replay: mmap the raw file, calibration + rate from its header.
        fileStream_ = std::make_unique<FileStreamer>(
            config_.viewFile, batchQueue_);
        rateHz_ = fileStream_->rateHz();
        const auto& h = fileStream_->header();
        for (int i = 0; i < 2; ++i) {
            calVPC_[i] = h.voltsPerCode[i];
            calOC_[i] = h.offsetCodes[i];
        }
    } else {
        device_ = std::make_unique<HantekDevice>(
            config_.firmwarePath, config_.rateMHz, config_.probes,
            config_.gains);
        device_->open();
        device_->configure();
        const auto& st = device_->settings();
        rateHz_ = st.rateHz;
        for (int i = 0; i < 2; ++i) {
            calVPC_[i] = st.voltsPerCode[i];
            calOC_[i] = st.offsetCodes[i];
        }
        stream_ = std::make_unique<IsoStream>(*device_, batchQueue_);
    }
    // Per-channel voltage: v_i = voltsPerCode_i * (adc_i - offset_i);
    // the differential trace is v0 - v1 (see processSegment).

    // Window + buffer sizing need the real sample rate (device or file).
    windowSamples_ = static_cast<size_t>(config_.windowMs * 1e-3 * rateHz_);
    if (config_.displayCapacity == 0) {
        // 2 s of stream history — deep enough for retrospective
        // triggering (accepted triggers are detected after their
        // window is already buffered) and for holdoff freezes.
        const size_t streamSamples = static_cast<size_t>(rateHz_);
        config_.displayCapacity = std::max<size_t>(windowSamples_ * 12,
                                                   streamSamples * 2);
    }

    for (auto& buf : chanBuffers_)
        buf = std::make_unique<DisplayBuffer>(config_.displayCapacity);
    decoder_ = std::make_unique<can::LiveDecoder>(
        rateHz_, bitrate_.load(), threshold_.load(), 0.7f);
    trigger_ = std::make_unique<can::VirtualTrigger>(
        rateHz_, bitrate_.load(), threshold_.load(), config_.triggerMode,
        config_.holdoffUs);

    if (!config_.logCanPath.empty())
        canLogger_ = std::make_unique<CanLogger>(config_.logCanPath);
    if (!config_.logRawPath.empty())
        rawLogger_ = std::make_unique<RawLogger>(
            config_.logRawPath, rateHz_, calVPC_, calOC_);

    if (stream_) stream_->start();
    else fileStream_->start();
    analyzerThread_ = std::thread(&ScopeApp::analyzerMain, this);

    if (config_.noGui) return runHeadless();

    // ---- VolcanoPlot -------------------------------------------------
    volcano::backend::BackendDesc desc;
    desc.width = config_.width;
    desc.height = config_.height;
    desc.windowTitle = "UliCAN — Live CAN scope";
    backend_ = volcano::backend::createScreenBackend(desc);
    renderer_ = std::make_unique<volcano::render::Renderer>(*backend_);
    // Two stacked scopes sharing the X (time) viewport: CANH + CANL on
    // top, the differential trace below. Both axes get the same xlim;
    // updateScopePlots mirrors manual navigation between them.
    figure_ = std::make_unique<vp::Figure>(2, 1);
    axesTop_ = figure_->addAxes(0, 0);
    axesTop_->setStyle(vp::styles::ggplotStyle());
    axesTop_->setTitle("CANH / CANL (live)");
    axesTop_->style().xAxis.label = "";
    axesTop_->style().yAxis.label = "V";
    axesTop_->setXlim(0.0f, 20.0f);
    axesTop_->setYlim(-0.5f, 5.0f);

    axesDiff_ = figure_->addAxes(1, 0);
    axesDiff_->setStyle(vp::styles::ggplotStyle());
    axesDiff_->setTitle("CANH − CANL");
    axesDiff_->style().xAxis.label = "t [ms]";
    axesDiff_->style().yAxis.label = "diff [V]";
    axesDiff_->setXlim(0.0f, 20.0f);
    axesDiff_->setYlim(-0.2f, 2.2f);

    using vp::Color;
    vp::Series2D canhMinS, canhMaxS, canlMinS, canlMaxS, minS, maxS,
        thrS, loS, hiS, markS, markTopS, trigTopS, trigDiffS;
    // CANH gold, CANL teal — min/max decimation in dark/light shades.
    canhMinS.color = Color::fromRgba8(150, 95, 10);
    canhMinS.lineWidth = 1.3f;
    canhMaxS.color = Color::fromRgba8(235, 175, 55);
    canhMaxS.lineWidth = 1.3f;
    canhMaxS.label = "CANH";
    canlMinS.color = Color::fromRgba8(20, 100, 105);
    canlMinS.lineWidth = 1.3f;
    canlMaxS.color = Color::fromRgba8(80, 195, 200);
    canlMaxS.lineWidth = 1.3f;
    canlMaxS.label = "CANL";
    minS.color = Color::fromRgba8(30, 90, 160);
    minS.lineWidth = 1.3f;
    maxS.color = Color::fromRgba8(120, 170, 230);
    maxS.lineWidth = 1.3f;
    thrS.color = Color::fromRgba8(220, 60, 60);
    thrS.lineStyle = vp::LineStyle::Dashed;
    thrS.lineWidth = 1.2f;
    loS.color = Color::fromRgba8(120, 120, 120);
    loS.lineStyle = vp::LineStyle::Dashed;
    loS.lineWidth = 1.0f;
    hiS = loS;
    markS.color = Color::fromRgba8(240, 150, 40);
    markS.marker = vp::MarkerStyle::TriDown;
    markS.size = 8.0f;
    markS.lineStyle = vp::LineStyle::None;
    markTopS = markS;
    // Vertical line at the centered trigger position.
    trigTopS.color = Color::fromRgba8(240, 150, 40);
    trigTopS.lineStyle = vp::LineStyle::Dotted;
    trigTopS.lineWidth = 1.0f;
    trigDiffS = trigTopS;
    for (auto* s : {&canhMinS, &canhMaxS, &canlMinS, &canlMaxS, &minS,
                    &maxS, &thrS, &loS, &hiS, &markS, &markTopS,
                    &trigTopS, &trigDiffS}) {
        s->usePropCycle = false;
        // LinePlot::prepare uploads unconditionally — an empty series
        // would create a zero-sized Vulkan buffer. One off-screen point
        // keeps the layer valid until real data arrives.
        s->points = {{0.0f, -50.0f}};
    }

    canhMinLine_ = static_cast<vp::LinePlot*>(
        axesTop_->addPlot(std::make_shared<vp::LinePlot>(canhMinS)));
    canhMaxLine_ = static_cast<vp::LinePlot*>(
        axesTop_->addPlot(std::make_shared<vp::LinePlot>(canhMaxS)));
    canlMinLine_ = static_cast<vp::LinePlot*>(
        axesTop_->addPlot(std::make_shared<vp::LinePlot>(canlMinS)));
    canlMaxLine_ = static_cast<vp::LinePlot*>(
        axesTop_->addPlot(std::make_shared<vp::LinePlot>(canlMaxS)));
    markerTopLine_ = static_cast<vp::LinePlot*>(
        axesTop_->addPlot(std::make_shared<vp::LinePlot>(markTopS)));
    trigLineTop_ = static_cast<vp::LinePlot*>(
        axesTop_->addPlot(std::make_shared<vp::LinePlot>(trigTopS)));

    minLine_ = static_cast<vp::LinePlot*>(
        axesDiff_->addPlot(std::make_shared<vp::LinePlot>(minS)));
    maxLine_ = static_cast<vp::LinePlot*>(
        axesDiff_->addPlot(std::make_shared<vp::LinePlot>(maxS)));
    thrLine_ = static_cast<vp::LinePlot*>(
        axesDiff_->addPlot(std::make_shared<vp::LinePlot>(thrS)));
    limLoLine_ = static_cast<vp::LinePlot*>(
        axesDiff_->addPlot(std::make_shared<vp::LinePlot>(loS)));
    limHiLine_ = static_cast<vp::LinePlot*>(
        axesDiff_->addPlot(std::make_shared<vp::LinePlot>(hiS)));
    markerLine_ = static_cast<vp::LinePlot*>(
        axesDiff_->addPlot(std::make_shared<vp::LinePlot>(markS)));
    trigLineDiff_ = static_cast<vp::LinePlot*>(
        axesDiff_->addPlot(std::make_shared<vp::LinePlot>(trigDiffS)));
    frameEdgesOk_ = static_cast<vp::Vlines*>(axesDiff_->addPlot(
        std::make_shared<vp::Vlines>(std::vector<float>{}, 0.0f, 0.0f,
                                     vp::Color::fromRgba8(60, 170, 90, 200),
                                     1.0f)));
    frameEdgesBad_ = static_cast<vp::Vlines*>(axesDiff_->addPlot(
        std::make_shared<vp::Vlines>(std::vector<float>{}, 0.0f, 0.0f,
                                     vp::Color::fromRgba8(230, 80, 70, 200),
                                     1.0f)));

    // ---- Widgets ------------------------------------------------------
    messageList_ = figure_->addWidget<MessageListWidget>();
    auto rect = [](int32_t x, int32_t y, uint32_t w, uint32_t h) {
        return vp::Rect2D{x, y, w, h};
    };
    thrSlider_ = figure_->addWidget<vp::Slider>(
        rect(16, 0, 200, 30), "Trigger V", 0.05f, 5.0f,
        config_.thresholdV, 0);
    // Logarithmic stepped holdoff: slider 0..35 maps to
    // 10^(0.2*v) µs → 1 µs … 10 s in 36 detents (1-2-5 scheme).
    holdoffSlider_ = figure_->addWidget<vp::Slider>(
        rect(224, 0, 200, 30), "", 0.0f, 35.0f,
        static_cast<float>(std::log10(
            std::clamp(config_.holdoffUs, 1.0, 1e7)) /
            0.2),
        35);
    bitrateButton_ = figure_->addWidget<vp::Button>(rect(432, 0, 140, 30), "");
    modeButton_ =
        figure_->addWidget<vp::Button>(rect(580, 0, 160, 30), "");
    autoButton_ =
        figure_->addWidget<vp::Button>(rect(748, 0, 120, 30), "");
    followButton_ =
        figure_->addWidget<vp::Button>(rect(876, 0, 110, 30), "");
    yautoButton_ =
        figure_->addWidget<vp::Button>(rect(994, 0, 100, 30), "Y-Auto");
    clearButton_ =
        figure_->addWidget<vp::Button>(rect(1102, 0, 90, 30), "Clear");

    thrSlider_->onChanged = [this](float v) { threshold_.store(v); };
    holdoffSlider_->onChanged = [this](float v) {
        const double us = std::pow(10.0, static_cast<double>(v) * 0.2);
        holdoffUs_.store(us);
        holdoffSlider_->label = "Holdoff " + formatUs(us);
    };
    bitrateButton_->onClick = [this] {
        bitrateIndex_ = (bitrateIndex_ + 1) % 4;
        bitrate_.store(kBitrates[bitrateIndex_]);
        bitrateButton_->label = kBitrateLabels[bitrateIndex_];
    };
    modeButton_->onClick = [this] {
        triggerMode_.store((triggerMode_.load() + 1) % 3);
        static const char* names[3] = {"Trigger: sof", "Trigger: error",
                                       "Trigger: dominant"};
        modeButton_->label = names[triggerMode_.load() % 3];
    };
    autoButton_->onClick = [this] {
        trigAuto_.store(!trigAuto_.load());
        autoButton_->label = trigAuto_.load() ? "Trig: auto"
                                              : "Trig: normal";
    };
    followButton_->onClick = [this] {
        follow_.store(!follow_.load());
    };
    yautoButton_->onClick = [this] {
        yAuto_.store(!yAuto_.load());
        lastAutoYTopValid_ = false;
        lastAutoYDiffValid_ = false;
    };
    clearButton_->onClick = [this] {
        if (messageList_) messageList_->clear();
        // Force re-arm: drop the freeze and its snapshot.
        lastAcceptSeen_ = -1;
        snapshot_.valid = false;
    };
    for (size_t i = 0; i < kBitrates.size(); ++i)
        if (kBitrates[i] == config_.bitrate) bitrateIndex_ = int(i);
    bitrateButton_->label = kBitrateLabels[bitrateIndex_];
    static const char* kTriggerNames[3] = {
        "Trigger: sof", "Trigger: error", "Trigger: dominant"};
    modeButton_->label =
        kTriggerNames[static_cast<int>(config_.triggerMode) % 3];
    autoButton_->label = "Trig: auto";
    followButton_->label = "Follow: on";
    yautoButton_->label = "Y-Auto: on";
    holdoffSlider_->label = "Holdoff " + formatUs(config_.holdoffUs);

    // ---- Navigation ----------------------------------------------------
    auto& nav = figure_->nav();
    nav.scrollZoom = true;
    // Scope semantics: wheel / zoom-rect affect the time axis only;
    // the Y axis is governed by Y-Auto.
    nav.constrainX = true;
    nav.onQuitRequest = [this] { running_.store(false); };
    nav.onFullscreenToggle = [this] { backend_->toggleFullscreen(); };
    nav.onCursorMove = [this](std::string_view s) {
        cursorText_ = std::string(s);
    };

    // RUN/STOP like a real scope: spacebar freezes the display.
    // Debounced — GLFW key repeat would toggle continuously.
    figure_->canvas().connect("key_press_event",
                              [this](const vp::Event& e) {
        if (e.key == "r" || e.key == "R") {
            // Screen reset: resume, drop freeze+snapshot, re-enable
            // follow + Y-auto, force re-center/refit next frame.
            paused_.store(false, std::memory_order_relaxed);
            lastAcceptSeen_ = -1;
            snapshot_.valid = false;
            follow_.store(true, std::memory_order_relaxed);
            yAuto_.store(true, std::memory_order_relaxed);
            lastAutoValid_ = false;
            syncedXValid_ = false;
            lastAutoYTopValid_ = false;
            lastAutoYDiffValid_ = false;
            titleMode_ = -1;
            return;
        }
        if (e.key != " " && e.key != "space") return;
        const auto now = std::chrono::steady_clock::now();
        if (now - lastSpaceToggle_ < std::chrono::milliseconds(250))
            return;
        lastSpaceToggle_ = now;
        paused_.store(!paused_.load(std::memory_order_relaxed),
                      std::memory_order_relaxed);
        if (backend_)
            backend_->setWindowTitle(
                paused_.load(std::memory_order_relaxed)
                    ? "UliCAN — STOPPED (space to run)"
                    : "UliCAN");
    });

    if (config_.stats)
        std::cerr << "[ulican] backend created, entering render loop\n";
    renderer_->prepare(*figure_);
    while (backend_->pollEvents() && running_.load(std::memory_order_relaxed) &&
           !g_sigStop.load(std::memory_order_relaxed)) {
        if (!renderer_->processInput(*figure_)) break;
        uiFrame();
        renderer_->renderIfStale(*figure_);
    }
    if (config_.stats)
        std::cerr << "[ulican] render loop ended (window closed)\n";

    shutdown();
    return 0;
}

// ---------------------------------------------------------------------
// Headless mode (--no-gui): capture/decode/log until SIGINT/SIGTERM.
// ---------------------------------------------------------------------

int ScopeApp::runHeadless() {
    if (config_.stats)
        std::cerr << "[ulican] headless mode — Ctrl-C stoppt\n";
    auto lastStats = std::chrono::steady_clock::now();
    while (running_.load(std::memory_order_relaxed) &&
           !g_sigStop.load(std::memory_order_relaxed) &&
           !(fileStream_ && fileStream_->finished() &&
             batchQueue_.empty())) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!config_.stats) continue;
        const auto now = std::chrono::steady_clock::now();
        if (now - lastStats < std::chrono::seconds(1)) continue;
        lastStats = now;
        uint64_t done = 0, dropped = 0;
        if (stream_) {
            done = stream_->completedBatches();
            dropped = stream_->droppedBatches();
        } else if (fileStream_) {
            done = fileStream_->completedBatches();
        }
        std::cerr << std::format(
            "[ulican] {} frames, {} missing-ack | trigger: {} events/"
            "{} view-accepts/{} candidates | {} batches done, {} dropped "
            "| can-log: {} frames/{} trig | raw-log: {} samples{}\n",
            decoder_->framesTotal, decoder_->missingAck,
            trigger_->dbgEvents, trigger_->dbgAccepted,
            trigger_->dbgCandidates, done, dropped,
            canLogger_ ? canLogger_->frames() : 0,
            canLogger_ ? canLogger_->triggers() : 0,
            rawLogger_ ? rawLogger_->samples() : 0,
            fileStream_ && fileStream_->finished() ? " (eof)" : "");
    }
    if (config_.stats)
        std::cerr << std::format(
            "[ulican] headless exit: {} frames, {} missing-ack | trigger: "
            "{} events/{} view-accepts/{} candidates | can-log: {} "
            "frames/{} trig | raw-log: {} samples{}\n",
            decoder_->framesTotal, decoder_->missingAck,
            trigger_->dbgEvents, trigger_->dbgAccepted,
            trigger_->dbgCandidates,
            canLogger_ ? canLogger_->frames() : 0,
            canLogger_ ? canLogger_->triggers() : 0,
            rawLogger_ ? rawLogger_->samples() : 0,
            fileStream_ && fileStream_->finished() ? " (eof)" : "");
    shutdown();
    return 0;
}

// ---------------------------------------------------------------------
// Analyzer thread: USB batches -> ADC->voltage diff -> decoder + trigger.
// ---------------------------------------------------------------------

void ScopeApp::analyzerMain() {
    decoder_->onFrame = [this](const can::FrameEvent& ev) {
        if (canLogger_) canLogger_->frame(ev);
        uiQueue_.push(UiEvent{UiEvent::Kind::Frame, ev, {}});
    };
    std::unique_ptr<IsoBatch> batch;
    uint64_t expectedSeq = 0;
    bool haveSeq = false;
    while (running_.load(std::memory_order_relaxed)) {
        if (!batchQueue_.pop(batch)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (!haveSeq) {
            expectedSeq = batch->sequence;
            haveSeq = true;
        }
        if (batch->sequence != expectedSeq) {
            // Batch dropped upstream: do not join the decoder tail across
            // the gap (positions stay absolute, samples are compressed).
            decoder_->reset();
            trigger_->reset();
        }
        expectedSeq = batch->sequence + 1;

        if (rawLogger_)
            rawLogger_->write(batch->data.data(), batch->data.size());

        // Split the batch at bad USB intervals (mirrors live_can.py).
        size_t begin = 0, end = 0;
        for (const auto& packet : batch->packets) {
            if (packet.status == 0) {
                end += packet.length;
                continue;
            }
            processSegment(batch->data.data() + begin, (end - begin) / 2,
                           batch->firstSample + static_cast<int64_t>(begin) / 2);
            decoder_->reset();
            trigger_->reset();
            begin = end;
        }
        processSegment(batch->data.data() + begin, (end - begin) / 2,
                       batch->firstSample + static_cast<int64_t>(begin) / 2);
        batch.reset();
    }
}

void ScopeApp::processSegment(const uint8_t* bytes, size_t sampleCount,
                              int64_t origin) {
    if (!sampleCount) return;
    // Sync UI-adjustable settings (only touched here on this thread).
    const float thr = threshold_.load(std::memory_order_relaxed);
    const uint32_t br = bitrate_.load(std::memory_order_relaxed);
    decoder_->setThreshold(thr);
    decoder_->setBitrate(br);
    trigger_->setThreshold(thr);
    trigger_->setBitrate(br);
    trigger_->setMode(static_cast<can::VirtualTrigger::Mode>(
        triggerMode_.load(std::memory_order_relaxed) % 3));
    trigger_->setHoldoffUs(holdoffUs_.load(std::memory_order_relaxed));

    // Calibrated channel voltages: v_i = vpc_i * (adc_i - oc_i).
    canhScratch_.resize(sampleCount);
    canlScratch_.resize(sampleCount);
    diffScratch_.resize(sampleCount);
    const float s0 = static_cast<float>(calVPC_[0]);
    const float o0 = static_cast<float>(calOC_[0]);
    const float s1 = static_cast<float>(calVPC_[1]);
    const float o1 = static_cast<float>(calOC_[1]);
    for (size_t i = 0; i < sampleCount; ++i) {
        const float v0 = s0 * (static_cast<float>(bytes[2 * i]) - o0);
        const float v1 =
            s1 * (static_cast<float>(bytes[2 * i + 1]) - o1);
        canhScratch_[i] = v0;
        canlScratch_[i] = v1;
        diffScratch_[i] = v0 - v1;
    }
    {
        std::lock_guard lock(displayMutex_);
        chanBuffers_[0]->append(canhScratch_.data(), sampleCount, origin);
        chanBuffers_[1]->append(canlScratch_.data(), sampleCount, origin);
        chanBuffers_[2]->append(diffScratch_.data(), sampleCount, origin);
    }
    decoder_->analyze(diffScratch_, origin);
    for (const auto& event : trigger_->scan(diffScratch_, origin)) {
        if (canLogger_) canLogger_->trigger(event);
        uiQueue_.push(UiEvent{UiEvent::Kind::Trigger, {}, event});
    }
    // The view anchor is the newest holdoff-accepted candidate —
    // holdoff gates only the picture, never the trigger event stream.
    latestAccepted_.store(trigger_->lastAcceptedPos(),
                          std::memory_order_relaxed);
}

// ---------------------------------------------------------------------
// UI thread: drain events, decimate the display window, drive VolcanoPlot.
// ---------------------------------------------------------------------

void ScopeApp::uiFrame() {
    const bool paused = paused_.load(std::memory_order_relaxed);
    if (paused) {
        // STOP: the scope/markers stay frozen on the snapshot taken
        // at pause time, but decoded CAN frames keep flowing into
        // the message list (and the Klipper streams stay in sync).
        UiEvent ev;
        while (uiQueue_.pop(ev)) {
            if (ev.kind != UiEvent::Kind::Frame) continue;
            if (config_.packetFormat == PacketFormat::Klipper &&
                ev.frame.frame.identifier >= 256) {
                auto& stream =
                    klipperStreams_[ev.frame.frame.identifier];
                if (klipperDictLoaded_) stream.setDict(&klipperDict_);
                for (auto& b : stream.feed(
                         {ev.frame.frame.data.data(),
                          ev.frame.frame.dlc})) {
                    if (!ev.frame.decoded.empty())
                        ev.frame.decoded += "; ";
                    ev.frame.decoded += b.text;
                }
            }
            if (messageList_) messageList_->addFrame(ev.frame);
            ++framesSeen_;
        }
        updateScopePlots();
        updateWidgets();
        return;
    }
    UiEvent event;
    while (uiQueue_.pop(event)) {
        if (event.kind == UiEvent::Kind::Frame) {
            if (config_.packetFormat == PacketFormat::Klipper &&
                event.frame.frame.identifier >= 256) {
                if (!klipperDictLoaded_ &&
                    !config_.klipperDictPath.empty()) {
                    klipperDictLoaded_ = true;
                    std::string err;
                    if (!klipperDict_.load(config_.klipperDictPath, &err))
                        std::cerr << "ulican: klipper dict: " << err
                                  << "\n";
                    else if (config_.stats)
                        std::cerr << std::format(
                            "[klipper] dict: {} cmds, {} resps, "
                            "{} outputs\n",
                            klipperDict_.commands.size(),
                            klipperDict_.responses.size(),
                            klipperDict_.outputs.size());
                }
                auto& stream =
                    klipperStreams_[event.frame.frame.identifier];
                if (klipperDictLoaded_) stream.setDict(&klipperDict_);
                for (auto& b : stream.feed(
                         {event.frame.frame.data.data(),
                          event.frame.frame.dlc})) {
                    if (!event.frame.decoded.empty())
                        event.frame.decoded += "; ";
                    event.frame.decoded += b.text;
                }
            }
            if (messageList_) messageList_->addFrame(event.frame);
            frameMarks_.push_back(event.frame);
            if (frameMarks_.size() > 512) frameMarks_.pop_front();
            ++framesSeen_;
        } else {
            // Trigger events feed the scope markers and the JSONL log
            // only — the message list shows CAN frames exclusively.
            triggerMarks_.push_back(event.trigger);
            if (triggerMarks_.size() > 512) triggerMarks_.pop_front();
            ++triggersSeen_;
        }
    }
    updateScopePlots();
    updateWidgets();
    const uint64_t batchesDone =
        stream_ ? stream_->completedBatches()
                : (fileStream_ ? fileStream_->completedBatches() : 0);
    if (++frameCounter_ % 30 == 0 && backend_) {
        backend_->setWindowTitle(std::format(
            "UliCAN — {} frames · {} triggers · {} dropped",
            framesSeen_, triggersSeen_,
            stream_ ? stream_->droppedBatches() : 0));
        cursorText_.clear();
    }
    if (config_.stats && frameCounter_ % 300 == 0) {
        size_t buffered = 0;
        {
            std::lock_guard lock(displayMutex_);
            if (chanBuffers_[2]) buffered = chanBuffers_[2]->size();
        }
        std::cerr << std::format(
            "[ulican] {} frames decoded, {} triggers, {} batches dropped/{} "
            "done, {} samples buffered, decoder: {} total/{} missing-ack\n",
            framesSeen_, triggersSeen_,
            stream_ ? stream_->droppedBatches() : 0, batchesDone, buffered,
            decoder_ ? decoder_->framesTotal : 0,
            decoder_ ? decoder_->missingAck : 0);
        if (trigger_) {
            std::cerr << std::format(
                "[ulican]   trigger: {} scans, {} candidates, {} seen-skip, "
                "{} holdoff-skip, {} accepted, evalUpTo={} lastAcc={}\n",
                trigger_->dbgScans, trigger_->dbgCandidates,
                trigger_->dbgSeenSkip, trigger_->dbgHoldoffSkip,
                trigger_->dbgAccepted, trigger_->dbgEvaluatedUpTo,
                trigger_->dbgLastAccepted);
        }
        if (stream_) {
            std::cerr << std::format("[ulican]   stream: {} submit-errors\n",
                                     stream_->submitErrors());
        }
    }
}

void ScopeApp::updateScopePlots() {
    const double msPerSample = 1e3 / rateHz_;
    const double windowMs =
        static_cast<double>(windowSamples_) * msPerSample;

    int64_t origin = 0, head = 0;
    {
        std::lock_guard lock(displayMutex_);
        origin = chanBuffers_[2]->origin();
        head = chanBuffers_[2]->head();
        bufferOrigin_ = origin;
        bufferHead_ = head;
    }

    // ---- X viewport --------------------------------------------------
    // Trigger-centered view (scope semantics): the anchor is the newest
    // holdoff-accepted trigger. Once the centered window is fully
    // populated the sweep is held (normal: until the next accept; auto:
    // plus timeout). While held or paused the display buffers are
    // snapshotted so zoom/pan resolves real samples — the frozen
    // picture stays fully navigable. The trigger event stream itself
    // runs continuously.
    double vx0 = 0.0, vx1 = windowMs;
    double anchorMs = -1.0;
    bool frozen = false;
    if (axesTop_ && axesDiff_ && head > origin) {
        const int64_t halfW =
            static_cast<int64_t>(windowSamples_ / 2);
        const int64_t maxAnchor = head - halfW;
        const int64_t cand =
            latestAccepted_.load(std::memory_order_relaxed);

        // ---- freeze management ----
        // Every newly accepted trigger pauses the screen for the
        // holdoff window — measured in stream samples, so the freeze
        // lasts exactly as long as it takes the stream to move past
        // the post-trigger holdoff window (a new accept can only
        // happen after it). Auto releases once the window has passed,
        // normal holds until the next accept arrives.
        const int64_t holdoffSamples = static_cast<int64_t>(
            holdoffUs_.load(std::memory_order_relaxed) * rateHz_ / 1e6);
        const bool paused = paused_.load(std::memory_order_relaxed);
        if (cand != lastAcceptSeen_) {
            lastAcceptSeen_ = cand;
            // Refresh the freeze snapshot on a new accept — but not
            // while paused: the spacebar snapshot must stay put, or
            // the picture would keep chasing fresh accepts.
            if (cand >= 0 && !paused) snapshot_.valid = false;
        }
        const bool holdFrozen =
            cand >= 0 && (trigAuto_.load(std::memory_order_relaxed)
                              ? head < cand + holdoffSamples
                              : true);
        frozen = holdFrozen || paused;
        holdFrozenUi_ = holdFrozen;
        if (frozen && !snapshot_.valid) {
            // Copy the whole buffer at freeze time so zoom/pan
            // resolves real samples while acquisition runs on.
            // The coordinate anchor must move with every retake —
            // a new accept during a freeze refreshes the snapshot
            // with a newer origin, and a stale anchor would map the
            // view outside the copied range (empty screen).
            frozenCoordOrigin_ = origin;
            const int64_t f0 = origin, f1 = head;
            std::lock_guard lock(displayMutex_);
            for (int c = 0; c < 3; ++c)
                chanBuffers_[static_cast<size_t>(c)]->extractRange(
                    f0, static_cast<size_t>(f1 - f0),
                    snapshot_.ch[static_cast<size_t>(c)]);
            snapshot_.origin = f0;
            snapshot_.head =
                f0 + static_cast<int64_t>(snapshot_.ch[0].size());
            snapshot_.valid = true;
        }
        if (!frozen) snapshot_.valid = false;

        const int64_t anchor =
            cand >= origin + halfW ? std::min(cand, maxAnchor)
                                   : maxAnchor;
        // The trigger line keeps its last position while frozen.
        anchorMs = frozen ? -1.0 : (anchor - origin) * msPerSample;
        double x0 = anchorMs - windowMs * 0.5;
        double x1 = anchorMs + windowMs * 0.5;
        if (x0 < 0.0) {
            x1 -= x0;
            x0 = 0.0;
        }
        if (!frozen && follow_.load(std::memory_order_relaxed)) {
            const auto t = axesTop_->xlim();
            const auto d = axesDiff_->xlim();
            const bool manual =
                lastAutoValid_ &&
                (std::fabs(t.min - lastAutoX0_) > 0.02 ||
                 std::fabs(t.max - lastAutoX1_) > 0.02 ||
                 std::fabs(d.min - lastAutoX0_) > 0.02 ||
                 std::fabs(d.max - lastAutoX1_) > 0.02);
            if (manual) {
                follow_.store(false, std::memory_order_relaxed);
            } else {
                axesTop_->setXlim(static_cast<float>(x0),
                                  static_cast<float>(x1));
                axesDiff_->setXlim(static_cast<float>(x0),
                                   static_cast<float>(x1));
                lastAutoX0_ = x0;
                lastAutoX1_ = x1;
                lastAutoValid_ = true;
                syncedX0_ = x0;
                syncedX1_ = x1;
                syncedXValid_ = true;
            }
        }
        if (frozen || !follow_.load(std::memory_order_relaxed)) {
            const auto t = axesTop_->xlim();
            const auto d = axesDiff_->xlim();
            if (!syncedXValid_) {
                syncedX0_ = t.min;
                syncedX1_ = t.max;
                syncedXValid_ = true;
            }
            const auto moved = [this](const auto& r) {
                return std::fabs(r.min - syncedX0_) > 0.02 ||
                       std::fabs(r.max - syncedX1_) > 0.02;
            };
            if (moved(t)) {
                axesDiff_->setXlim(t.min, t.max);
                syncedX0_ = t.min;
                syncedX1_ = t.max;
            } else if (moved(d)) {
                axesTop_->setXlim(d.min, d.max);
                syncedX0_ = d.min;
                syncedX1_ = d.max;
            }
        }
        const auto cur = axesTop_->xlim();
        vx0 = cur.min;
        vx1 = cur.max;
    }

    // While frozen the x coordinates stay anchored to the origin that
    // was live at freeze time; data comes from the snapshot then.
    const int64_t coordBase = frozen ? frozenCoordOrigin_ : origin;
    const int64_t dataOrigin = frozen ? snapshot_.origin : origin;
    const int64_t dataEnd = frozen ? snapshot_.head : head;

    // Visible absolute-index range from the x viewport (ms → samples).
    int64_t vFirst =
        coordBase + static_cast<int64_t>(std::floor(vx0 / msPerSample));
    int64_t vLast =
        coordBase + static_cast<int64_t>(std::ceil(vx1 / msPerSample));
    vFirst = std::max<int64_t>(vFirst, dataOrigin);
    vLast = std::min<int64_t>(vLast, dataEnd);

    // Re-decimate only when the view changed or reaches into the live
    // edge; a static historical view keeps its last decimation.
    const bool dirty = vLast <= vFirst || vFirst != decVF_ ||
                       vLast != decVL_ || coordBase != decOrigin_ ||
                       vLast >= decHead_;
    const int64_t viewSpan = vLast - vFirst;

    vp::LinePlot* minL[3] = {canhMinLine_, canlMinLine_, minLine_};
    vp::LinePlot* maxL[3] = {canhMaxLine_, canlMaxLine_, maxLine_};

    if (dirty && canhMinLine_ && minLine_) {
        decVF_ = vFirst;
        decVL_ = vLast;
        decOrigin_ = coordBase;
        decHead_ = dataEnd;
        bool any = false;
        if (viewSpan > 0) {
            const auto extent = backend_->extent();
            for (int ch = 0; ch < 3; ++ch) {
                size_t cnt = 0;
                if (frozen) {
                    cnt = snapshot_.extract(
                        ch, vFirst, static_cast<size_t>(viewSpan),
                        window_);
                } else {
                    std::lock_guard lock(displayMutex_);
                    cnt = chanBuffers_[ch]->extractRange(
                        vFirst, static_cast<size_t>(viewSpan), window_);
                }
                if (!cnt) continue;
                size_t cols = std::min<size_t>(
                    {static_cast<size_t>(extent.width) / 2, cnt,
                     static_cast<size_t>(3000)});
                cols = std::max<size_t>(cols, 1);
                xs_.resize(cols);
                auto& lo = decCh_[ch].ymin;
                auto& hi = decCh_[ch].ymax;
                lo.resize(cols);
                hi.resize(cols);
                for (size_t c = 0; c < cols; ++c) {
                    const size_t i0 = c * cnt / cols;
                    const size_t i1 =
                        std::max(i0 + 1, (c + 1) * cnt / cols);
                    float l = std::numeric_limits<float>::max();
                    float h = std::numeric_limits<float>::lowest();
                    for (size_t i = i0; i < i1 && i < cnt; ++i) {
                        l = std::min(l, window_[i]);
                        h = std::max(h, window_[i]);
                    }
                    const double mid = vFirst + (i0 + i1) * 0.5;
                    xs_[c] = static_cast<float>((mid - coordBase) *
                                                msPerSample);
                    lo[c] = l;
                    hi[c] = h;
                }
                minL[ch]->setData(xs_, lo);
                maxL[ch]->setData(xs_, hi);
                any = true;
            }
        }
        if (!any) {
            // View outside the retained buffer: blank the traces.
            const std::vector<float> blankX{0.0f}, blankY{-50.0f};
            for (int ch = 0; ch < 3; ++ch) {
                minL[ch]->setData(blankX, blankY);
                maxL[ch]->setData(blankX, blankY);
                decCh_[ch].ymin.clear();
                decCh_[ch].ymax.clear();
            }
            xs_.clear();
        }
    }

    // Visible extents from the (fresh or cached) decimation.
    float topLo = 0.0f, topHi = 0.0f, diffLo = 0.0f, diffHi = 0.0f;
    bool haveData = !decCh_[2].ymin.empty();
    if (haveData) {
        const auto vecMin = [](const std::vector<float>& v) {
            return *std::min_element(v.begin(), v.end());
        };
        const auto vecMax = [](const std::vector<float>& v) {
            return *std::max_element(v.begin(), v.end());
        };
        diffLo = vecMin(decCh_[2].ymin);
        diffHi = vecMax(decCh_[2].ymax);
        topLo = std::min(vecMin(decCh_[0].ymin), vecMin(decCh_[1].ymin));
        topHi = std::max(vecMax(decCh_[0].ymax), vecMax(decCh_[1].ymax));
    }

    // Continuous Y auto-fit per axes over the visible columns; a manual
    // ylim change on either axes disables it until the button re-arms.
    if (haveData && yAuto_.load(std::memory_order_relaxed)) {
        const auto fit = [](vp::Axes* ax, float lo, float hi, bool& valid,
                            double& y0, double& y1) {
            const float pad = std::max(0.1f, (hi - lo) * 0.15f);
            const auto yr = ax->ylim();
            if (valid &&
                (std::fabs(yr.min - y0) > 0.02 ||
                 std::fabs(yr.max - y1) > 0.02))
                return false; // user-adjusted → stop auto-fitting
            ax->setYlim(lo - pad, hi + pad);
            y0 = lo - pad;
            y1 = hi + pad;
            valid = true;
            return true;
        };
        const bool okT =
            fit(axesTop_, topLo, topHi, lastAutoYTopValid_,
                lastAutoYTop0_, lastAutoYTop1_);
        const bool okD =
            fit(axesDiff_, diffLo, diffHi, lastAutoYDiffValid_,
                lastAutoYDiff0_, lastAutoYDiff1_);
        if (!okT || !okD)
            yAuto_.store(false, std::memory_order_relaxed);
    }

    // Threshold + limit guide lines spanning the diff viewport.
    if (thrLine_ && limLoLine_ && limHiLine_ && axesDiff_) {
        const auto xr = axesDiff_->xlim();
        guideX_ = {xr.min, xr.max};
        const float thr = threshold_.load(std::memory_order_relaxed);
        thrLine_->setData(guideX_, {thr, thr});
        limLoLine_->setData(guideX_, {0.5f, 0.5f});
        limHiLine_->setData(guideX_, {0.9f, 0.9f});
    }

    // Vertical trigger line at the anchor position — in follow mode this
    // is the window center; with manual nav it tracks the newest
    // candidate (or the live-edge anchor) in view coordinates.
    if (anchorMs >= 0.0) {
        const std::vector<float> tx{static_cast<float>(anchorMs),
                                    static_cast<float>(anchorMs)};
        if (trigLineTop_) {
            const auto yr = axesTop_->ylim();
            trigLineTop_->setData(tx, {yr.min, yr.max});
        }
        if (trigLineDiff_) {
            const auto yr = axesDiff_->ylim();
            trigLineDiff_->setData(tx, {yr.min, yr.max});
        }
    }

    // Trigger markers inside the retained buffer, on both scopes.
    if (markerLine_ || markerTopLine_) {
        while (!triggerMarks_.empty() &&
               triggerMarks_.front().position < dataOrigin)
            triggerMarks_.pop_front();
        std::vector<float> mx;
        mx.reserve(triggerMarks_.size());
        for (const auto& ev : triggerMarks_) {
            if (ev.position > dataEnd) continue;
            mx.push_back(static_cast<float>(
                (ev.position - coordBase) * msPerSample));
        }
        const size_t n = mx.size();
        const auto markYs = [](vp::Axes* ax, size_t count) {
            const auto yr = ax->ylim();
            return std::vector<float>(
                count, yr.max - yr.span() * 0.04f);
        };
        if (markerLine_)
            markerLine_->setData(std::vector<float>(mx),
                                 markYs(axesDiff_, n));
        if (markerTopLine_)
            markerTopLine_->setData(std::move(mx),
                                    markYs(axesTop_, n));
    }

    // Packet overlay: frame-edge lines plus a label centred over each
    // decoded frame on the diff axes — only once frames span enough
    // pixels (zoom gate), so zoomed-out views stay clean.
    if (axesDiff_) {
        auto& texts = axesDiff_->texts();
        texts.clear();
        while (!frameMarks_.empty() &&
               frameMarks_.front().position < dataOrigin)
            frameMarks_.pop_front();
        const auto yr = axesDiff_->ylim();
        const float yTop = yr.max - yr.span() * 0.10f;
        const float yRow = yr.span() * 0.09f;
        const double msPerPx = (vx1 - vx0) /
                               std::max(1.0, double(axesDiff_->rect.width));
        const vp::Color okCol = vp::Color::fromRgba8(35, 150, 80);
        const vp::Color badCol = vp::Color::fromRgba8(200, 55, 45);
        std::vector<float> edgesOk, edgesBad;
        // Newest frames first (live edge), two staggered rows; a label
        // is skipped when it would overlap an already-placed one.
        double leftEdge[2] = {std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max()};
        for (auto it = frameMarks_.rbegin();
             it != frameMarks_.rend(); ++it) {
            const auto& fe = *it;
            // Only frames fully inside the view get an overlay.
            if (fe.position < vFirst || fe.end > vLast) continue;
            const bool ok = fe.frame.ack && fe.frame.ackDelimiterOk &&
                            fe.frame.eofOk &&
                            !fe.frame.dominantErrorAfterAck;
            const double x0 =
                (fe.position - coordBase) * msPerSample;
            const double x1 = (fe.end - coordBase) * msPerSample;
            const double framePx = (x1 - x0) / msPerPx;
            if (framePx < 8.0) continue;
            auto& dst = ok ? edgesOk : edgesBad;
            dst.push_back(static_cast<float>(x0));
            dst.push_back(static_cast<float>(x1));
            std::string code;
            if (!fe.frame.ack) code += "+NAK";
            if (!fe.frame.ackDelimiterOk) code += "+ADEL";
            if (!fe.frame.eofOk) code += "+EOF";
            if (fe.frame.dominantErrorAfterAck) code += "+DOM";
            if (!code.empty()) code.erase(0, 1);
            const std::string txt = std::format(
                "{} {}", formatPacketLabel(fe, config_.packetFormat),
                ok ? "✓" : std::format("{} ✗", code));
            const double xm = (x0 + x1) * 0.5;
            // ~6.5 px per 9pt char + padding, in data units.
            const double wData =
                (static_cast<double>(txt.size()) * 6.5 + 12.0) * msPerPx;
            // The label only appears once the box fits inside the frame.
            if (wData > x1 - x0) continue;
            const double right = xm + wData * 0.5;
            const double left = xm - wData * 0.5;
            const int row =
                right < leftEdge[0] ? 0 : right < leftEdge[1] ? 1 : -1;
            if (row < 0) continue;
            leftEdge[row] = left;
            auto* a = axesDiff_->text(
                static_cast<float>(xm),
                yTop - static_cast<float>(row) * yRow, txt,
                vp::CoordSystem::Data);
            a->fontSize = 9.0f;
            a->color = ok ? okCol : badCol;
            a->halign = vp::HAlign::Center;
            a->valign = vp::VAlign::Top;
            a->hasBbox = true;
            a->bboxFaceColor = ok
                ? vp::Color::fromRgba8(238, 250, 240, 235)
                : vp::Color::fromRgba8(252, 240, 238, 235);
            a->bboxEdgeColor = a->color;
            a->bboxEdgeWidth = 1.0f;
            a->bboxPadding = 3.0f;
            a->boxStyle = vp::parseBoxStyle("round,pad=0.3");
        }
        if (frameEdgesOk_)
            frameEdgesOk_->setData(std::move(edgesOk), yr.min, yr.max);
        if (frameEdgesBad_)
            frameEdgesBad_->setData(std::move(edgesBad), yr.min, yr.max);
    }
}

void ScopeApp::updateWidgets() {
    const auto extent = backend_->extent();
    const uint32_t w = extent.width, h = extent.height;
    if (!w || !h) return;

    messageList_->rect = vp::Rect2D{
        0, static_cast<int32_t>(static_cast<int32_t>(h) - kListHeight), w,
        static_cast<uint32_t>(kListHeight)};
    figure_->grid().left = 0.06f;
    figure_->grid().right = 0.99f;
    figure_->grid().top = 0.96f;
    figure_->grid().bottom =
        static_cast<float>(kListHeight + kControlHeight + 26) /
        static_cast<float>(h);
    figure_->grid().hspace = 0.25f;

    const int32_t cy = static_cast<int32_t>(h) - kListHeight -
                       kControlHeight + 4;
    thrSlider_->rect = vp::Rect2D{16, cy, 200, 30};
    holdoffSlider_->rect = vp::Rect2D{224, cy, 200, 30};
    bitrateButton_->rect = vp::Rect2D{432, cy, 140, 30};
    modeButton_->rect = vp::Rect2D{580, cy, 160, 30};
    autoButton_->rect = vp::Rect2D{748, cy, 120, 30};
    followButton_->rect = vp::Rect2D{876, cy, 110, 30};
    yautoButton_->rect = vp::Rect2D{994, cy, 100, 30};
    clearButton_->rect = vp::Rect2D{1102, cy, 90, 30};

    const int mode = paused_.load() ? 1 : holdFrozenUi_ ? 2 : 0;
    if (mode != titleMode_) {
        titleMode_ = mode;
        if (backend_)
            backend_->setWindowTitle(
                mode == 1 ? "UliCAN — STOPPED (space to run)"
                          : mode == 2 ? "UliCAN — HOLDOFF"
                                      : "UliCAN");
    }
    followButton_->label = mode == 1 ? "STOPPED (space)"
                       : mode == 2 ? "HOLDOFF"
                       : follow_.load() ? "Follow: on" : "Follow: off";
    yautoButton_->label = yAuto_.load() ? "Y-Auto: on" : "Y-Auto: off";
}

} // namespace ucan
