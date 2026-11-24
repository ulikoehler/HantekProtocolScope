#pragma once

#include "../util/SpscQueue.hpp"

#include <libusb-1.0/libusb.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

struct libusb_transfer;

namespace ucan {

class HantekDevice;

/// One isochronous USB interval inside a transfer.
struct UsbPacket {
    uint32_t length = 0;  // payload bytes when status == 0
    int32_t status = 0;   // 0 ok, <0 libusb/mapped error
};

/// 64 intervals of interleaved CH1/CH2 ADC bytes plus their status index.
struct IsoBatch {
    /// Completion order across all collected transfers (including
    /// warmup discards); gaps mean dropped batches.
    uint64_t sequence = 0;
    /// Absolute index of the first valid sample in `data`, counting all
    /// stream samples since acquisition start.
    int64_t firstSample = 0;
    std::vector<UsbPacket> packets;
    std::vector<uint8_t> data;
};

/// Isochronous ADC stream on the 6022BL firmware's alternate interface 1,
/// endpoint 0x82 — port of iso_stream.py plus the async transfer loop.
class IsoStream {
public:
    static constexpr int kIntervalsPerTransfer = 64;
    static constexpr uint32_t kBytesPerInterval = 3072;
    static constexpr size_t kTransferSize =
        kIntervalsPerTransfer * kBytesPerInterval;
    static constexpr int kDepth = 12;

    /// out: batches are moved in completion order. Batches dropped on a
    /// full queue are counted in droppedBatches().
    IsoStream(HantekDevice& device, SpscQueue<std::unique_ptr<IsoBatch>, 128>& out);
    ~IsoStream();

    IsoStream(const IsoStream&) = delete;
    IsoStream& operator=(const IsoStream&) = delete;

    /// Claim-free: expects interface 0 claimed; selects alt 1, queues all
    /// transfers, starts acquisition and the event thread.
    void start();
    /// Stop acquisition, cancel transfers and join the thread.
    void stop();

    [[nodiscard]] uint64_t droppedBatches() const noexcept {
        return droppedBatches_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t completedBatches() const noexcept {
        return completedBatches_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t submitErrors() const noexcept {
        return submitErrors_.load(std::memory_order_relaxed);
    }

private:
    struct Slot {
        IsoStream* owner = nullptr;
        libusb_transfer* transfer = nullptr;
        std::vector<uint8_t> buffer;
    };

    static void LIBUSB_CALL onTransfer(libusb_transfer* transfer);
    void run();
    void collect(Slot& slot);
    /// Returns false when libusb refuses the transfer (device gone,
    /// overrun). Never throws — safe to call from the event callback.
    bool submit(Slot& slot) noexcept;

    HantekDevice& device_;
    SpscQueue<std::unique_ptr<IsoBatch>, 128>& out_;
    std::vector<std::unique_ptr<Slot>> slots_;
    std::thread thread_;
    std::atomic<bool> stopping_{false};
    std::atomic<int> activeTransfers_{0};
    std::atomic<uint64_t> droppedBatches_{0};
    std::atomic<uint64_t> completedBatches_{0};
    std::atomic<uint64_t> submitErrors_{0};
    int discardRemaining_ = 0;
    // Event-thread only: batch sequence + absolute valid-sample origin.
    uint64_t nextSequence_ = 0;
    int64_t nextOrigin_ = 0;
};

} // namespace ucan
