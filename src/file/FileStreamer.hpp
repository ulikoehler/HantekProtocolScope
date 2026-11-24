#pragma once

#include "../device/IsoStream.hpp"
#include "../util/RawFormat.hpp"
#include "../util/SpscQueue.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace ucan {

/// Replays a `--log-raw` .bin file through the same IsoBatch pipeline
/// the USB stream uses. The file is memory-mapped read-only; a producer
/// thread chops it into USB-batch-sized IsoBatch objects. Unlike
/// IsoStream nothing is dropped — a full queue just applies backpressure.
class FileStreamer {
public:
    /// Throws std::runtime_error on missing/invalid file.
    FileStreamer(const std::string& path,
                 SpscQueue<std::unique_ptr<IsoBatch>, 128>& out);
    ~FileStreamer();

    FileStreamer(const FileStreamer&) = delete;
    FileStreamer& operator=(const FileStreamer&) = delete;

    void start();
    void stop();

    [[nodiscard]] double rateHz() const noexcept { return rateHz_; }
    [[nodiscard]] const RawFileHeader& header() const noexcept {
        return header_;
    }
    [[nodiscard]] uint64_t completedBatches() const noexcept {
        return completedBatches_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool finished() const noexcept {
        return finished_.load(std::memory_order_relaxed);
    }

private:
    void run();
    void close();

    SpscQueue<std::unique_ptr<IsoBatch>, 128>& out_;
    int fd_ = -1;
    void* map_ = nullptr;
    size_t mapSize_ = 0;
    const uint8_t* data_ = nullptr;
    size_t dataBytes_ = 0;
    RawFileHeader header_{};
    double rateHz_ = 0.0;
    std::thread thread_;
    std::atomic<bool> stopping_{false};
    std::atomic<bool> finished_{false};
    std::atomic<uint64_t> completedBatches_{0};
};

} // namespace ucan
