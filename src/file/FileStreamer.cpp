#include "FileStreamer.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace ucan {

namespace {
constexpr size_t kChunkBytes = 64 * 3072; // one USB batch = 98304 samples
}

FileStreamer::FileStreamer(
    const std::string& path,
    SpscQueue<std::unique_ptr<IsoBatch>, 128>& out)
    : out_(out) {
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0)
        throw std::runtime_error("Raw-Datei öffnen: " + path + ": " +
                                 std::strerror(errno));
    struct stat st {};
    if (::fstat(fd_, &st) != 0 || st.st_size < 0) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("Raw-Datei stat: " + path);
    }
    mapSize_ = static_cast<size_t>(st.st_size);
    if (mapSize_ < sizeof(RawFileHeader)) {
        close();
        throw std::runtime_error("Raw-Datei zu klein für Header: " + path);
    }
    map_ = ::mmap(nullptr, mapSize_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if (map_ == MAP_FAILED) {
        map_ = nullptr;
        close();
        throw std::runtime_error("Raw-Datei mmap: " + path + ": " +
                                 std::strerror(errno));
    }
    std::memcpy(&header_, map_, sizeof(RawFileHeader));
    if (!rawHeaderValid(header_)) {
        close();
        throw std::runtime_error("Keine gültige UliCAN-Raw-Datei: " + path);
    }
    data_ = static_cast<const uint8_t*>(map_) + sizeof(RawFileHeader);
    dataBytes_ = mapSize_ - sizeof(RawFileHeader);
    if (dataBytes_ & 1) --dataBytes_; // whole sample pairs only
    rateHz_ = header_.rateHz;
}

FileStreamer::~FileStreamer() { stop(); }

void FileStreamer::close() {
    if (map_) {
        ::munmap(map_, mapSize_);
        map_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void FileStreamer::start() {
    stopping_.store(false);
    thread_ = std::thread(&FileStreamer::run, this);
}

void FileStreamer::stop() {
    stopping_.store(true, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
    close();
}

void FileStreamer::run() {
    size_t offset = 0;
    uint64_t sequence = 0;
    int64_t firstSample = 0;
    while (!stopping_.load(std::memory_order_relaxed)) {
        const size_t remaining = dataBytes_ - offset;
        if (!remaining) break;
        size_t take = std::min(remaining, kChunkBytes);
        take -= take & 1; // whole sample pairs only
        if (!take) break;

        auto batch = std::make_unique<IsoBatch>();
        batch->sequence = sequence++;
        batch->firstSample = firstSample;
        firstSample += static_cast<int64_t>(take / 2);
        batch->packets.push_back({static_cast<uint32_t>(take), 0});
        batch->data.assign(data_ + offset, data_ + offset + take);
        offset += take;

        // Replay must not drop data: wait for queue space.
        while (!stopping_.load(std::memory_order_relaxed) &&
               !out_.push(std::move(batch)))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        completedBatches_.fetch_add(1, std::memory_order_relaxed);
    }
    finished_.store(true, std::memory_order_relaxed);
}

} // namespace ucan
