#include "IsoStream.hpp"

#include "HantekDevice.hpp"

#include <chrono>
#include <cstring>
#include <libusb-1.0/libusb.h>
#include <stdexcept>

namespace ucan {

IsoStream::IsoStream(HantekDevice& device,
                     SpscQueue<std::unique_ptr<IsoBatch>, 128>& out)
    : device_(device), out_(out) {}

IsoStream::~IsoStream() { stop(); }

void IsoStream::start() {
    device_.setAltSetting(1);
    slots_.reserve(kDepth);
    for (int i = 0; i < kDepth; ++i) {
        auto slot = std::make_unique<Slot>();
        slot->owner = this;
        slot->buffer.resize(kTransferSize);
        slot->transfer = libusb_alloc_transfer(kIntervalsPerTransfer);
        if (!slot->transfer)
            throw std::runtime_error("Kein USB-Transferpuffer verfügbar");
        libusb_fill_iso_transfer(slot->transfer, device_.handle(), 0x82,
                                 slot->buffer.data(),
                                 static_cast<int>(kTransferSize),
                                 kIntervalsPerTransfer, onTransfer, slot.get(),
                                 500);
        for (int packet = 0; packet < kIntervalsPerTransfer; ++packet)
            slot->transfer->iso_packet_desc[packet].length = kBytesPerInterval;
        if (!submit(*slot))
            throw std::runtime_error("Hantek-USB-Transfer anfordern fehlgeschlagen");
        slots_.push_back(std::move(slot));
    }
    const uint8_t startAcquisition = 1;
    device_.controlWrite(0xE3, &startAcquisition, 1);
    discardRemaining_ = kDepth;
    activeTransfers_.store(kDepth, std::memory_order_relaxed);
    thread_ = std::thread(&IsoStream::run, this);
}

bool IsoStream::submit(Slot& slot) noexcept {
    const int status = libusb_submit_transfer(slot.transfer);
    if (status == LIBUSB_SUCCESS) return true;
    submitErrors_.fetch_add(1, std::memory_order_relaxed);
    return false;
}

void IsoStream::collect(Slot& slot) {
    auto batch = std::make_unique<IsoBatch>();
    batch->packets.reserve(kIntervalsPerTransfer);
    size_t validBytes = 0;
    for (int i = 0; i < kIntervalsPerTransfer; ++i) {
        const auto& packet = slot.transfer->iso_packet_desc[i];
        const bool ok = packet.status == LIBUSB_TRANSFER_COMPLETED &&
                        packet.actual_length > 0 &&
                        packet.actual_length <= kBytesPerInterval &&
                        (packet.actual_length % 2) == 0;
        batch->packets.push_back(
            {ok ? packet.actual_length : 0u,
             ok ? 0 : (packet.status != LIBUSB_TRANSFER_COMPLETED
                           ? static_cast<int32_t>(packet.status)
                           : (packet.actual_length == 0
                                  ? -1
                                  : -2))});
        if (ok) validBytes += packet.actual_length;
    }
    batch->data.reserve(validBytes);
    for (int i = 0; i < kIntervalsPerTransfer; ++i) {
        const auto& packet = slot.transfer->iso_packet_desc[i];
        const auto& info = batch->packets[i];
        if (info.status == 0)
            batch->data.insert(batch->data.end(),
                               slot.buffer.data() + i * kBytesPerInterval,
                               slot.buffer.data() + i * kBytesPerInterval +
                                   info.length);
    }
    batch->sequence = nextSequence_++;
    batch->firstSample = nextOrigin_;
    nextOrigin_ += static_cast<int64_t>(validBytes / 2);
    completedBatches_.fetch_add(1, std::memory_order_relaxed);
    if (discardRemaining_ > 0) {
        --discardRemaining_;
        return;
    }
    if (!out_.push(std::move(batch)))
        droppedBatches_.fetch_add(1, std::memory_order_relaxed);
}

void LIBUSB_CALL IsoStream::onTransfer(libusb_transfer* transfer) {
    auto* slot = static_cast<Slot*>(transfer->user_data);
    IsoStream* self = slot->owner;
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED && !self->stopping_) {
        self->collect(*slot);
        // Resubmit failed → this slot no longer has a transfer in flight.
        if (!self->submit(*slot))
            self->activeTransfers_.fetch_sub(1, std::memory_order_release);
    } else {
        self->activeTransfers_.fetch_sub(1, std::memory_order_release);
    }
}

void IsoStream::run() {
    while (activeTransfers_.load(std::memory_order_acquire) > 0) {
        timeval timeout{0, 100000};
        libusb_handle_events_timeout(device_.context(), &timeout);
    }
}

void IsoStream::stop() {
    if (!thread_.joinable()) return;
    stopping_ = true;
    const uint8_t stopAcquisition = 0;
    try {
        device_.controlWrite(0xE3, &stopAcquisition, 1);
    } catch (...) {
    }
    for (auto& slot : slots_)
        if (slot->transfer) libusb_cancel_transfer(slot->transfer);
    thread_.join();
    device_.setAltSetting(0);
    for (auto& slot : slots_) {
        if (slot->transfer) {
            libusb_free_transfer(slot->transfer);
            slot->transfer = nullptr;
        }
    }
    slots_.clear();
}

} // namespace ucan
