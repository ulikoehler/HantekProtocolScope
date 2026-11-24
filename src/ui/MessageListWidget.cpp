#include "MessageListWidget.hpp"

#include <algorithm>
#include <format>
#include <volcano/plot/Types.hpp>

namespace ucan {

namespace vp = volcano::plot;

void MessageListWidget::addFrame(const can::FrameEvent& event) {
    Row row;
    row.isTrigger = false;
    row.timeMs = event.timeUs / 1000.0;
    row.id = event.frame.identifier;
    row.dlc = event.frame.dlc;
    row.data = event.frame.data;
    row.ack = event.frame.ack;
    row.ackDelimOk = event.frame.ackDelimiterOk;
    row.eofOk = event.frame.eofOk;
    row.domErr = event.frame.dominantErrorAfterAck;
    row.crc = event.frame.crc;
    row.recMargin = event.recessiveMarginV;
    row.domMargin = event.dominantMarginV;
    rows_.push_front(row);
    if (rows_.size() > kMaxRows) rows_.pop_back();
}

void MessageListWidget::addTrigger(const can::TriggerEvent& event) {
    Row row;
    row.isTrigger = true;
    row.timeMs = event.timeUs / 1000.0;
    row.triggerReason = event.reasonText();
    rows_.push_front(row);
    if (rows_.size() > kMaxRows) rows_.pop_back();
}

void MessageListWidget::clear() {
    rows_.clear();
    scroll_ = 0;
}

bool MessageListWidget::handleEvent(const vp::Event& e) {
    if (!visible || !active) return false;
    if (e.type == vp::Event::Type::Scroll && hitTest(e.x, e.y)) {
        const int steps = e.step > 0 ? 3 : -3;
        const long long next =
            static_cast<long long>(scroll_) + steps * 3;
        const size_t maxScroll = rows_.size();
        scroll_ = static_cast<size_t>(
            std::clamp<long long>(next, 0, static_cast<long long>(maxScroll)));
        return true;
    }
    return false;
}

void MessageListWidget::layout(float width) const {
    (void)width;
}

void MessageListWidget::draw(vp::WidgetPainter& p) {
    if (!visible) return;
    p.fillRect(rect, vp::Color::fromRgba8(28, 30, 34));
    p.outlineRect(rect, vp::Color::fromRgba8(90, 95, 105), 1.0f);

    const float x0 = rect.x + kPad;
    const float w = rect.width - 2 * kPad;
    const vp::Color kHeader = vp::Color::fromRgba8(210, 214, 222);
    const vp::Color kDim = vp::Color::fromRgba8(140, 146, 158);
    const vp::Color kOk = vp::Color::fromRgba8(120, 220, 140);
    const vp::Color kWarn = vp::Color::fromRgba8(240, 200, 90);
    const vp::Color kBad = vp::Color::fromRgba8(240, 110, 100);
    const vp::Color kTrig = vp::Color::fromRgba8(240, 160, 60);
    const vp::Color kFg = vp::Color::fromRgba8(220, 224, 232);

    // Column layout (fractions of w).
    struct Col { const char* title; float x, w; };
    const std::vector<Col> cols = {
        {"t [ms]", 0.00f, 0.09f}, {"ID", 0.09f, 0.07f},
        {"DLC", 0.16f, 0.05f}, {"DATA", 0.21f, 0.34f},
        {"CRC", 0.55f, 0.07f}, {"ACK", 0.62f, 0.09f},
        {"EOF", 0.71f, 0.07f}, {"ERR", 0.78f, 0.07f},
        {"RES R/D [V]", 0.85f, 0.15f},
    };
    auto colX = [&](size_t i) { return x0 + cols[i].x * w; };

    // Header
    const float hy = rect.y + kHeaderH * 0.5f + 4.0f;
    for (const auto& col : cols)
        p.text(col.title, x0 + col.x * w, hy, kHeader, 0.72f);
    const vp::Point2D sep[2] = {
        {static_cast<float>(rect.x) + kPad,
         static_cast<float>(rect.y) + kHeaderH},
        {static_cast<float>(rect.x) + static_cast<float>(rect.width) - kPad,
         static_cast<float>(rect.y) + kHeaderH}};
    p.line(sep, vp::Color::fromRgba8(70, 75, 85), 1.0f);

    const float rowTop = rect.y + kHeaderH;
    const size_t visibleRows =
        static_cast<size_t>((rect.height - kHeaderH) / kRowH);
    for (size_t r = 0; r < visibleRows && scroll_ + r < rows_.size();
         ++r) {
        const Row& row = rows_[scroll_ + r];
        const float ry = rowTop + r * kRowH + kRowH * 0.5f + 4.0f;
        if (row.isTrigger) {
            p.text(std::format("TRIGGER {:<14} {:>10.3f}", row.triggerReason,
                               row.timeMs),
                   colX(0), ry, kTrig, 0.75f);
            continue;
        }
        const bool ok = row.ack && row.ackDelimOk && row.eofOk && !row.domErr;
        const vp::Color status =
            ok ? kOk : (!row.ack ? kWarn : kBad);
        p.text(std::format("{:>10.3f}", row.timeMs), colX(0), ry,
               kFg, 0.75f);
        p.text(std::format("0x{:03X}", row.id), colX(1), ry, kFg, 0.75f);
        p.text(std::format("{}", row.dlc), colX(2), ry, kFg, 0.75f);
        std::string hex;
        for (int i = 0; i < row.dlc; ++i) {
            if (i) hex += ' ';
            hex += std::format("{:02X}", static_cast<unsigned>(row.data[i]));
        }
        p.text(hex, colX(3), ry, kFg, 0.75f);
        p.text(std::format("0x{:03X}", row.crc), colX(4), ry, kOk, 0.75f);
        p.text(row.ack ? "dom" : "--", colX(5), ry,
               row.ack ? status : kDim, 0.75f);
        p.text((row.ackDelimOk && row.eofOk) ? "ok" : "bad", colX(6), ry,
               (row.ackDelimOk && row.eofOk) ? kOk : kBad, 0.75f);
        p.text(row.domErr ? "DOM!" : "", colX(7), ry, kBad, 0.75f);
        const float worst = std::min(row.recMargin, row.domMargin);
        p.text(std::format("{:+.2f}/{:+.2f}", row.recMargin, row.domMargin),
               colX(8), ry, worst < 0 ? kWarn : kOk, 0.72f);
    }
}

} // namespace ucan
