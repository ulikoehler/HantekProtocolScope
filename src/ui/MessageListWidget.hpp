#pragma once

#include "../can/CanTypes.hpp"

#include <volcano/plot/Widgets.hpp>

#include <deque>
#include <string>
#include <vector>

namespace ucan {

/// Scrollable CAN message table drawn as a VolcanoPlot widget along the
/// bottom of the canvas. Newest message on top; mouse wheel scrolls back.
class MessageListWidget : public volcano::plot::Widget {
public:
    MessageListWidget() { rect = {0, 0, 100, 160}; }

    void addFrame(const can::FrameEvent& event);
    void clear();

    bool handleEvent(const volcano::plot::Event& e) override;
    void draw(volcano::plot::WidgetPainter& p) override;

private:
    struct Row {
        double timeMs = 0;
        uint32_t id = 0;
        uint8_t dlc = 0;
        std::array<uint8_t, 8> data{};
        bool ack = false, ackDelimOk = false, eofOk = false,
             domErr = false;
        uint16_t crc = 0;
        float recMargin = 0, domMargin = 0;
    };

    std::deque<Row> rows_;
    size_t scroll_ = 0;
    static constexpr size_t kMaxRows = 400;
    static constexpr float kHeaderH = 22.0f;
    static constexpr float kRowH = 19.0f;
    static constexpr float kPad = 8.0f;

    void layout(float width) const;
};

} // namespace ucan
