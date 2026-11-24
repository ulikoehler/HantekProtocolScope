// UliCAN — live CAN scope on the Hantek DSO-6022BL (VolcanoPlot GUI).
//
//   ulican live [options]            capture vom Oszilloskop (GUI oder
//                                    --no-gui), dekodiert + triggert live
//   ulican view FILE.bin [options]   Raw-.bin (von --log-raw) per mmap
//                                    visualisieren, inklusive Decoding
//
// Ohne Subcommand wird 'live' angenommen.
#include "ui/ScopeApp.hpp"

#include <argparse/argparse.hpp>

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef UCAN_FIRMWARE_DEFAULT
#define UCAN_FIRMWARE_DEFAULT ""
#endif

namespace {

using argparse::ArgumentParser;

// Options shared by 'live' and 'view' (decoder, trigger, logging, window).
void addCommonArgs(ArgumentParser& p) {
    p.add_argument("--bitrate")
        .help("CAN-Bitrate in bit/s (Standard: 1000000)")
        .scan<'i', int>()
        .default_value(1000000);
    p.add_argument("--threshold")
        .help("Trigger-Schwelle in Volt (Standard: 1.1)")
        .scan<'g', double>()
        .default_value(1.1);
    p.add_argument("--holdoff-us")
        .help("Trigger-Holdoff in Mikrosekunden (Standard: 1000; "
              "GUI-Slider: 1 us - 10 s logarithmisch)")
        .scan<'g', double>()
        .default_value(1000.0);
    p.add_argument("--trigger")
        .help("error | dominant (Standard: error)")
        .default_value(std::string("error"));
    p.add_argument("--window-ms")
        .help("angezeigte Zeitbasis in ms (Standard: 20)")
        .scan<'g', double>()
        .default_value(20.0);
    p.add_argument("--width")
        .help("Fensterbreite (Standard: 1600)")
        .scan<'i', int>()
        .default_value(1600);
    p.add_argument("--height")
        .help("Fensterhöhe (Standard: 950)")
        .scan<'i', int>()
        .default_value(950);
    p.add_argument("--stats")
        .help("Statistik zeilenweise auf stderr")
        .default_value(false)
        .implicit_value(true);
    p.add_argument("--no-gui")
        .help("Kein Fenster: nur capturen/dekodieren/loggen")
        .default_value(false)
        .implicit_value(true);
    p.add_argument("--log-can")
        .help("dekodierte Frames/Trigger als JSON Lines loggen")
        .default_value(std::string(""));
    p.add_argument("--log-raw")
        .help("rohe ADC-Paare mit Header loggen (für 'view')")
        .default_value(std::string(""));
    p.add_argument("--packets")
        .help("Scope-Overlay-Format: raw oder klipper")
        .default_value(std::string("raw"));
}

void fillCommon(const ArgumentParser& p, ucan::AppConfig& c) {
    c.bitrate = static_cast<uint32_t>(p.get<int>("--bitrate"));
    c.thresholdV = static_cast<float>(p.get<double>("--threshold"));
    c.holdoffUs = p.get<double>("--holdoff-us");
    const std::string mode = p.get<std::string>("--trigger");
    if (mode == "error") {
        c.triggerMode = ucan::can::VirtualTrigger::Mode::Error;
    } else if (mode == "dominant") {
        c.triggerMode = ucan::can::VirtualTrigger::Mode::Dominant;
    } else {
        throw std::runtime_error("--trigger: error oder dominant erwartet");
    }
    c.windowMs = p.get<double>("--window-ms");
    c.width = static_cast<uint32_t>(p.get<int>("--width"));
    c.height = static_cast<uint32_t>(p.get<int>("--height"));
    c.stats = p.get<bool>("--stats");
    c.noGui = p.get<bool>("--no-gui");
    c.logCanPath = p.get<std::string>("--log-can");
    c.logRawPath = p.get<std::string>("--log-raw");
    const std::string pkts = p.get<std::string>("--packets");
    if (pkts == "raw") {
        c.packetFormat = ucan::PacketFormat::Raw;
    } else if (pkts == "klipper") {
        c.packetFormat = ucan::PacketFormat::Klipper;
    } else {
        throw std::runtime_error("--packets: raw oder klipper erwartet");
    }
}

} // namespace

int main(int argc, char** argv) {
    // parse_args skips args[0] like argv[0] — keep the program name.
    // Bare `ulican ...` defaults to the `live` subcommand; `-h/--help`
    // at top level shows the program help instead.
    std::vector<std::string> args(argv, argv + argc);
    const bool topLevelHelp =
        args.size() > 1 && (args[1] == "-h" || args[1] == "--help");
    if (args.size() > 1 && args[1] != "live" && args[1] != "view" &&
        !topLevelHelp)
        args.insert(args.begin() + 1, "live");

    ArgumentParser prog("ulican",
                        "Live-CAN-Oszilloskop auf dem Hantek DSO-6022BL");
    prog.add_description(
        "Capture, Decode und virtuelles Triggern von Classic-CAN auf "
        "CAN_H/CAN_L. Subcommands: 'live' (Device) und 'view' (.bin-Replay).");

    ArgumentParser live("live");
    live.add_description("Capture vom Oszilloskop — GUI oder --no-gui");
    live.add_argument("--firmware")
        .help("RAM-Firmware-Intel-Hex")
        .default_value(std::string(UCAN_FIRMWARE_DEFAULT));
    live.add_argument("--rate")
        .help("Abtastrate in MS/s: 10, 12 oder 15 (Standard: 10)")
        .scan<'i', int>()
        .default_value(10);
    live.add_argument("--probe")
        .help("Tastkopffaktor: --probe KANAL 1|10")
        .metavar("KANAL FAKTOR")
        .nargs(2)
        .scan<'i', int>();
    live.add_argument("--gain")
        .help("Kanalskalierung: --gain KANAL 1|2|5|10")
        .metavar("KANAL FAKTOR")
        .nargs(2)
        .scan<'i', int>();
    addCommonArgs(live);

    ArgumentParser view("view");
    view.add_description(
        "Raw-.bin (von --log-raw) per mmap visualisieren + dekodieren");
    view.add_argument("file")
        .help(".bin Datei (RawFileHeader + ADC-Paare)")
        .required();
    addCommonArgs(view);

    prog.add_subparser(live);
    prog.add_subparser(view);

    try {
        prog.parse_args(args);

        ucan::AppConfig config;
        if (prog.is_subcommand_used("view")) {
            const auto& v = prog.at<ArgumentParser>("view");
            config.viewFile = v.get<std::string>("file");
            fillCommon(v, config);
        } else {
            const auto& l = prog.at<ArgumentParser>("live");
            fillCommon(l, config);
            config.firmwarePath = l.get<std::string>("--firmware");
            config.rateMHz = l.get<int>("--rate");
            if (const auto probe = l.present<std::vector<int>>("--probe")) {
                if (probe->size() != 2 || (*probe)[0] < 0 ||
                    (*probe)[0] > 1) {
                    throw std::runtime_error(
                        "--probe: Kanal muss 0 oder 1 sein");
                }
                config.probes[(*probe)[0]] = (*probe)[1];
            }
            if (const auto gain = l.present<std::vector<int>>("--gain")) {
                if (gain->size() != 2 || (*gain)[0] < 0 ||
                    (*gain)[0] > 1) {
                    throw std::runtime_error(
                        "--gain: Kanal muss 0 oder 1 sein");
                }
                config.gains[(*gain)[0]] = (*gain)[1];
            }
            if (config.rateMHz != 10 && config.rateMHz != 12 &&
                config.rateMHz != 15) {
                throw std::runtime_error(
                    "--rate: nur 10, 12 oder 15 MS/s unterstützt");
            }
            if (config.firmwarePath.empty()) {
                throw std::runtime_error(
                    "Keine Firmware-Datei: --firmware angeben oder "
                    "deps/dso6022bl-firmware.hex bereitstellen");
            }
        }

        ucan::ScopeApp app(std::move(config));
        return app.run();
    } catch (const std::exception& e) {
        std::cerr << "ulican: " << e.what() << "\n";
        return 2;
    }
}
