// ===========================================================================
// dataset_main — headless CLI that generates a supervised-learning dataset of
// rigid-body trajectories using the ML Environment + DatasetGenerator.
//
// No rendering, no GLFW: this links only the physics translation units, so it
// runs anywhere (CI, a training box) without a display.
//
// Usage:
//   DatasetGenerator [output.csv] [--episodes N] [--frames N] [--horizon N]
//                    [--seed N] [--min-bodies N] [--max-bodies N]
//
// Defaults produce a small, quick dataset. Every run is deterministic for a
// given seed.
// ===========================================================================

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "../ml/DatasetGenerator.h"

namespace {

// Parse an integer flag value; leaves `out` unchanged on a missing/bad value.
bool parseIntArg(int argc, char** argv, int& i, int& out) {
    if (i + 1 >= argc) { std::cerr << "missing value for " << argv[i] << "\n"; return false; }
    out = std::atoi(argv[++i]);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    DatasetGenerator::Config cfg;
    std::string outPath = "dataset.csv";

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (a[0] != '-') {
            outPath = a; // first positional argument = output path
        } else if (std::strcmp(a, "--episodes") == 0) {
            if (!parseIntArg(argc, argv, i, cfg.episodes)) return 1;
        } else if (std::strcmp(a, "--frames") == 0) {
            if (!parseIntArg(argc, argv, i, cfg.frames)) return 1;
        } else if (std::strcmp(a, "--horizon") == 0) {
            if (!parseIntArg(argc, argv, i, cfg.horizon)) return 1;
        } else if (std::strcmp(a, "--min-bodies") == 0) {
            if (!parseIntArg(argc, argv, i, cfg.minBodies)) return 1;
        } else if (std::strcmp(a, "--max-bodies") == 0) {
            if (!parseIntArg(argc, argv, i, cfg.maxBodies)) return 1;
        } else if (std::strcmp(a, "--seed") == 0) {
            int s = 0;
            if (!parseIntArg(argc, argv, i, s)) return 1;
            cfg.seed = static_cast<unsigned>(s);
        } else if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
            std::cout <<
                "Usage: DatasetGenerator [output.csv] [--episodes N] [--frames N]\n"
                "                        [--horizon N] [--seed N]\n"
                "                        [--min-bodies N] [--max-bodies N]\n"
                "Generates a supervised dataset: each row is a body's state plus\n"
                "its position `horizon` frames in the future (default 30).\n";
            return 0;
        } else {
            std::cerr << "unknown argument: " << a << "\n";
            return 1;
        }
    }

    // Sanity: keep body counts in a sensible order/range.
    if (cfg.minBodies < 1) cfg.minBodies = 1;
    if (cfg.maxBodies < cfg.minBodies) cfg.maxBodies = cfg.minBodies;

    std::cout << "[dataset] episodes=" << cfg.episodes
              << " frames=" << cfg.frames
              << " horizon=" << cfg.horizon
              << " bodies=[" << cfg.minBodies << "," << cfg.maxBodies << "]"
              << " seed=" << cfg.seed << "\n";

    DatasetGenerator gen(cfg);
    if (!gen.generate(outPath)) {
        std::cerr << "[dataset] failed to write " << outPath << "\n";
        return 1;
    }

    std::cout << "[dataset] wrote " << gen.lastRowCount()
              << " rows to " << outPath << "\n";
    return 0;
}
