#include <cstdio>
#include <string>

#include "rovsim/simulation.hpp"

static void usage() {
    std::printf("usage: run_sim <scenario> [output.csv]\n       run_sim --list\nscenarios:");
    for (const auto& n : rovsim::scenario_names()) std::printf(" %s", n.c_str());
    std::printf("\n");
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }
    const std::string arg = argv[1];
    if (arg == "--list") {
        for (const auto& n : rovsim::scenario_names()) std::printf("%s\n", n.c_str());
        return 0;
    }

    rovsim::Scenario sc;
    if (!rovsim::make_scenario(arg, sc)) {
        std::fprintf(stderr, "unknown scenario '%s'\n", arg.c_str());
        usage();
        return 2;
    }

    const auto rows = rovsim::run_scenario(sc);
    const std::string out = argc >= 3 ? argv[2] : (sc.name + ".csv");
    if (!rovsim::write_csv(out, rows)) {
        std::fprintf(stderr, "cannot write '%s'\n", out.c_str());
        return 1;
    }

    // Summary: report each safety-state change with its time.
    std::printf("scenario: %s  (%zu samples) -> %s\n", sc.name.c_str(), rows.size(), out.c_str());
    rovsim::SafetyState prev = rows.front().state;
    for (const auto& r : rows) {
        if (r.state != prev) {
            std::printf("  t=%6.1fs  %s -> %s  (depth %.2f m, battery %.1f%%)\n", r.time_s,
                        rovsim::to_string(prev), rovsim::to_string(r.state), r.true_depth_m,
                        100.0 * r.battery_frac);
            prev = r.state;
        }
    }
    const auto& last = rows.back();
    std::printf("final: state=%s depth=%.2f m battery=%.1f%%\n", rovsim::to_string(last.state),
                last.true_depth_m, 100.0 * last.battery_frac);
    return 0;
}
