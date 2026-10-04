#include "apk_model.h"
#include "pulse_conversion.h"

#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

bool parseRow(const std::string& line, std::vector<double>& values)
{
    values.clear();
    std::stringstream input(line);
    std::string part;
    while (std::getline(input, part, ',')) {
        try {
            values.push_back(std::stod(part));
        } catch (...) {
            return false;
        }
    }
    return values.size() == 31 || values.size() == 55;
}

} // namespace

int main()
{
    const apk_model::RobotConfig config = apk_model::makeDefaultConfig();
    std::string line;
    while (std::getline(std::cin, line)) {
        std::vector<double> values;
        if (!parseRow(line, values)) {
            std::cerr << "expected CSV row: seq, body[6], layer[6], feet[18], optional active[6], angles[18]\n";
            return 2;
        }

        const long long sequence = static_cast<long long>(values[0]);
        apk_model::BodyState state{};
        apk_model::Pose layer{};
        state.body.xyz = {values[1], values[2], values[3]};
        state.body.uvw = {values[4], values[5], values[6]};
        layer.xyz = {values[7], values[8], values[9]};
        layer.uvw = {values[10], values[11], values[12]};
        for (int leg = 0; leg < 6; ++leg) {
            const size_t offset = static_cast<size_t>(13 + (leg * 3));
            state.feet[leg] = {values[offset], values[offset + 1], values[offset + 2]};
        }

        std::array<std::array<double, 3>, 6> angles{};
        std::array<bool, 6> active = {true, true, true, true, true, true};
        if (values.size() == 55) {
            for (int leg=0; leg<6; ++leg) {
                active[leg] = values[31+leg] != 0;
                for (int joint=0; joint<3; ++joint) angles[leg][joint] = values[37+3*leg+joint];
            }
        }
        const bool valid = apk_model::inverseKinematics(config, state, layer, active, angles);
        std::array<int, 18> pulses{};
        if (valid) {
            for (int leg = 0; leg < 6; ++leg) {
                for (int joint = 0; joint < 3; ++joint) {
                    const int pin = apk_model::ServoPinByApkLeg[leg][joint];
                    pulses[pin] = chica_apk_angle_to_pulse(angles[leg][joint], leg, joint);
                }
            }
        }

        std::cout << sequence << ',' << (valid ? 1 : 0);
        for (int pulse : pulses) std::cout << ',' << pulse;
        std::cout << '\n';
    }
    return 0;
}
