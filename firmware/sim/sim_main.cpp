// Симулятор «виртуальной ESP32»: настоящее ядро прошивки + модель робота.
// Говорит тем же протоколом через stdin/stdout, поэтому ПО Raspberry Pi можно
// отлаживать без железа:  python3 -m cyclone run --sim ...
//
//   ./cyclone_sim [--speed 5] [--obstacle x,y,r[,t0,t1]] [--person x,y,секунд] [--stuck-x 3.0] [--vbat 15.2] [--true]
//   --person: препятствие, которое уходит через N секунд после того, как робот к нему подъехал
// Строки, начинающиеся с '#', — служебные (истинное положение), парсер Pi их пропускает.
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "../lib/core/protocol.h"
#include "../lib/core/robot.h"
#include "plant.h"

using namespace cyc;

class StdoutHal : public SimHal {
public:
    using SimHal::SimHal;
    void send(const char* line) override {
        std::fputs(line, stdout);
        std::fflush(stdout);
    }
};

int main(int argc, char** argv) {
    PlantParams pp;
    double speed = 1.0;
    bool printTrue = false;
    std::vector<Obstacle> obs;
    for (int i = 1; i < argc; ++i) {
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (!std::strcmp(argv[i], "--speed")) speed = std::atof(next());
        else if (!std::strcmp(argv[i], "--stuck-x")) pp.stuckX = std::atof(next());
        else if (!std::strcmp(argv[i], "--vbat")) pp.vbat = std::atof(next());
        else if (!std::strcmp(argv[i], "--left-gain")) pp.leftGain = std::atof(next());
        else if (!std::strcmp(argv[i], "--seed")) pp.seed = std::atoi(next());
        else if (!std::strcmp(argv[i], "--true")) printTrue = true;
        else if (!std::strcmp(argv[i], "--obstacle")) {
            Obstacle o{0, 0, 0.15f};
            int n = std::sscanf(next(), "%f,%f,%f,%f,%f", &o.x, &o.y, &o.r, &o.t0, &o.t1);
            if (n >= 2) obs.push_back(o);
        } else if (!std::strcmp(argv[i], "--person")) {
            Obstacle o{0, 0, 0.15f};
            if (std::sscanf(next(), "%f,%f,%f", &o.x, &o.y, &o.hold) == 3) obs.push_back(o);
        } else {
            std::fprintf(stderr, "неизвестный параметр %s\n", argv[i]);
            return 2;
        }
    }
    if (speed <= 0) speed = 1;

    Plant plant(pp);
    plant.obstacles = obs;
    StdoutHal hal(plant);
    Config cfg;
    Robot robot(hal, cfg);
    LineReader reader;
    robot.begin();

    auto t0 = std::chrono::steady_clock::now();
    uint32_t lastTrue = 0;
    for (;;) {
        // входные данные (без блокировки)
        pollfd pfd{STDIN_FILENO, POLLIN, 0};
        while (poll(&pfd, 1, 0) > 0 && (pfd.revents & (POLLIN | POLLHUP))) {
            char buf[256];
            ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
            if (n <= 0) return 0;  // stdin закрыт — выходим
            for (ssize_t k = 0; k < n; ++k)
                if (reader.push(buf[k])) robot.onLine(reader.line());
        }
        double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        double target = wall * speed;
        while (plant.t_ < target) {
            plant.step(0.001f);
            uint32_t ms = hal.millis();
            if (ms % cfg.tickMs == 0) robot.tick();
            if (printTrue && ms - lastTrue >= 200) {
                lastTrue = ms;
                std::printf("#TRUE,%u,%.3f,%.3f,%.1f\n", ms, plant.x, plant.y, plant.th * 57.2958f);
                std::fflush(stdout);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
