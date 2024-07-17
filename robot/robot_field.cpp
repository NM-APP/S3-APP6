#include "robot_field.hpp"
#include "robot_field.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <future>

class Xorshift32 {
public:
    explicit Xorshift32(uint32_t seed) : state(seed) {}

    uint32_t next() {
        uint32_t x = state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        state = x;
        return x;
    }

    float nextFloat() {
        return static_cast<float>(next()) / static_cast<float>(UINT32_MAX);
    }

private:
    uint32_t state;
};

RobotField::RobotField(RobotState* robots_ptr, size_t robot_count):
    robots_(robots_ptr, robot_count)
{
    unsigned int thread_count = std::thread::hardware_concurrency();

    thread_pool.reserve(thread_count);
    for (unsigned int i = 0; i < thread_count; i++)
    {
        thread_pool.emplace_back(std::jthread([this, i, thread_count]()
        {
            do
            {
                Xorshift32 random(rand());

                semaphore.acquire();

                size_t start = (i * robots_.size()) / thread_count;
                size_t end = ((i + 1) * robots_.size()) / thread_count;

                for (size_t j = start; j < end; ++j) {

                    RobotState& robot = robots_[j];

                    static constexpr double time_step = 0.01; // 10 ms

                    // Appliquer le contrôle et mettre à jour l'état
                    robot.applyControl(time_step);

                    // S'assurer que le robot reste dans le champ (supposé 1x1)
                    robot.x = std::clamp(robot.x, 0.0, 1.0);
                    robot.y = std::clamp(robot.y, 0.0, 1.0);

                    // Limiter l'angle à [-π, π]
                    robot.t = std::atan2(std::sin(robot.t), std::cos(robot.t));

                    // Commandes initiales aléatoires.
                    robot.ux += random.nextFloat() * 0.01f; // Valeur aléatoire entre 0 et 0.01
                    robot.ut += random.nextFloat() * 1.00f - 0.50f; // Valeur aléatoire entre -0.5 et 0.5
                }

                if(++completed_threads == thread_count)
                {
                    signal_semaphore.notify_one();
                }

            } while (running);
        }));
    }
}

void RobotField::request_stop()
{
    running = false;
    signal_semaphore.notify_all();
}

void RobotField::initRandom()
{
    int c = 0;
    for (auto& robot: robots_)
    {
        robot.x = 0.01+static_cast<double>(rand()) / RAND_MAX;
        robot.y = 0.01+static_cast<double>(rand()) / RAND_MAX;
        robot.t = static_cast<double>(rand()) / RAND_MAX * 2 * M_PI - M_PI;
        
        robot.dx = 0;
        robot.dt = 0;
        robot.ux = 0;
        robot.ut = 0;

        c++;
        
        // Pour déboguage:
        // std::cerr << "RS: Robot " << c << " initialisé à ("
        //           << robot.x << ", " << robot.y << ", " << robot.t << ")" << std::endl;
    }
}

void RobotField::runCycle()
{
    unsigned int thread_count = std::thread::hardware_concurrency();

    completed_threads = 0;
    semaphore.release(thread_count);

    std::unique_lock lock(signal_mutex);
    signal_semaphore.wait(lock, [this, thread_count]()
        {
            return completed_threads == thread_count;
        });
}