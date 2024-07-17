#include "robot/robot.hpp"
#include "robot/robot_field.hpp"
#include <semaphore.h>
#include <unistd.h>
#include <sys/mman.h>
#include <iostream>
#include <fcntl.h>
#include <memory>

constexpr int MAX_ROBOTS = 100000;

struct SharedMemory {
    RobotState robots[MAX_ROBOTS];
    int32_t message;
};

enum SharedMessage {
    MSG_NONE = 0,
    MSG_STOP = 1
};

namespace {
    sem_t* sem_start_;
    sem_t* sem_done_;
    int shared_mem_fd_;
    char shared_mem_fname_[] = "gro300_shared.XXXXXXXX";
    SharedMemory* shared_memory_;
    std::unique_ptr<RobotField> robot_field_;
    int robot_count_;

    void close_data()
    {
        munmap(shared_memory_, sizeof(SharedMemory));
        close(shared_mem_fd_);
        unlink(shared_mem_fname_);
        sem_close(sem_start_);
        sem_close(sem_done_);
    }

    void init_data()
    {
        shared_mem_fd_ = mkstemp(shared_mem_fname_);
        if (shared_mem_fd_ == -1) {
            std::cerr << "RS: Impossible de créer le fichier temporaire" << std::endl;
            exit(-1);
        }

        if (ftruncate(shared_mem_fd_, sizeof(SharedMemory)) == -1) {
            std::cerr << "RS: Impossible de créer un fichier de la bonne taille" << std::endl;
            close_data();
            exit(-1);
        }

        shared_memory_ = static_cast<SharedMemory*>(mmap(
            NULL,
            sizeof(SharedMemory),
            PROT_READ | PROT_WRITE, MAP_SHARED,
            shared_mem_fd_,
            0));
        if (shared_memory_ == MAP_FAILED) {
            std::cerr << "RS: mmap impossible - terminaison." << std::endl;
            close_data();
            exit(-1);
        }

        std::cerr << "RS: Mémoire partagée initialisée, " << sizeof(SharedMemory) << " octets" << std::endl;
        std::cerr << "RS: Initialisation aléatoire des robots..." << std::endl;

        robot_field_ = std::make_unique<RobotField>(shared_memory_->robots, robot_count_);
        robot_field_->initRandom();

        std::cerr << "Initialisation terminée, retour de l'adresse de la mémoire partagée..." << std::endl;

        // Retourne au script Python le nom du fichier représentant la mémoire partagée
        std::cout << shared_mem_fname_ << std::endl;
        // Debug
        std::cerr << "RS: Mémoire sur fichier temporaire: " << shared_mem_fname_ << std::endl;
    }

    void simulate_step()
    {
        robot_field_->runCycle();
    }
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "Utilisation: " << argv[0] << " semstart_name semdone_name robot_count" << std::endl;
        return -1;
    }

    const char* sem_start_name = argv[1];
    const char* sem_done_name = argv[2];
    robot_count_ = std::atoi(argv[3]);

    if (robot_count_ > MAX_ROBOTS) {
        std::cerr << "Le nombre de robots ne peut pas dépasser " << MAX_ROBOTS << std::endl;
        return -1;
    }

    sem_start_= sem_open(sem_start_name, 0);
    sem_done_= sem_open(sem_done_name, 0);
    if (sem_start_ == SEM_FAILED || sem_done_ == SEM_FAILED) {
        std::cerr << "RS: Impossible d'accéder au sémaphore" << std::endl;
        return -1;
    }

    std::cerr << "RS: Initialisation de la simulation avec " << robot_count_ << " robots" << std::endl;

    init_data();

    std::cerr << "RS: Démarrage de la simulation..." << std::endl;
    while (true)
    {
        sem_wait(sem_start_);
        if (shared_memory_->message == MSG_STOP) {
            std::cerr << "RS: Arrêt de la simulation demandé" << std::endl;
            robot_field_->request_stop();
            break;
        }

        simulate_step();
        sem_post(sem_done_);
    }

    close_data();
    std::cerr << "RS: Fin de la simulation" << std::endl;
    return 0;
}