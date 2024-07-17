#include <iostream>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/wait.h>

int main() {
    // Créer une mémoire partagée
    int* shared_value = static_cast<int*>(mmap(NULL, sizeof(int), PROT_READ | PROT_WRITE, 
                                               MAP_SHARED | MAP_ANONYMOUS, -1, 0));
    *shared_value = 0;

    pid_t pid = fork();

    if (pid == -1) {
        std::cerr << "Échec du fork" << std::endl;
        return 1;
    } else if (pid == 0) {
        // Processus enfant
        std::cout << "Enfant : Valeur partagée initiale : " << *shared_value << std::endl;
        *shared_value = 42;
        std::cout << "Enfant : Valeur partagée mise à jour : " << *shared_value << std::endl;
    } else {
        // Processus parent
        wait(NULL); // Attendre que l'enfant termine
        std::cout << "Parent : Valeur partagée après l'exécution de l'enfant : " << *shared_value << std::endl;
    }

    // Nettoyer la mémoire partagée
    munmap(shared_value, sizeof(int));
    return 0;
}