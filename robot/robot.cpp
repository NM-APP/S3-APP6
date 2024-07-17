#include "robot.hpp"
#include <cmath>

void RobotState::applyControl(double ts)
{
    // Mettre à jour les vitesses basées sur les entrées de contrôle
    dx += ux;
    dt += ut;

    // Mettre à jour la position et l'orientation
    x += std::cos(t) * dx * ts;
    y += std::sin(t) * dx * ts;
    t += dt * ts;

    // Appliquer une friction simple pour réduire progressivement les vitesses
    constexpr double friction = 0.99;
    dx *= friction;
    dt *= friction;

    // Réinitialiser les entrées de contrôle
    ux = 0.0;
    ut = 0.0;
}