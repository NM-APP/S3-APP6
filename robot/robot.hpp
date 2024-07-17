#ifndef ROBOT_HPP
#define ROBOT_HPP

struct RobotState
{
public:
    double x;   // Position en X (repère global)
    double y;   // Position en Y (repère global)
    double t;   // Angle theta (autour de Z, repère global)
    double dx;  // Vitesse en X (repère local)
    double dt;  // Vitesse angulaire (repère local)
    double ux;  // Entrée de contrôle pour la vitesse linéaire
    double ut;  // Entrée de contrôle pour la vitesse angulaire

    /// @brief Applique les entrées de contrôle actuelles et met à jour l'état du robot
    /// @param ts Durée du pas de temps
    void applyControl(double ts);
};

#endif