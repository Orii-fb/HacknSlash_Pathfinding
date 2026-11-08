#include "cpe/simulation.hpp"
namespace cpe {
void Simulation::UpdateEnemy(Enemy& enemy, double dt) {
    // TODO PROJET : perception, transitions, décisions et appel du Navigator.
    // Les ennemis restent immobiles tant que ce module n'est pas implémenté.
    enemy.position = enemy.navigator.Follow(level, enemy.position, enemy.speed, dt);
}
}
