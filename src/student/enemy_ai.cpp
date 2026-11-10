#include "cpe/simulation.hpp"
namespace cpe {
void Simulation::UpdateEnemy(Enemy& enemy, double dt) {
    // TODO PROJET : perception, transitions, décisions et appel du Navigator.
    // Les ennemis restent immobiles tant que ce module n'est pas implémenté.

    // range enemy 
    
    switch (enemy.state) {
        case State::Waiting:
            enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
            bool Finished = enemy.TargetIsInRange(player.position);
            break;
        case State::Recover:
            break;
        case State::Chase:
            break;
        case State::Idle:
            break;

    }

    
    enemy.position = enemy.navigator.Follow(level, enemy.position, enemy.speed, dt);
}
}
