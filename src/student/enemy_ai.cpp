#include "cpe/simulation.hpp"
namespace cpe {
void Simulation::UpdateEnemy(Enemy& enemy, double dt) {
    // TODO PROJET : perception, transitions, décisions et appel du Navigator.
    // Les ennemis restent immobiles tant que ce module n'est pas implémenté.

    // range enemy 
    
    switch (enemy.state) {
        case State::Waiting:
            enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
            if (enemy.navigator.path.search.status == Status::Found) {
                enemy.state = State::Chase;
            }
            break;
        case State::Recover:
            break;
        case State::Chase:
            enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
            enemy.position = enemy.navigator.Follow(level, enemy.position, enemy.speed, dt);
            if (enemy.TargetIsInRange(player.position)) {
                enemy.state = State::Idle;
            }
            break;
        case State::Idle:
            if (!enemy.TargetIsInRange(player.position)) {
                enemy.state = State::Waiting;
            }
            break;
        default:
            enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
            enemy.position = enemy.navigator.Follow(level, enemy.position, enemy.speed, dt);
            break;

    }

    
    
}
}
