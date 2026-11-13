#include "cpe/simulation.hpp"
#include <iostream>
namespace cpe {
void Simulation::UpdateEnemy(Enemy& enemy, double dt) {
    // TODO PROJET : perception, transitions, décisions et appel du Navigator.
    // Les ennemis restent immobiles tant que ce module n'est pas implémenté.

    // range enemy 

    if (enemy.state == State::Dead) return;
    
    switch (enemy.state) {
        case State::Waiting:
            enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
            if (enemy.navigator.path.search.status == Status::Found && enemy.isInRangeDetection(player.position)) {
                enemy.state = State::Chase;
            }
            break;
        case State::Recover:
            // changer le state de l'enemy à recover quand il se fait tapper
            enemy.cooldown += dt;
            if (enemy.cooldown >= enemy.recovery) {
                enemy.cooldown = 0;
                enemy.state = State::Idle;
            }
            break;
        case State::Chase:
            enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
            enemy.position = enemy.navigator.Follow(level, enemy.position, enemy.speed, dt);
            if (!enemy.isInRangeDetection(player.position)) {
                enemy.state = State::Waiting;
            }
            if (enemy.TargetIsInRange(player.position)) {
                enemy.state = State::Windup;
            }
            break;
        case State::Idle:
            if (!enemy.TargetIsInRange(player.position)) {
                enemy.state = State::Waiting;
            }
            break;
        case State::Windup:
            if (!enemy.isAttacking) {
                enemy.isAttacking = true;
            }

            if (enemy.cooldown < enemy.attackCooldown) {
                enemy.cooldown += dt;
            }
            else {
                // TODO : a mettre la fonction d'attaque
                if (Distance(enemy.position, player.position) - level.agentRadius <= enemy.attackRange && level.Visible(enemy.position, player.position) && player.hp > 0) {
                    std::cout << "Player was attack";
                    player.hp -= enemy.attack;
                }

                if (enemy.TargetIsInRange(player.position)) {
                    enemy.cooldown = 0;
                }
                else {
                    enemy.isAttacking = false;
                    enemy.cooldown = 0;
                    enemy.state = State::Waiting;
                }
                
            }
            break;
        default:
            enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
            enemy.position = enemy.navigator.Follow(level, enemy.position, enemy.speed, dt);
            break;

    }

    
    
}
}
