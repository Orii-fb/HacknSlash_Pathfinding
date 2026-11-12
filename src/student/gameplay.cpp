#include "cpe/simulation.hpp"
#include <iostream>
namespace cpe {
void Simulation::UpdateGameplay(bool actionPressed, double dt) {
    // TODO PROJET : attaques, dégâts, vie, mort, vagues, victoire et défaite.
    // Espace alimente actionPressed ; aucun effet de jeu n'est fourni.
    if (player.hp <= 0) return; // state de la game ?

    player.attackCooldown = std::max(0.0, player.attackCooldown - dt);

    if (!actionPressed || player.attackCooldown > 0) return;

    for (Enemy& enemy : enemies) {
        if (enemy.hp <= 0 || enemy.state == State::Dead) {
            continue;
        }

        if (Distance(player.position, enemy.position) - level.agentRadius <= combat::PlayerAttackRange &&
            level.Visible(player.position, enemy.position)) {
            enemy.hp -= combat::PlayerAttackDamage;
            if (enemy.hp <= 0) {
                std::cout << "an Enemy is dead";
                enemy.state = State::Dead;
                enemy.isAttacking = false;
            }
        }
    }
    
}
}
