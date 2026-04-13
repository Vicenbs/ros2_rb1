#include "ActorMovementSystem.hpp"

using namespace sample_system;

ActorMovementSystem::ActorMovementSystem() {}

ActorMovementSystem::~ActorMovementSystem() {}

void ActorMovementSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {
  static double lastUpdateTime = 0.0;

  if (_info.simTime - lastUpdateTime >= 10.0) {
    // Mueve el actor cada 10 segundos
    // Aquí deberías implementar la lógica para mover el actor
    gzmsg << "Mover el actor cada 10 segundos" << std::endl;

    lastUpdateTime = _info.simTime;
  }
}
