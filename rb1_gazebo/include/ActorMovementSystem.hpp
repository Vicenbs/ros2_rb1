#ifndef ACTOR_MOVEMENT_SYSTEM_HPP
#define ACTOR_MOVEMENT_SYSTEM_HPP

#include <gz/sim/System.hh>

namespace sample_system {
class ActorMovementSystem : public gz::sim::System,
                            public gz::sim::ISystemPreUpdate {
public:
  ActorMovementSystem();
  ~ActorMovementSystem() override;

  // Implementa el método PreUpdate de la interfaz ISystemPreUpdate
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;
};
} // namespace sample_system

#endif // ACTOR_MOVEMENT_SYSTEM_HPP
