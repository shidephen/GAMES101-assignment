#include "rope.h"

#include <iostream>
#include <vector>

#include "CGL/vector2D.h"
#include "mass.h"
#include "spring.h"

using namespace std;

namespace CGL {

Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass,
           float k, vector<int> pinned_nodes) {
  // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and
  // containing `num_nodes` nodes.

  double length = (end - start).norm();
  Vector2D unit = (end - start).unit();

  Vector2D pos = start;
  Vector2D delta = 1.0 / (num_nodes - 1) * length * unit;

  for (size_t i = 0; i < num_nodes - 1; i++, pos += delta) {
    Mass *mass = new Mass(pos, node_mass, false);
    masses.push_back(mass);
    if (i >= 1) {
      Spring *spring = new Spring(masses[i - 1], mass, k);
      springs.push_back(spring);
    }
  }

  Mass *mass = new Mass(end, node_mass, false);
  Spring *spring = new Spring(*masses.rbegin(), mass, k);
  masses.push_back(mass);
  springs.push_back(spring);

  //        Comment-in this part when you implement the constructor
  for (auto &i : pinned_nodes) {
    masses[i]->pinned = true;
  }
}

void Rope::simulateEuler(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 2): Use Hooke's law to calculate the force on a node
    double l = (s->m1->position - s->m2->position).norm() - s->rest_length;
    
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      // TODO (Part 2): Add the force due to gravity, then compute the new
      // velocity and position

      // TODO (Part 2): Add global damping
    }

    // Reset all forces on each mass
    m->forces = Vector2D(0, 0);
  }
}

void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet
    // （solving constraints)
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      Vector2D temp_position = m->position;
      // TODO (Part 3.1): Set the new position of the rope mass

      // TODO (Part 4): Add global Verlet damping
    }
  }
}

Rope::~Rope() {
  for (auto &m : masses) {
    delete m;
  }

  for (auto &s : springs) {
    delete s;
  }
}

}  // namespace CGL
