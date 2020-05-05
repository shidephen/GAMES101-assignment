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
    Vector2D a2b = s->m2->position - s->m1->position;
    double l = a2b.norm() - s->rest_length;

    Vector2D f = s->k * a2b.unit() * l; // Fa2b

    s->m1->forces += f;
    s->m2->forces -= f; // Fb2a

    // Damping
    // Vector2D v_a2b = s->m2->velocity - s->m1->velocity;
    // Vector2D damp_b = -damping_factor * dot(a2b.unit(), v_a2b) * a2b.unit();

    // s->m2->forces += damp_b;
    // s->m1->forces -= damp_b;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      // TODO (Part 2): Add the force due to gravity, then compute the new
      // velocity and position
      m->forces += gravity;
      Vector2D acc = m->forces / m->mass;
      Vector2D vt = m->velocity;

      m->last_position = m->position;

      m->velocity = m->velocity + acc * delta_t;

      m->position = m->last_position + m->velocity * delta_t;
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

    Vector2D a2b = s->m2->position - s->m1->position;
    Vector2D unit = a2b.unit();
    double l = a2b.norm() - s->rest_length;

    if (!s->m1->pinned) {
      s->m1->position += 0.5 * l * unit;
    }
    if (!s->m2->pinned) {
      s->m2->position -= 0.5 * l * unit;
    }
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      Vector2D temp_position = m->position;
      // TODO (Part 3.1): Set the new position of the rope mass
      Vector2D acc = gravity / m->mass;

      m->position =
          2 * temp_position - m->last_position + acc * delta_t * delta_t;

      // TODO (Part 4): Add global Verlet damping

      m->position -= damping_factor * (temp_position - m->last_position);

      m->last_position = temp_position;
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

} // namespace CGL
