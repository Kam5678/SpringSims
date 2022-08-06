#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace simulation {

using vec2f = glm::vec2;
using vec3f = glm::vec3;

struct Model {
  virtual ~Model() = default;
  virtual void reset() = 0;
  virtual void step(float dt) = 0;
};

struct Particle {
  explicit Particle(vec3f position) : x(position) {}
  Particle(vec3f position, vec3f velocity, float mass, vec3f net_Force ) : x(position), v(velocity), m(mass) , nF(net_Force)  {}

  vec3f x;
  vec3f v = vec3f{0.f};
  float m = 0;
  vec3f nF = vec3f{0.f};
};

struct Spring {
    Spring(Particle *a , Particle *b , float rL , float s , float d) : a1(a) , b1(b) , restingLength(rL) , stiffness(s) , damping(d){}

    Particle *a1;
    Particle *b1;
    float restingLength;
    float stiffness;
    float damping;

};

//
// Small angle pendulum
//
class SmallAnglePendulumModel : public Model {
public:
  SmallAnglePendulumModel();
  void reset() override;
  void step(float dt) override;

public:
  // constants
  float const gravity = 9.81f;
  float const theta0 = 0.5f;
  float const armLength = 5.f;
  float const mass = 1.f;

  // dependent variables
  float t = 0.f;
  float theta = theta0;
  Particle fixed = Particle(vec3f(0.f, 0.f, 0.f), vec3f(0.f, 0.f, 0.f), 0.f, vec3f(0.f, 0.f, 0.f));
  Particle moving = Particle(vec3f(0.f, -10.f, 0.f), vec3f(0.f, 0.f, 0.f), 1.f, vec3f(0.f, 0.f, 0.f));
  
  Spring middle = Spring(&fixed, &moving, 2.0f, 3.0f, 0.2f);
  
};
// free functions
float smallAnglePendulum(float t, float theta0, float l, float mass,
                         float graivty);

vec3f pendulumPosition(float theta, float l);

float getMagnitude(Particle* a, Particle* b);

void springForces1D(Spring s);

void springForcesnD(Spring s);

void gravityForce(Particle* a);

void SemiEuler(Particle* a, float dt);

void collisionForce(Particle* a);

void collisionForceCloth(Particle* a);


//
// Double Pendulum
//
class Chain : public Model {
public:
  Chain();

  void reset() override;
  void step(float dt) override;

  vec3f mass0Position() const;
  vec3f mass1Position() const;

 //void createSprings(std::vector<Particle> particles, float dist);

public:
  // constants
  float const g = 9.81f;
  float const l = 10.f; // arm lengths
  float const m = 1.f;  // mass

  // dependent variables
  // angle
  float theta0 = 0.f;
  float theta1 = 0.f;

  // momentum
  float p0 = 0.f;
  float p1 = 0.f;

  std::vector<Particle> particleList;
  std::vector<Spring> springList;
};

//
// Particle Model
//
class JellyCube : public Model {
public:
  JellyCube();
  void reset() override;
  void step(float dt) override;

public:
  std::vector<Particle> particleList;
  std::vector<Spring> springList;
  float bounds = 10.f;
};

class Cloth : public Model {
public:
    Cloth();
    void reset() override;
    void step(float dt) override;

public:
    std::vector<Particle> particleList;
    std::vector<Spring> springList;
};



} // namespace simulation
