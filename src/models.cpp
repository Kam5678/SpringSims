#include "models.h"
#include <iostream>

namespace simulation {

//
// Small angle pendulum
//
SmallAnglePendulumModel::SmallAnglePendulumModel() { reset(); }

void SmallAnglePendulumModel::reset() {
  t = 0.f;
  theta = theta0;
  fixed = Particle(vec3f(0.f, 0.f, 0.f), vec3f(0.f, 0.f, 0.f), 0.f, vec3f(0.f, 0.f, 0.f));
  moving = Particle(vec3f(0.f, -10.f, 0.f), vec3f(0.f, 0.f, 0.f), 1.f, vec3f(0.f, 0.f, 0.f));
}

void SmallAnglePendulumModel::step(float dt) {
  t += dt;
  //theta = smallAnglePendulum(t, theta0, armLength, mass, gravity);

  springForces1D(middle);
  gravityForce(&fixed);
  gravityForce(&moving);

  SemiEuler(&fixed, dt);
  SemiEuler(&moving, dt);



}

float smallAnglePendulum(float t, float theta0, float l, float mass,
                         float graivty) {
  using std::cos;
  using std::sqrt;
  return theta0 * cos(sqrt(graivty / l) * t);
}

vec3f pendulumPosition(float theta, float l) {
  //   /
  //	/
  // o
  using std::cos;
  using std::sin;

  float y = -l * cos(theta);
  float x = l * sin(theta);
  return {x, y, 0.f};
}

//
// Double Pendulum
//
Chain::Chain() { reset(); }

void Chain::reset() {
  particleList.clear();
  springList.clear();
  theta0 = 5.f;
  theta1 = 10.f;
  p0 = 0.f;
  p1 = 0.f;

  // Fixed
  particleList.push_back(Particle(vec3f(0.0f, 0.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));

  for (int i = 1; i <= 10; i++) {
      float iv = (float)i;
      particleList.push_back(Particle(vec3f(iv*5.0f, 0.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 1.f, vec3f(0.0f, 0.0f, 0.0f)));
 
  }
  //createSprings(particleList, 5.0f);
 

  for (int i = 0; i < particleList.size(); i++) {
      for (int j = particleList.size() - 1; i < j; j--) {
          float distance = getMagnitude(&particleList[i], &particleList[j]);
          if (distance <= 5.0f) {
              springList.push_back(Spring(&particleList[i], &particleList[j], 6.0f, 10.0f, 1.0f));


          }
      }
  }

  /*
  for (int a = 0; a < springList.size(); a++) {
 
  }
  */
}


void Chain::step(float dt) {
/* Original Code
  float cosDeltaTheta = std::cos(theta0 - theta1);
  float sinDeltaTheta = std::sin(theta0 - theta1);
  float denom = (m * l * l) * (16.f - 9.f * cosDeltaTheta * cosDeltaTheta);

  // velocities
  float v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
  float v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

  // forces
  float f0 = -(0.5f * m * l * l) *
             (v0 * v1 * sinDeltaTheta + 3.f * (g / l) * std::sin(theta0));
  float f1 = -(0.5f * m * l * l) *
             (-v0 * v1 * sinDeltaTheta + (g / l) * std::sin(theta1));

  // update kinematic/dynamic quantites using Euler integration
  // update momentum
  p0 = p0 + f0 * dt;
  p1 = p1 + f1 * dt;

  // Semi-implicit Euler
  // would use the updated momemnta/velocities for the position updates
  v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
  v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

  // update (angular) positions
  theta0 = theta0 + v0 * dt;
  theta1 = theta1 + v1 * dt;
  **/
    for (int i = 0; i < springList.size(); i++) {
        springForcesnD(springList[i]);
    }

    for (int i = 0; i < particleList.size(); i++) {
        gravityForce(&particleList[i]);
    }

    for (int i = 0; i < particleList.size(); i++) {
        SemiEuler(&particleList[i], dt);
    }

}

vec3f Chain::mass0Position() const {
  return l * vec3f(std::sin(theta0), -std::cos(theta0), 0.f);
}

vec3f Chain::mass1Position() const {
  using std::cos;
  using std::sin;
  return l * vec3f(sin(theta0) + sin(theta1), -cos(theta0) - cos(theta1), 0.f);
}

/*
void DoublePendulumModel::createSprings(std::vector<Particle> particles, float dist){
    //std::vector<Spring> springs;
    for (int i = 0; i < particles.size(); i++) {
        for (int j = particles.size() - 1; i < j; j--) {
            float distance = getMagnitude(&particles[i], &particles[j]);
       
            if (distance <= dist) {
                springList.push_back(Spring(&particles[i], &particles[j], 4.0f, 3.0f, 0.2f));
               

            }
        }
    }
    **/
    /*
    for (int a = 0; a < springs.size(); a++) {
   
    }
    

}
*/

//
// Particle Model
//
JellyCube::JellyCube() { reset(); }

void JellyCube::reset() {
  particleList.clear();
  springList.clear();
  // setup
  /*
  for (int i = -5; i <= 5; ++i) {
      particleList.push_back(Particle({ i, i, 0.f }, { -i, 3.f * i, 0.f }, 1, { 0.f,0.f,0.f }));
  }
  */
  // Base Particles
  /*
  particleList.push_back(Particle(vec3f(0.0f, 0.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));
  for (int i = 1; i <= 6; i++) {
      particleList.push_back(Particle(vec3f(i * 5.0f, 0.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 1.f, vec3f(0.0f, 0.0f, 0.0f)));
  }
  for (int i = 1; i <= 6; i++) {
      particleList.push_back(Particle(vec3f(0.0f, i * 5.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 1.f, vec3f(0.0f, 0.0f, 0.0f)));
  }
  for (int i = 1; i <= 6; i++) {
      particleList.push_back(Particle(vec3f(0.0f, 0.0f, i * 5.0f), vec3f(0.0f, 0.0f, 0.0f), 1.f, vec3f(0.0f, 0.0f, 0.0f)));
  }
  */
  // Fill in Particles
  for (int i = 1; i <= 7; i++) {
      for (int j = 1; j <= 7; j++) {
          for (int k = 1; k <= 7; k++) {
              particleList.push_back(Particle(vec3f(j * 5.0f, k * 5.0f, i * 5.0f), vec3f(-5.0f, -10.0f, -20.0f), 1.f, vec3f(0.0f, 0.0f, 0.0f)));
          }
      }
  }

  for (int i = 0; i < particleList.size(); i++) {
      for (int j = particleList.size() - 1; i < j; j--) {
          float distance = getMagnitude(&particleList[i], &particleList[j]);
         
          if (distance <= 11.0f) {
              // 6.0F 150.0F 2.0F WORKS NICELY FOR PARAMETERS
              springList.push_back(Spring(&particleList[i], &particleList[j], 6.0f, 300.0f, 2.5f));


          }
      }
  }

}

void JellyCube::step(float dt) {
    /*
  for (int iter = 0; iter < 16; ++iter) {
    // do collisions
    for (auto &p : particleList) {
      if (length(p.x) > bounds) {
        auto n = normalize(p.x);
        p.v = glm::reflect(p.v, n);
      }
    }

    // move particles
    for (auto &p : particleList) {
      // forward Euler
      p.x += p.v * dt;
    }
  }
  */

    for (int i = 0; i < springList.size(); i++) {
        springForcesnD(springList[i]);
    }

    for (int i = 0; i < particleList.size(); i++) {
        collisionForce(&particleList[i]);
    }

    for (int i = 0; i < particleList.size(); i++) {
        gravityForce(&particleList[i]);
    }


    for (int i = 0; i < particleList.size(); i++) {
        SemiEuler(&particleList[i], dt);
    }
}


Cloth::Cloth() { reset(); }

void Cloth::reset() {
    particleList.clear();
    springList.clear();

    //particleList.push_back(Particle(vec3f(0.0f, 0.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));
    /*
    particleList.push_back(Particle(vec3f(0.0f, 0.0f, -5.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));
    particleList.push_back(Particle(vec3f(50.0f, 0.0f, -5.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));

    for (int i = 1; i < 10; i++) {
        particleList.push_back(Particle(vec3f(i * 5.0f, 0.0f, -5.0f), vec3f(0.0f, 0.0f, 0.0f), 1.0f, vec3f(0.0f, 0.0f, 0.0f)));
    }
    */
    
    /*
    for (int i = 0; i <= 10; i++) {
        particleList.push_back(Particle(vec3f(55.0f, 0.0f, i*5.0f), vec3f(0.0f, 0.0f, 0.0f), 1.0f, vec3f(0.0f, 0.0f, 0.0f)));
    }
    */

    particleList.push_back(Particle(vec3f(0.0f, 0.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));
    particleList.push_back(Particle(vec3f(0.0f, 0.0f, 50.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));
   // particleList.push_back(Particle(vec3f(-5.0f, 5.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), 0.f, vec3f(0.0f, 0.0f, 0.0f)));
    
    for (int i = 1; i < 10; i++) {
        particleList.push_back(Particle(vec3f(0.0f, 0.0f, i*5.0f), vec3f(0.0f, 0.0f, 0.0f), 1.0f, vec3f(0.0f, 0.0f, 0.0f)));
    }
    

    for (int i = 1; i <=10 ; i++) {
        for (int j = 0; j <= 10; j++) {
            particleList.push_back(Particle(vec3f(i * 5.0f, 0.0f, j*5.0f), vec3f(0.0f, 0.0f, 0.0f), 1.0f, vec3f(0.0f, 0.0f, 0.0f)));
        }
    }
    
    for (int i = 0; i < particleList.size(); i++) {
        for (int j = particleList.size() - 1; i < j; j--) {
            float distance = getMagnitude(&particleList[i], &particleList[j]);

            if (distance <=5.0f) {
                // 6.0f 10.0f 6.0f     8.0f 10.0f 6,0f WORKS
                // // 6.0f, 25.0f, 10.0f    8.0f, 25.0f, 10.0f) WORKS
                //  30.0f , 10.0f -> BEST RESULTS SO FAR
                //springList.push_back(Spring(&particleList[i], &particleList[j], 4.0f, 50.0f, 2.0f));
                // 30
                springList.push_back(Spring(&particleList[i], &particleList[j], 6.0f, 60.0f, 1.0f));

            }
            else if (distance > 5.0f && distance < 10.0f) {
                springList.push_back(Spring(&particleList[i], &particleList[j], 8.0f, 60.0f, 1.0f));
            }
            /*
            else if (distance >= 10.0f && distance < 11.0f) {
                springList.push_back(Spring(&particleList[i], &particleList[j], 11.0f, 4.0f, 2.0f));
            }
            */
            
        }
    }
    
    std::cout << particleList.size() << "\n";
    std::cout << springList.size();
}

void Cloth::step(float dt) {

    for (int i = 0; i < springList.size(); i++) {
        springForcesnD(springList[i]);
    }

    for (int i = 0; i < particleList.size(); i++) {
        gravityForce(&particleList[i]);
    }

    for (int i = 0; i < particleList.size(); i++) {
        collisionForceCloth(&particleList[i]);
    }

    for (int i = 0; i < particleList.size(); i++) {
        SemiEuler(&particleList[i], dt);
    }
}



float getMagnitude(Particle *a, Particle *b) {
  
    float x = a->x.x - b->x.x;
    x *= x;

    float y = a->x.y - b->x.y;
    y *= y;

    float z = a->x.z - b->x.z;
    z *= z;
  
    return sqrt(x + y + z);
}

void springForces1D(Spring s) {

    
    float springLength = getMagnitude(s.a1, s.b1);
 
    
    // Spring Force Calculation
    float calc1 = -1.0f * s.stiffness * (springLength - s.restingLength);
    vec3f calc2 = calc1 * ((s.a1->x - s.b1->x) / springLength);
 

    // Dampining
    //vec3f calc3 = (((s.damping * -1.0f) * (s.a1->v - s.b1->v) * calc2) / (calc2 * calc2)) * calc2;
    vec3f calc3 = (((s.damping * -1.0f) * (s.a1->v - s.b1->v)));
    //vec3f calc3 = (s.damping * -1.0f) * ((glm::dot((s.a1->v - s.b1->v), calc2)) / (glm::dot(calc2, calc2))) * calc2;
 
    // Updateing Net Force for Particles
    s.a1->nF = s.a1->nF + calc2 + calc3;
    s.b1->nF = s.b1->nF - calc2 - calc3;
 
    
}

void springForcesnD(Spring s) {
  

    float springLength = getMagnitude(s.a1, s.b1);
   
    // Spring Force Calculation
    float calc1 = -1.0f * s.stiffness * (springLength - s.restingLength);
    vec3f calc2 = calc1 * ((s.a1->x - s.b1->x) / springLength);
  

    // Dampining
    //vec3f calc3 = (((s.damping * -1.0f) * (s.a1->v - s.b1->v) * calc2) / (calc2 * calc2)) * calc2;
    //vec3f calc3 = (((s.damping * -1.0f) * (s.a1->v - s.b1->v)));
    vec3f calc3 = (s.damping * -1.0f) * ((glm::dot((s.a1->v - s.b1->v), calc2)) / (glm::dot(calc2, calc2))) * calc2;

    // Updateing Net Force for Particles
    s.a1->nF = s.a1->nF + calc2 + calc3;
    s.b1->nF = s.b1->nF - calc2 - calc3;
   

}

void gravityForce(Particle *a) {
    a->nF += vec3f(0.0f,a->m * -9.81f,0.0f);
}

void SemiEuler(Particle *a , float dt) {
    if (a->m > 0) {
        a->v = a->v + ((a->nF / a->m) * dt);
        a->x = a->x + (a->v * dt);
    }

    a->nF = vec3f(0.0f, 0.0f, 0.0f);
}

void collisionForce(Particle *a) {
    if (a->x.y < -19.0f) {
      
        a->x.y = -19.0f;
        vec3f vi = a->v;
        vec3f vf = vec3f(0.0f, 0.0f, 0.0f);
        vec3f deltaV = (vf - vi)/0.01f;
        
        a->nF += deltaV;
        //float height = 
    }
}

void collisionForceCloth(Particle* a) {
    if (a->x.x < 0.0f) {
        //std::cout << "collided" << "\n";
        //a->x.x = 0.0f;
        vec3f vi = a->v;
        vec3f vf = vec3f(0.0f, 0.0f, 0.0f);
        vec3f deltaV = (vf - vi) / 10.0f;

        a->nF += deltaV;
        //float height = 
    }
}







} // namespace simulation
