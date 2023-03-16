#include "quad-tree.h"
#include "world.h"
#include <algorithm>
#include <iostream>

// TASK 2

// NOTE: You may modify this class definition as you see fit, as long as the
// class name, and type of simulateStep and buildAccelerationStructure remain
// the same. You may modify any code outside this class unless otherwise
// specified.

const int QuadTreeLeafSize = 8;
class ParallelNBodySimulator : public INBodySimulator {
public:
  // TODO: implement a function that builds and returns a quadtree containing
  // particles. You do not have to preserve this function type.
  std::unique_ptr<QuadTreeNode> buildQuadTree(std::vector<Particle> &particles,
                                              Vec2 bmin, Vec2 bmax) {
    if ((int)particles.size() <= QuadTreeLeafSize) {
//        auto Node = new QuadTreeNode;
        std::unique_ptr<QuadTreeNode> Node(new QuadTreeNode); 
 
        Node->isLeaf = 1;
        Node->particles = particles;
        return Node;
    } else {
      std::unique_ptr<QuadTreeNode> result(new QuadTreeNode);
//      auto result = new QuadTreeLeafSize;
      result->isLeaf = 0;
      result->particles = particles;
      auto pivot = (bmin + bmax) * 0.5f;
      Vec2 size = (bmax - bmin) * 0.5f;
    #pragma omp parallel for
    for (int i = 0; i < 4; i++) {
      Vec2 childBMin;
      childBMin.x = (i & 1) ? pivot.x : bmin.x;
      childBMin.y = ((i >> 1) & 1) ? pivot.y : bmin.y;
      Vec2 childBMax = childBMin + size;
      std::vector<Particle> newpar;
      
      for (int i = 0; i < (int)particles.size(); i++) {
        
        auto pi = particles[i];
      
        if (pi.position.x < childBMax.x && pi.position.y < childBMax.y && pi.position.x >= childBMin.x && pi.position.y >= childBMin.y) {
          newpar.push_back(pi);
        }
      }
      result->children[i] = buildQuadTree(newpar, childBMin, childBMax);
  }
      // <assign particles to appropriate children of result>
      
      return result;
    }
    return nullptr;
  }

  // Do not modify this function type.
  virtual std::unique_ptr<AccelerationStructure>
  buildAccelerationStructure(std::vector<Particle> &particles) {
    // build quad-tree
    auto quadTree = std::make_unique<QuadTree>();

    // find bounds
    Vec2 bmin(1e30f, 1e30f);
    Vec2 bmax(-1e30f, -1e30f);

    for (auto &p : particles) {
      bmin.x = fminf(bmin.x, p.position.x);
      bmin.y = fminf(bmin.y, p.position.y);
      bmax.x = fmaxf(bmax.x, p.position.x);
      bmax.y = fmaxf(bmax.y, p.position.y);
    }

    quadTree->bmin = bmin;
    quadTree->bmax = bmax;

    // build nodes
    quadTree->root = buildQuadTree(particles, bmin, bmax);
    if (!quadTree->checkTree()) {
      std::cout << "Your Tree has Error!" << std::endl;
    }

    return quadTree;
  }

  // Do not modify this function type.
  virtual void simulateStep(AccelerationStructure *accel,
                            std::vector<Particle> &particles,
                            std::vector<Particle> &newParticles,
                            StepParameters params) override {
    // TODO: implement parallel version of quad-tree accelerated n-body
  
    // simulation here, using quadTree as acceleration structure
    #pragma omp parallel for
    for (int i = 0; i < (int)particles.size(); i++) {
        
      
      auto pi = particles[i];
      std::vector<Particle> newpar;

// getParticlesImpl(std::vector<Particle> &particles, QuadTreeNode *node,
//                      Vec2 bmin, Vec2 bmax, Vec2 position, float radius) {
      //#pragma omp for schedule(dynamic)
      accel->getParticles(newpar, pi.position, params.cullRadius); 
      
      Vec2 force = Vec2(0.0f, 0.0f);
      // accumulate attractive forces to apply to particle i
//      #pragma omp parallel for
//      #pragma omp for schedule(dynamic)
      for (size_t j = 0; j < newpar.size(); j++) {
      //  if (j == i)
      //    continue;
          force += computeForce(pi, newpar[j], params.cullRadius);
      }
      // update particle state using the computed force
      newParticles[i] = updateParticle(pi, force, params.deltaTime);
    }
  }
};

// Do not modify this function type.
std::unique_ptr<INBodySimulator> createParallelNBodySimulator() {
  return std::make_unique<ParallelNBodySimulator>();
}

