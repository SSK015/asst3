#include "quad-tree.h"
#include "world.h"
#include <algorithm>
#include <iostream>

// TASK 1

// NOTE: You may modify any of the contents of this file, but preserve all
// function types and names. You may add new functions if you believe they will
// be helpful.



const int QuadTreeLeafSize = 8;
class SequentialNBodySimulator : public INBodySimulator {
public:
  void getParticlesImpl(std::vector<Particle> &particles, QuadTreeNode *node,
                      Vec2 bmin, Vec2 bmax, Vec2 position, float radius) {
  if (node->isLeaf) {
    for (auto &p : node->particles)
      if ((position - p.position).length() < radius)
        particles.push_back(p);
    return;
  }
  Vec2 pivot = (bmin + bmax) * 0.5f;
  Vec2 size = (bmax - bmin) * 0.5f;
  int containingChild =
      (position.x < pivot.x ? 0 : 1) + ((position.y < pivot.y ? 1 : 0) << 1);
  for (int i = 0; i < 4; i++) {
    Vec2 childBMin;
    childBMin.x = (i & 1) ? pivot.x : bmin.x;
    childBMin.y = ((i >> 1) & 1) ? pivot.y : bmin.y;
    Vec2 childBMax = childBMin + size;
    if (boxPointDistance(childBMin, childBMax, position) <= radius)
      getParticlesImpl(particles, node->children[i].get(), childBMin, childBMax,
                       position, radius);
  }
}
  std::unique_ptr<QuadTreeNode> buildQuadTree(std::vector<Particle> &particles,
                                              Vec2 bmin, Vec2 bmax) {
    // TODO: implement a function that builds and returns a quadtree containing
    // particles.

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
  virtual void simulateStep(AccelerationStructure *accel,
                            std::vector<Particle> &particles,
                            std::vector<Particle> &newParticles,
                            StepParameters params) override {
    // TODO: implement sequential version of quad-tree accelerated n-body
    // simulation here, using quadTree as acceleration structure
//      auto acce = std::make_unique<QuadTree>();

// Vec2 bmin(1e30f, 1e30f);
   // Vec2 bmax(-1e30f, -1e30f);

    //  auto Tree = buildQuadTree(particles, bmin, bmax);
      for (int i = 0; i < (int)particles.size(); i++) {
        
      
      auto pi = particles[i];
      std::vector<Particle> newpar;

// getParticlesImpl(std::vector<Particle> &particles, QuadTreeNode *node,
//                      Vec2 bmin, Vec2 bmax, Vec2 position, float radius) {
      accel->getParticles(newpar, pi.position, params.cullRadius); 
      Vec2 force = Vec2(0.0f, 0.0f);
      // accumulate attractive forces to apply to particle i
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

std::unique_ptr<INBodySimulator> createSequentialNBodySimulator() {
  return std::make_unique<SequentialNBodySimulator>();
}
