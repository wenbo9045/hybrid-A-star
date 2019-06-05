#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
  
class Node3D;
class Node2D;

namespace {
void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();
    
  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  template<typename T> 
  bool isTraversable(const T* node) {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
    float cost = 0;
    float x;
    float y;
    float t;
    
    // assign values to the configuration
    getConfiguration(node, x, y, t);
    // 2D collision test
    if (t == 99) {
      return !grid->data[node->getIdx()];
    }

    cost = configurationTest(x, y, t) ? 0 : 1;

    return cost <= 0;
  }

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t);

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

 private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid;
  /// A structure describing the relative position of the occupied cell based on the center of the vehicle
  struct relPos {
  /// the x position relative to the center
  int x;
  /// the y position relative to the center
  int y;
  };
  
  struct config {
  /// the number of cells occupied by this configuration of the vehicle
  int length;
   /*!
     \var relPos pos[64]
     \brief The maximum number of occupied cells
     \todo needs to be dynamic
  */
  relPos pos[64];
  };
  
  /// The collision lookup table
  config collisionLookup[Constants::headings * Constants::positions];
  void collisionLookup_function(config* lookup);
};
}
#endif // COLLISIONDETECTION_H
