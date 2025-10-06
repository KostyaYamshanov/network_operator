#pragma once

#include "model.hpp"
#include "nop.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

class Controller 
{
public:
  Controller(const Model::State &goalState, NetOper &netOper);
  /// RP from pascal version
  virtual Model::Control calcControl(const Model::State &currState);
  /// set new goal state
  void setGoal(Model::State newGoal);

  NetOper& netOper();
  
  void setUMax(float newUMax);

protected:
  Model::State m_goal;
  NetOper m_netOper;
  float Umax = 1.0f; // was 0.4 - OK
};
