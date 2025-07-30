#pragma once
#include <iostream>
#include <vector>
#include <cmath>

class Model {

public:

struct Control 
{
  float left;
  float right;

  Control operator+(const Control &ctrl);   
  Control operator-(const Control &ctrl);
  Control operator*(float val);
};

struct State 
{
  float x;
  float y;
  float yaw;

  bool operator==(const State &state) const;
  State operator+(const State &state) const;
  State operator-(const State &state) const;
  State operator*(float val) const;
  float dist(const State &state) const;
  float distXY(const State &state) const;
  const void print() const;
};

public:
  Model(const State &state, float dt);
  void setState(const State &state);
  const State& getState();
  State velocityFromControl(const Control &u) ;
  State nextStateFromVelocity(State &vel) ;
  State nextStateFromControl(const Control &u);

private:

  float k = 0.5f;
  State m_currentState;
  float m_dt; 

};
