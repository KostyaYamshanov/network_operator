#include "model.hpp"

// Model::Control

Model::Control Model::Control::operator+(const Model::Control &ctrl)
{
  return Model::Control{this->left + ctrl.left, this->right + ctrl.right};
}

Model::Control Model::Control::operator-(const Model::Control &ctrl) 
{
  return Model::Control{this->left - ctrl.left, this->right - ctrl.right};
}

Model::Control Model::Control::operator*(float val) 
{
  return Control{this->left * val, this->right * val};
}

// Model::State

bool Model::State::operator==(const Model::State &state) const
{
  return (this->x == state.x) && (this->y == state.y) && (this->yaw == state.yaw);
}

Model::State Model::State::operator+(const Model::State &state) const
{
  return Model::State{this->x + state.x, this->y + state.y, this->yaw + state.yaw};
}

Model::State Model::State::operator-(const Model::State &state) const
{
  const float &yaw1=this->yaw;
  const float &yaw2=state.yaw;
  float dyaw = atan2f(sinf(yaw1 - yaw2), cosf(yaw1 - yaw2));
  return State{this->x - state.x, this->y - state.y, dyaw};
}

Model::State Model::State::operator*(float val) const
{
  return Model::State{this->x * val, this->y * val, this->yaw * val};
}

float Model::State::dist(const Model::State &state) const
{
  float dx = fabs(this->x - state.x);
  float dy = fabs(this->y - state.y);
  float dyaw = fabs(this->yaw - state.yaw);

  return std::sqrt(dx * dx + dy * dy + dyaw * dyaw);
}

float Model::State::distXY(const Model::State &state) const
{
  auto ds = this->operator-(state);
  return std::sqrt(ds.x * ds.x + ds.y * ds.y);
}

const void Model::State::print() const
{
  std::cout << x << " " << y << " " << yaw << "\n";
}

// Model::Model

Model::Model(const Model::State &state, float dt): 
  m_currentState(state),
  m_dt(dt) 
  {}

void Model::setState(const Model::State &state) 
{ 
  m_currentState = state; 
}

const Model::State& Model::getState() 
{ 
  return m_currentState; 
}

Model::State Model::velocityFromControl(const Model::Control &u) 
{
  float k_w = 0.5;
  return  State{k * (u.left + u.right) * cosf(m_currentState.yaw),
                k * (u.left + u.right) * sinf(m_currentState.yaw),
                k_w * k * (u.left - u.right)};
}

Model::State Model::nextStateFromVelocity(Model::State &vel) 
{
  return m_currentState + vel * m_dt;
}

Model::State Model::nextStateFromControl(const Model::Control &u) 
{
  Model::State vel = velocityFromControl(u);
  return nextStateFromVelocity(vel);
}

