#include "controller.hpp"

Controller::Controller(const Model::State &goalState, NetOper &netOper):
	m_goal(goalState),
	m_netOper(netOper)
	{ }

Model::Control Controller::calcControl(const Model::State& currState)
{	
	Model::State delta = m_goal - currState; 
	std::vector<float> u(2, 0);
	m_netOper.calcResult({delta.x, delta.y, delta.yaw}, u);
  	u[0] = std::min(std::max(u[0], -Umax), Umax);
  	u[1] = std::min(std::max(u[1], -Umax), Umax);

	return Model::Control{u[0], u[1]};
}

void Controller::setGoal(Model::State newGoal)
{
	if(m_goal == newGoal) return;
	m_goal = newGoal;
}

NetOper& Controller::netOper()
{
	return m_netOper;
}

void Controller::setUMax(float newUMax)
{
	if(newUMax == Umax) return;
	Umax = newUMax;
}