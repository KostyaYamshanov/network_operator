#include "runner.hpp"


Runner::Runner(Model& model, Controller& controller): 
    m_model(model),
    m_controller(controller) 
    { }

void Runner::setGoal(const Model::State &goal)
{
    m_controller.setGoal(goal);
}

void Runner::init(const Model::State& state)
{
    m_model.setState(state);
    m_model.setVelocity(0.0, 0.0);
}

// TODO MB ERROR
// void Runner::Euler2() 
// {
//     constexpr float K = 0.5;

//     Model::State initialState = m_model.getState();

//     Model::Control u1 = m_controller.calcControl(initialState);
//     Model::State v1 = // m_model.velocityFromControl(u1); // fa
//     Model::State s1 = m_model.nextNNStateFromControl(u1); // m_model.nextStateFromVelocity(v1); // moved state

//     Model::Control u2 = m_controller.calcControl(s1);
//     m_model.setState(s1);
//     Model::State v2 = m_model.velocityFromControl(u2); // fb
//     m_model.setState(initialState);
//     Model::State ctrl = (v1 + v2) * K;
//     // TODO TEST ONLY
//     Model::State nextState = m_model.nextNNStateFromControl(ctrl); // m_model.nextStateFromVelocity(ctrl);
//     // Model::State nextState = m_model.nextStateFromControl(ctrl);

//     m_model.setState(nextState);
// }

void Runner::Euler2() 
{
    constexpr float K = 0.5f;

    Model::State initialState = m_model.getState();

    // 1. Считаем первый контроль
    Model::Control u1 = m_controller.calcControl(initialState);

    // 2. NN предсказывает новое состояние (s1) при u1
    Model::State s1 = m_model.nextNNStateFromControl(u1);

    // // 3. Считаем второй контроль в промежуточной точке
    // Model::Control u2 = m_controller.calcControl(s1);

    // // 4. NN предсказывает новое состояние при u2
    // Model::State s2 = m_model.nextNNStateFromControl(u2);

    // // 5. Усредняем (как RK2 / Heun)
    // auto avg = (s1 + s2);
    // avg = avg * K;
    // auto next_state = m_model.nextStateFromVelocity(avg);
    auto next_state = s1;
    m_model.setState(next_state);
}


Model::State Runner::makeStep() 
{
    Euler2();
    // std::cout<<m_model.m_v<<"  "<<m_model.m_w<<std::endl;
    return m_model.getState(); 
}
