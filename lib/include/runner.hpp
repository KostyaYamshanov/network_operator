// #include <onnxruntime_cxx_api.h>
#include "model.hpp"
#include "controller.hpp"

class Runner {

    public:
        Runner(Model& model, Controller& controller);
        void setGoal(const Model::State& goal);
        void init(const Model::State& goal);
        Model::State makeStep(); 

    private:
        void Euler2();

    private:
        Model &m_model;
        Controller &m_controller;  
};
