// pybind11_wrapper.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "../inc/Controller.h"
#include "../inc/Simulator.h"

#include <pybind11/stl.h> // to use std::vector

namespace py = pybind11;

// python class for controller
class PyController : public C {
        public:
                using C::C;
                void run() {
                        PYBIND11_OVERRIDE_PURE(
                                        void,
                                        C,
                                        run,);
                }
};

// python class for simulator
class PySimulator : public S {
        public:
                using S::S;
                void run() {
                        PYBIND11_OVERRIDE_PURE(
                                        void,
                                        S,
                                        run,);
                }
};

PYBIND11_MODULE(hopper, m) {

  ////////////////////////////////// CONTROLLER Class //////////////////////////////////
  py::class_<C, PyController> c(m, "C");
  c
          .def(py::init<>())
          .def("run",&C::run);

 py::enum_<ProgramState>(m, "ProgramState")
	  .value("RUNNING", RUNNING)
	  .value("STOPPED", STOPPED)
	  .value("RESET", RESET)
	  .value("KILL", KILL)
	  .export_values();

  py::class_<Controller> controller(m, "Controller", c);
  controller
  .def(py::init<>()) // constructor
  .def_readwrite("t_last", &Controller::t_last)
  .def_readwrite("t_last_MPC", &Controller::t_last_MPC)
  .def_readwrite("duration", &Controller::duration)
  .def_readwrite("x", &Controller::x)
  .def_readwrite("q", &Controller::q)
  .def_readwrite("v", &Controller::v)
  .def_readwrite("initialCondition", &Controller::initialCondition_)
  .def_readwrite("goalState", &Controller::goalState_)
  .def_readwrite("objVal", &Controller::objVal)
  .def_readwrite("num_hops", &Controller::num_hops)
  .def_readwrite("programState", &Controller::programState_)
  .def_readwrite("stateSequence", &Controller::stateSequence_)
  .def_readwrite("paramsSequence", &Controller::paramsSequence_)
  .def_readwrite("sim_type", &Controller::sim_type_)
  .def_readwrite("max_hops", &Controller::max_hops_)
  .def_property_readonly("TX_torques", [](py::object&obj) {
        Controller& o = obj.cast<Controller&>();
        return py::array{26,o.TX_torques,obj};})
  .def("startSimulation", &Controller::startSimulation)
  .def("stopSimulation", &Controller::stopSimulation)
  .def("killSimulation", &Controller::killSimulation)
  .def("resetSimulation", py::overload_cast<vector_t>(&Controller::resetSimulation))
  .def("setInitialState", py::overload_cast<vector_t>(&Controller::setInitialState))
  .def("setGoalState", py::overload_cast<vector_t>(&Controller::setGoalState))
  .def("setStateSequence", py::overload_cast<vector_array_t,scalar_array_t>(&Controller::setStateSequence))
  .def("setSimType", py::overload_cast<char>(&Controller::setSimType))
  .def("setMaxHops", py::overload_cast<int>(&Controller::setMaxHops))
 ;

  m.def("call_run", [](C *c) -> void {
                  py::gil_scoped_release release;
                  call_run(c);
                  py::gil_scoped_acquire acquire;
                  });

  ////////////////////////////////// SIMULATOR Class //////////////////////////////////
  py::class_<S, PySimulator> s(m, "S");
  s
          .def(py::init<>())
          .def("run",&S::run);
  
  py::class_<Simulator> simulator(m, "Simulator",s);
  simulator 
      .def(py::init<>()) // constructor
      .def("killSimulation",&Simulator::killSimulation)
      .def("setVisualization",py::overload_cast<bool>(&Simulator::setVisualization))
      .def_readwrite("visualize", &Simulator::visualize_)
      .def_readwrite("kill", &Simulator::kill);

  m.def("call_run_sim", [](S *s) -> void {
                  py::gil_scoped_release release;
                  call_run_sim(s);
                  py::gil_scoped_acquire acquire;
                  });
}

