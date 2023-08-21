// pybind11_wrapper.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "../inc/Controller.h"

namespace py = pybind11;

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

PYBIND11_MODULE(hopper, m) {
  py::class_<C, PyController> c(m, "C");
  c
          .def(py::init<>())
          .def("run",&C::run);

 py::enum_<ProgramState>(m, "ProgramState")
	  .value("RUNNING", RUNNING)
	  .value("STOPPED", STOPPED)
	  .value("RESET", RESET)
	  .export_values();

  py::class_<Controller> controller(m, "Controller", c);
  controller
  .def(py::init<>()) // constructor
  .def_readwrite("t_last", &Controller::t_last)
  .def_readwrite("t_last_MPC", &Controller::t_last_MPC)
  .def_readwrite("duration", &Controller::duration)
  .def_readwrite("state", &Controller::state)
.def_readwrite("programState_", &Controller::programState_)
  .def_property_readonly("TX_torques", [](py::object&obj) {
        Controller& o = obj.cast<Controller&>();
        return py::array{26,o.TX_torques,obj};})
  .def("startSimulation", &Controller::startSimulation)
  .def("stopSimulation", &Controller::stopSimulation)
  .def("resetSimulation", py::overload_cast<vector_t>(&Controller::resetSimulation))
 ;

  m.def("call_run", [](C *c) -> void {
                  py::gil_scoped_release release;
                  call_run(c);
                  py::gil_scoped_acquire acquire;
                  });
}

