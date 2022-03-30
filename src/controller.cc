#include "controller.h"

bool ControllerOptions::Check() const { return true; }

Controller::Controller(const ControllerOptions* options) : options_(options) {}