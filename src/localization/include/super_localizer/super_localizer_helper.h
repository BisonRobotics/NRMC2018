#include <localizer/localizer.h>

LocalizerInterface::stateVector diff(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs);
LocalizerInterface::stateVector multiply(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs);
LocalizerInterface::stateVector addfrommodel(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs,
                                             float dt);