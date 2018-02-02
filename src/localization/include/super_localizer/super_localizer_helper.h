#include <localizer/localizer.h>
namespace LocalizerInterface
{
LocalizerInterface::stateVector diff(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs);
LocalizerInterface::stateVector multiply(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs);
LocalizerInterface::stateVector addFromModel(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs,
                                             double dt, bool imu);
LocalizerInterface::stateVector initState(double xi, double yi, double theta);
}
