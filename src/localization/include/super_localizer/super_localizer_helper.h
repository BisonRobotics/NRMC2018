#include <localizer/localizer.h>
namespace LocalizerInterface
{
LocalizerInterface::stateVector diff(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs);
LocalizerInterface::stateVector multiply(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs);
LocalizerInterface::stateVector addFromModel(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs,
                                             float dt, bool imu);
LocalizerInterface::stateVector initState(float xi, float yi, float theta);
}
