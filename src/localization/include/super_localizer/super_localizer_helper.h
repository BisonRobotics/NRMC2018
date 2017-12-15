  #include <localizer/localizer.h>

  Localizer::stateVector_s diff(Localizer::stateVector_s lhs, Localizer::stateVector_s rhs);
  Localizer::stateVector_s multiply(Localizer::stateVector_s lhs, Localizer::stateVector_s rhs);
  Localizer::stateVector_s addfrommodel(Localizer::stateVector_s lhs, Localizer::stateVector_s rhs, float dt);