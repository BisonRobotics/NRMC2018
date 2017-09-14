#include <vesc_access/vesc_access.h>
#include <vesc_access/vesc_access_interface.h>
struct PosXy
{
  float x;
  float y;
};

class PositionController
{
public:
  PositionController(float velocity, float tolerance);
  PositionController(float velocity, float tolerance, iVescAccess *fl, iVescAccess *fr, iVescAccess *br,
                     iVescAccess *bl);
  float getVelocity(void);
  float getDistance(void);
  void setDistance(float distance);
  void update(float position_x, float position_y);
  float getTolerance(void);
  void setTolerance(float tolerance);
  bool isMoving(void);

private:
  float velocity;
  float distance;
  PosXy initial_state;
  PosXy current_state;
  float tolerance;
  iVescAccess *fleft_wheel, *fright_wheel, *bright_wheel, *bleft_wheel;
  float tol_sqr;
  bool goal_received;
  bool position_received;
  bool currently_moving;
  bool inTolerance(void);
  void setVelocity(float velocity);
  void setCurrentState(float position_x, float position_y);
  void setInitialState(void);
  void closeGoal(void);
  void stopVescs(void);
  void startVescs(void);
  float getDistanceTravelledSqr(void);
  bool exceededDistance(void);
};
