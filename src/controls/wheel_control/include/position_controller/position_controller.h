#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
struct PosXy
{
  float x;
  float y;
};

class PositionController
{
public:
  PositionController(float velocity);
  PositionController(float velocity, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl);
  float getVelocity(void);
  float getDistance(void);
  float getDistanceRemaining(void);
  void setDistance(float distance);
  void update(float position_x, float position_y);
  bool isMoving(void);
  ~PositionController();
  void closeGoal(void);
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;

private:
  float velocity;
  float distance;
  float distance_square;
  PosXy initial_state;
  PosXy current_state;
  bool goal_received;
  bool position_received;
  bool currently_moving;
  void setVelocity(float velocity);
  void setCurrentState(float position_x, float position_y);
  void setInitialState(void);
  //  void closeGoal(void);
  void stopVescs(void);
  void startVescs(void);
  float getDistanceTravelledSqr(void);
  bool exceededDistance(void);
  bool internally_alloc;
  void initializeMembers(float velocity);
};
