#include <vesc_access/vesc_access.h>


class PositionController{
  public:
    PositionController (float velocity, float tolerance);
    float getVelocity (void);
    float getDistance (void);
    void setDistance (float distance);
    void update (float position);
    float getTolerance (void);
    void setTolerance (float tolerance);
    bool isMoving (void);
  private:
    float velocity;
    float distance;
    float position;
    float tolerance;
    float goal;
    bool goal_received;
    bool position_received;
    bool inTolerance (void);
    void setVelocity (float velocity);    
    void setPosition (float position); 
};
