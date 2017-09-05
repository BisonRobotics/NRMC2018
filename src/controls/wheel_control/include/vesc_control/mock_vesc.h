#ifndef __MOCK_VESC_H_
#define __MOCK_VESC_H_

#include <vesc_control/vesc_interface.h>
#include <gmock/gmock.h>

class mockVesc : public iVesc {
  public:
//    MOCK_METHOD0 (~mockVesc);
    MOCK_METHOD1 (setRpm, void (float rpm));
    MOCK_METHOD1 (setCurrent, void (float current));
};

#endif
