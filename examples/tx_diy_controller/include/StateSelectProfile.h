#pragma once

#include "StateBase.h"

//=====================================================================
//=====================================================================
class StateSelectProfile: public StateBase
{
private:
    uint32_t stateTime;
    bool waitUnpress;
    bool exit;
    uint8_t profileIndex;

public:

    static StateSelectProfile instance;

    virtual void onEnter() override;
    virtual void onRun(uint32_t t) override;

};
