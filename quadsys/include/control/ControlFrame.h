#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"

class ControlFrame
{
public:
    ControlFrame(CtrlComponents *ctrlComp);
    ~ControlFrame()
    {
        delete _FSMController;
    }
    void run();

    FSM *_FSMController;
    CtrlComponents *_ctrlComp;

private:
    
};

#endif // CONTROLFRAME_H