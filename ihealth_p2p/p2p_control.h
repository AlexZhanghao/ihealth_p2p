// #ifdef  P2PCONTROL_H
// #define P2PCONTROL_H
#include"control_card.h"
#include "FTWrapper.h"
#include <vector>

struct p2pdata {
	std::vector<double> target_position[2];
};

class P2PControl{
public:
    P2PControl();
    ~P2PControl();

    void StartMove();
    void StopMove();
	//void SixDimForceStep();
	void P2PMovestep();

public:
	FTWrapper *mFTWrapper;

	bool is_exit_thread_;
	double cycle_time_in_second_;
	bool is_stop;

private:
	void MoveInNewThread();
	//void ExitMoveThread();

private:
	p2pdata *mp2pdata;
	HANDLE move_thread_handle_;
	bool is_moving_;
	int step_num_;
};

//#endif