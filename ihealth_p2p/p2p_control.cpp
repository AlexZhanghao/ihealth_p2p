#include"p2p_control.h"
#include <windows.h>
#include <process.h> 
#include<iostream>
#include<fstream>
#include<ctime>
#include <spdlog.h>
#include <sinks/basic_file_sink.h>
#include "data_acquisition.h"

using namespace std;

P2PControl::P2PControl() {
	mFTWrapper = new FTWrapper();
	is_moving_ = false;
	is_exit_thread_ = false;
	step_num_ = 0;
	is_stop = false;

	for (int i = 0; i < 10; ++i) {
		for (int j = 0; j < 10; ++j) {
			mp2pdata->target_position[0].push_back((i + 1)*4);
			mp2pdata->target_position[1].push_back((j + 1)*4);
		}
	}
}

P2PControl::~P2PControl() {
	if (mFTWrapper != NULL) {
		delete mFTWrapper;
	}
}


void P2PControl::StartMove() {
	 mFTWrapper->LoadCalFile();
	 mFTWrapper->BiasCurrentLoad(true);
	 mFTWrapper->setFUnit();
	 mFTWrapper->setTUnit();

	 ControlCard::GetInstance().SetMotor(ControlCard::MotorOn);
	 ControlCard::GetInstance().SetClutch(ControlCard::ClutchOn);
	 is_moving_ = true;
	 MoveInNewThread();
}

void P2PControl::StopMove() {
	ControlCard::GetInstance().SetMotor(ControlCard::MotorOff);
	is_moving_ = false;
	//ExitMoveThread();
}

unsigned int __stdcall P2PMoveThread(PVOID pParam) {
	P2PControl* p2p = (P2PControl*)pParam;
	UINT start, end;
	start = GetTickCount();

	std::string pathname = "..\\..\\resource\\ExportData\\";
	time_t t = time(0);
	char ch[64];
	strftime(ch, sizeof(ch), "%Y-%m-%d %H-%M-%S", localtime(&t)); //年-月-日 时-分-秒
	string paitent_info = "p2p_";
	ofstream joint_value(pathname + paitent_info + "joint_" + ch + "(只有六维力模式)" + ".txt", ios::app | ios::out);
	ofstream torque_value(pathname + paitent_info + "torque_" + ch + "(只有六维力模式)" + ".txt", ios::app | ios::out);
	ofstream sixdim_force_value(pathname + paitent_info + "sixdim_force_" + ch + "(只有六维力模式)" + ".txt", ios::app | ios::out);

	joint_value << " shoulder horizontal flexion/extension  " << " shoulder adduction/abduction  " << " shoulder flexion/extension " << " elbow flexion/extension"
		<< " forearm pronation/supination " << endl;
	torque_value << " shoulder(N.m)  " << "  elbow(N.m)  " << endl;
	sixdim_force_value << " fx(N) " << " fy(N) " << " fz(N) " << " tx(N.m) " << " ty(N.m) " << " tz(N.m) " << endl;

	double angle[2]{ 0 };
	double torque[2]{ 0 };
	double six_dim_force[7]{ 0 };

	while (true) {
		if (p2p->is_exit_thread_) {
			break;
		}

		// 每隔一定时间进行一次循环，这个循环时间应该是可调的。
		while (true) {
			end = GetTickCount();
			if (end - start >= p2p->cycle_time_in_second_ * 1000) {
				start = end;
				break;
			}
			else {
				SwitchToThread();
			}
		}
		//六维力线程
		p2p->P2PMovestep();
		ControlCard::GetInstance().GetEncoderData(angle);
		p2p->mFTWrapper->GetForcesAndTorques(six_dim_force);
		torque[0] = DataAcquisition::GetInstance().ShoulderTorque();
		torque[1] = DataAcquisition::GetInstance().ElbowTorque();

		joint_value << angle[0] << "          " << angle[1] << std::endl;
		torque_value << torque[0] << "          " << torque[1] << std::endl;
		sixdim_force_value << six_dim_force[0] << "          " << six_dim_force[1] << "          " << six_dim_force[2] << "          " << six_dim_force[3]
			<< "          " << six_dim_force[4] << "          " << six_dim_force[5] << std::endl;

		if (p2p->is_exit_thread_) {
			break;
		}
	}

	joint_value.close();
	torque_value.close();
	sixdim_force_value.close();
	p2p->is_stop = true;
	return 0;
}

void P2PControl::MoveInNewThread() {
	move_thread_handle_ = (HANDLE)_beginthreadex(NULL, 0, P2PMoveThread, NULL, 0, NULL);
}

void P2PControl::P2PMovestep() {
	double shoulder_angle = mp2pdata->target_position[0][step_num_];
	double elbow_angle= mp2pdata->target_position[1][step_num_];
	APS_absolute_move(ControlCard::ShoulderAxisId,shoulder_angle / ControlCard::Unit_Convert,3 / ControlCard::Unit_Convert);
	APS_absolute_move(ControlCard::ElbowAxisId, elbow_angle / ControlCard::Unit_Convert, 3 / ControlCard::Unit_Convert);
	Sleep(1000);
	++step_num_;
	if (step_num_ == 100) {
		is_exit_thread_ = true;
	}
}