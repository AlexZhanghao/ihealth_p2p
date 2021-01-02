#include<iostream>
#include"p2p_control.h"

using namespace std;

int main() {
	P2PControl p2pmove;
	p2pmove.StartMove();
	while(true){
		if(p2pmove.is_stop){
			p2pmove.StopMove();
		}
	}
	return 0;
}