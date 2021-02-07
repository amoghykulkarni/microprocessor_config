#ifndef _kalman
#define _kalman

	typedef struct kalman_state{
		float q;
		float r;
		float x;
		float p;
		float k;
	}kalman_state;

	void kalman(void *m,float measurement);
#endif
