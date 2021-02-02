#ifndef _kalman
#define _kalman

	typedef struct parameters{
		float q;
		float r;
		float x;
		float p;
		float k;
	}parameters;

	void kalman(void *m,float measurement);
#endif
