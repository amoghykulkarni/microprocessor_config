#ifndef _kalman
#define _kalman

	struct measurement {
		float q;
		float r;
		float x;
		float p;
		float k;
	};

	int update(struct measurement);

#endif
