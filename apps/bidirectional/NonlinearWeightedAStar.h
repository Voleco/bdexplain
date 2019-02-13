

#ifndef NonlinearWeightedAStar_h
#define NonlinearWeightedAStar_h

#include "AStarOpenClosed.h"
#include <cmath>

const double n_weight = 2.0;
const double n_PI = 3.1415927;
const double additive_bound_K = 10;

template <class state>
struct PureHCompare {

	//f = h
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double f1, f2;
		f1 = i1.h ;
		f2 = i2.h;
		if (fequal(f1, f2))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(f1, f2));
	}
};

inline double quadraticEquationRoot(double a, double b, double c)
{
	double delta = b*b - 4*a*c;
	double x = (-b + sqrt(delta)) / (2 * a);
	return x;
}

inline double cubicEquationRoot(double a, double b, double c, double d)
{
	double P = 36 * a*b*c - 8 * b*b*b - 108 * a*a*d;
	double Q = 12 * a*c - 4 * b*b;
	double delta = P*P + Q*Q*Q;
	double x = (-2 * b + cbrt(P + sqrt(delta)) + cbrt(P - sqrt(delta))) / (6 * a);
	return x;
}


//curve below y + wx above y + x
template <class state>
struct QuadraticCompare1 {

	//f = g + (2w-1)h + sqrt(g^2 + h^2 -2gh + 4wgh)
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double f1, f2;
		f1 = i1.g + (2 * n_weight - 1)*i1.h + sqrt(i1.g*i1.g + i1.h*i1.h - 2 * i1.g*i1.h + 4 * n_weight*i1.g*i1.h);
		f2 = i2.g + (2 * n_weight - 1)*i2.h + sqrt(i2.g*i2.g + i2.h*i2.h - 2 * i2.g*i2.h + 4 * n_weight*i2.g*i2.h);
		if (fequal(f1, f2))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(f1, f2));
	}
};

//curve above y + wx below y + x
template <class state>
struct QuadraticCompare2 {

	//f = g + h + sqrt(g^2 + h^2 + 2gh + 4w(w-1)h^2)
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double f1, f2;
		f1 = i1.g + i1.h + sqrt(i1.g*i1.g + i1.h*i1.h + 2 * i1.g*i1.h + 4 * n_weight*(n_weight - 1)*i1.h*i1.h);
		f2 = i2.g + i2.h + sqrt(i2.g*i2.g + i2.h*i2.h + 2 * i2.g*i2.h + 4 * n_weight*(n_weight - 1)*i2.h*i2.h);
		if (fequal(f1, f2))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(f1, f2));
	}
};

template <class state>
struct RickCompare {
	//f = g + h   if h==0
	//f = g + h + K    otherwise

	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double f1, f2;
		if (i1.h == 0)
			f1 = i1.g;
		else
			f1 = i1.g + i1.h + additive_bound_K;
		if (i2.h == 0)
			f2 = i2.g;
		else
			f2 = i2.g + i2.h + additive_bound_K;
		if (fequal(f1, f2))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(f1, f2));
	}
};

template <class state>
struct RickCompare2 {
	//h_i : h of start
	//f = g + h + K*min(h , h_i) / h_i

	//RickCompare2(hi):h_i(hi){ printf("h_i: %d\n", h_i); }

	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double f1, f2;
		double minh1, minh2;
		if (i1.h < h_i)
			minh1 = i1.h;
		else
			minh1 = h_i;

		if (i2.h < h_i)
			minh2 = i2.h;
		else
			minh2 = h_i;

		f1 = i1.g + i1.h + additive_bound_K *minh1/h_i;
		f2 = i2.g + i2.h + additive_bound_K*minh2/h_i;
		if (fequal(f1, f2))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(f1, f2));
	}
	double h_i;
};

template <class state>
struct LinearCompare {

	//f = g + h -K + sqrt( (g+h-K)^2 + 4hK)
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double f1, f2;
		double b1 = i1.g + i1.h - additive_bound_K;
		double b2 = i2.g + i2.h - additive_bound_K;
		f1 = b1+ sqrt(b1*b1 + 4 * i1.h*additive_bound_K);
		f2 = b2 + sqrt(b2*b2 + 4 * i2.h*additive_bound_K);
		if (fequal(f1, f2))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(f1, f2));
	}
};

#endif /* NonlinearWeightedAStar_h */
