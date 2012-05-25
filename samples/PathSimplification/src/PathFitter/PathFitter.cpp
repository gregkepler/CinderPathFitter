
//
//  PathFitter.cpp
//  PathFitter
//
//  Created by Greg Kepler on 4/2/12.
//  Copyright (c) 2012 The Barbarian Group. All rights reserved.
//
//  An Algorithm for Automatically Fitting Digitized Curves
//  by Philip J. Schneider
//  from "Graphics Gems", Academic Press, 1990
//




#include <iostream>
#include <stdio.h>
#include <math.h>
#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "PathFitter.h"


#define MAXPOINTS	1000		// The most points you can have

using namespace ci;
using namespace ci::app;
using namespace std;



/*
 *  FitCurve :
 *  	Fit a Bezier curve to a set of digitized points
 *  
 *  Vec2f	*d			Array of digitized points
 *  int		nPts        Number of digitized points
 *  double	error       User-defined error squared
 *
 */

Path2d PathFitter::FitCurve(vector<Vec2f> const &d, int startIndex, int endIndex, double error)
{
    if(error == 0.0) throw "error value must be greater than 0.0";
    
    Vec2f                   tHat1, tHat2;	// Unit tangent vectors at endpoints
    vector<BezierCurve>     bezCurves;      // The vector that will store the BezierCurve values to be returned once the curve fitting is complete
    
    
    // if startIndex is the beginning of the cur
    tHat1 = computeLeftTangent(d, startIndex);
    // if endIndex is the end of the curve
    tHat2 = computeRightTangent(d, endIndex - 1);
    
    FitCubic(d, &bezCurves, startIndex, endIndex - 1, tHat1, tHat2, error);
    
    // convert the vector of BezierCurves to a Path2d object
    Path2d curvePath;
    curvePath.moveTo(bezCurves[0].pt1.x, bezCurves[0].pt1.y);
    for(int i=0; i<bezCurves.size(); i++){
        curvePath.curveTo(bezCurves[i].c1, bezCurves[i].c2, bezCurves[i].pt2);
    }
    
    return curvePath;
}






void PathFitter::addBezierCurve(vector<Vec2f> const &bezCurve, vector<BezierCurve> *bezCurves)
{
    // add curve to an array of curves which will then be returned on FitCurve
    BezierCurve newBezier;
    newBezier.pt1 = bezCurve[0];
    newBezier.pt2 = bezCurve[3];
    newBezier.c1 = bezCurve[1];
    newBezier.c2 = bezCurve[2];
    bezCurves->push_back( newBezier );
}







/*
 *  FitCubic :
 *  	Fit a Bezier curve to a (sub)set of digitized points
 *  
 *  Vec2f	*d              Array of digitized points
 *  int		first, last     Indices of first and last pts in region
 *  Vec2f	tHat1, tHat2	Unit tangent vectors at endpoints
 *  double	error           User-defined error squared
 */
void PathFitter::FitCubic(vector<Vec2f> const &d, vector<BezierCurve> *bezCurves, int first, int last, Vec2f tHat1, Vec2f tHat2, double error)

{
    vector<Vec2f>	bezCurve(4);                                    // Control points of fitted Bezier curve
    vector<double>	*u = new vector<double>(last-first+1);          // Parameter values for point
    vector<double>	*uPrime = new vector<double>(last-first+1);     // Improved parameter values 
    double	maxError;           // Maximum fitting error	 
    int		splitPoint;         // Point to split point set at	 
    int		nPts;               // Number of points in subset 
    double	iterationError;     // Error below which you try iterating
    int		maxIterations = 4;  // Max times to try iterating
    Vec2f	tHatCenter;         // Unit tangent vector at splitPoint
    int		i;		
    
    iterationError = error * error;
    nPts = last - first + 1;
    
    //  Use heuristic if region only has two points in it
    if (nPts == 2) {
        float dist = d[last].distance(d[first]) / 3.0;

		bezCurve[0] = d[first];
		bezCurve[3] = d[last];
        bezCurve[1] = bezCurve[0] + v2Scale(tHat1, dist);
        bezCurve[2] = bezCurve[3] + v2Scale(tHat2, dist);
		addBezierCurve(bezCurve, bezCurves);
		return;
    }
    
    //  Parameterize points, and attempt to fit curve
    chordLengthParameterize(d, first, last, u);
    generateBezier(d, &bezCurve, first, last, *u, tHat1, tHat2);
    
    //  Find max deviation of points to fitted curve    
    maxError = computeMaxError(d, first, last, &bezCurve, *u, &splitPoint);
    if (maxError < error) {
		addBezierCurve(bezCurve, bezCurves);
		return;
    }
    
    
    //  If error not too large, try some reparameterization
    //  and iteration 
    if (maxError < iterationError) {
		for (i = 0; i < maxIterations; i++) {
	    	uPrime = reparameterize(d, first, last, *u, &bezCurve);
	    	generateBezier(d, &bezCurve, first, last, *uPrime, tHat1, tHat2);
	    	maxError = computeMaxError(d, first, last, &bezCurve, *uPrime, &splitPoint);
	    	if (maxError < error) {
                addBezierCurve(bezCurve, bezCurves);
                return;
            }
            u = uPrime;
        }
    }
    
    // Fitting failed -- split at max error point and fit recursively
    tHatCenter = computeCenterTangent(d, splitPoint);
    FitCubic(d, bezCurves, first, splitPoint, tHat1, tHatCenter, error);
    FitCubic(d, bezCurves, splitPoint, last, tHatCenter.inverse(), tHat2, error);
}


/*
 *  GenerateBezier :
 *  Use least-squares method to find Bezier control points for region.
 *
 *  Vec2f	*d              Array of digitized points
 *  int		first, last     Indices defining region	
 *  double	*uPrime         Parameter values for region 
 *  Vec2f	tHat1, tHat2	Unit tangents at endpoints	
 */
void PathFitter::generateBezier(vector<Vec2f> const &d, vector<Vec2f> *bezCurve, int first, int last, vector<double> const &uPrime, Vec2f tHat1, Vec2f tHat2)
{
    int 	i;
    Vec2f 	A[MAXPOINTS][2];        // Precomputed rhs for eqn
    int 	nPts;                   // Number of pts in sub-curve
    double 	C[2][2];                // Matrix C
    double 	X[2];                   // Matrix X
    double 	det_C0_C1,              // Determinants of matrices
    det_C0_X,
    det_X_C1;
    double 	alpha_l, alpha_r;       // Alpha values, left and right
    Vec2f 	tmp;                    // Utility variable	
    nPts = last - first + 1;
    
    
    // Compute the A's
    for (i = 0; i < nPts; i++) {
		Vec2f		v1, v2;
		v1 = v2Scale(tHat1, B1(uPrime[i]));
		v2 = v2Scale(tHat2, B2(uPrime[i]));
		A[i][0] = v1;
		A[i][1] = v2;
    }
    
    // Create the C and X matrices
    C[0][0] = 0.0;
    C[0][1] = 0.0;
    C[1][0] = 0.0;
    C[1][1] = 0.0;
    X[0]    = 0.0;
    X[1]    = 0.0;
    
    for (i = 0; i < nPts; i++) {
        C[0][0] += A[i][0].dot(A[i][0]);
		C[0][1] += A[i][0].dot(A[i][1]);

        //					C[1][0] += V2Dot(&A[i][0], &A[i][1]);
		C[1][0] = C[0][1];
		
        C[1][1] += A[i][1].dot(A[i][1]);
        
		tmp = (d[first + i]) - 
            ((d[first] * B0(uPrime[i])) +
                ((d[first] * B1(uPrime[i])) +
                    ((d[last] * B2(uPrime[i]))+
                        (d[last] * B3(uPrime[i])))));
        
        X[0] += A[i][0].dot(tmp);
        X[1] += A[i][1].dot(tmp);
    }
    
    // Compute the determinants of C and X
    det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1];
    det_C0_X  = C[0][0] * X[1]    - C[1][0] * X[0];
    det_X_C1  = X[0]    * C[1][1] - X[1]    * C[0][1];
    
    // Finally, derive alpha values
    alpha_l = (det_C0_C1 == 0) ? 0.0 : det_X_C1 / det_C0_C1;
    alpha_r = (det_C0_C1 == 0) ? 0.0 : det_C0_X / det_C0_C1;
    
    
    
    // Checks for "dangerous" points, meaning that the alpha_l or alpha_r are abnormally large
    // from here http://newsgroups.derkeiler.com/Archive/Comp/comp.graphics.algorithms/2005-08/msg00419.html
    // This is a common problem with this algoithm.
    float dif1 = d[first].distance(d[last]);
    Boolean danger = false;
    if((alpha_l > dif1*2) || (alpha_r > dif1*2)){
        first+=0;
        danger = true;
    }
    
    
    //  If alpha negative, use the Wu/Barsky heuristic (see text)
    //  (if alpha is 0, you get coincident control points that lead to
    //  divide by zero in any subsequent NewtonRaphsonRootFind() call.
    double segLength = d[last].distance(d[first]);
    double epsilon = 1.0e-6 * segLength;
    if (alpha_l < epsilon || alpha_r < epsilon || danger)
    {
		// fall back on standard (probably inaccurate) formula, and subdivide further if needed. 
		double dist = segLength / 3.0;
		bezCurve->at(0) = d[first];
		bezCurve->at(3) = d[last];
        bezCurve->at(1) = bezCurve->at(0) + v2Scale(tHat1, dist);
        bezCurve->at(2) = bezCurve->at(3) + v2Scale(tHat2, dist);
		return; //bezCurve;
    }
    
    //  First and last control points of the Bezier curve are
    //  positioned exactly at the first and last data points 
    //  Control points 1 and 2 are positioned an alpha distance out
    //  on the tangent vectors, left and right, respectively
    bezCurve->at(0) = d[first];
    bezCurve->at(3) = d[last];
    bezCurve->at(1) = bezCurve->at(0) + v2Scale(tHat1, alpha_l);
    bezCurve->at(2) = bezCurve->at(3) + v2Scale(tHat2, alpha_r);
}


/*
 *  Reparameterize:
 *	Given set of points and their parameterization, try to find
 *   a better parameterization.
 *
 *  Point2	*d                  Array of digitized points
 *  int		first, last         Indices defining region	
 *  double	*u                  Current parameter values	
 *  Vec2f	bezCurve            Current fitted curve	
 *
 */
vector<double>* PathFitter::reparameterize(vector<Vec2f> const &d, int first, int last, vector<double> const &u, vector<Vec2f> *bezCurve)

{
    int 	nPts = last-first+1;	
    int 	i;
    vector<double>	*uPrime = new vector<double>(nPts);		//  New parameter values	
    
    for (i = first; i <= last; i++) {
		uPrime->at(i-first) = newtonRaphsonRootFind(*bezCurve, d[i], u.at(i - first));
    }
    return uPrime;
}



/*
 *  NewtonRaphsonRootFind :
 *	Use Newton-Raphson iteration to find better root.
 *
 *  Vec2f	Q;			Current fitted curve
 *  Point2 		P;		Digitized point	
 *  double 		u;		Parameter value for "P"
 */
double PathFitter::newtonRaphsonRootFind(vector<Vec2f> const &Q, Vec2f P, double u)
{
    double 		numerator, denominator;
    vector<Vec2f> 		Q1(3), Q2(2);       //  Q' and Q''
    Vec2f		Q_u, Q1_u, Q2_u;    //  u evaluated at Q, Q', & Q''
    double 		uPrime;             //  Improved u
    int 		i;
    
    // Compute Q(u)
    Q_u = bezierII(3, Q, u);
    
    // Generate control vertices for Q'
    for (i = 0; i <= 2; i++) {
		Q1[i].x = (Q[i+1].x - Q[i].x) * 3.0;
		Q1[i].y = (Q[i+1].y - Q[i].y) * 3.0;
    }
    
    // Generate control vertices for Q'' 
    for (i = 0; i <= 1; i++) {
		Q2[i].x = (Q1[i+1].x - Q1[i].x) * 2.0;
		Q2[i].y = (Q1[i+1].y - Q1[i].y) * 2.0;
    }
    
    // Compute Q'(u) and Q''(u)	
    Q1_u = bezierII(2, Q1, u);
    Q2_u = bezierII(1, Q2, u);
    
    // Compute f(u)/f'(u) 
    numerator = (Q_u.x - P.x) * (Q1_u.x) + (Q_u.y - P.y) * (Q1_u.y);
    denominator = (Q1_u.x) * (Q1_u.x) + (Q1_u.y) * (Q1_u.y) +
    (Q_u.x - P.x) * (Q2_u.x) + (Q_u.y - P.y) * (Q2_u.y);
    if (denominator == 0.0f) return u;
    
    // u = u - f(u)/f'(u) 
    uPrime = u - (numerator/denominator);
    return (uPrime);
}



/*
 *  Bezier :
 *  	Evaluate a Bezier curve at a particular parameter value
 * 
 *  int		degree;		// The degree of the bezier curve	
 *  Point2 	*V;         // Array of control points		
 *  double 	t           // Parametric value to find point for
 */
Vec2f PathFitter::bezierII(int degree, vector<Vec2f> const &V, double t)	
{
    int 	i, j;		
    Vec2f 	Q;	        // Point on curve at parameter t	
    Vec2f 	*Vtemp;		// Local copy of control points		
    
    // Copy array
    Vtemp = new Vec2f[degree+1];
    for (i = 0; i <= degree; i++) {
		Vtemp[i] = V[i];
    }
    
    // Triangle computation	
    for (i = 1; i <= degree; i++) {	
		for (j = 0; j <= degree-i; j++) {
	    	Vtemp[j].x = (1.0 - t) * Vtemp[j].x + t * Vtemp[j+1].x;
	    	Vtemp[j].y = (1.0 - t) * Vtemp[j].y + t * Vtemp[j+1].y;
		}
    }
    
    Q = Vtemp[0];
    return Q;
}


/*
 *  B0, B1, B2, B3 :
 *	Bezier multipliers
 */
double PathFitter::B0(double u)
{
    double tmp = 1.0 - u;
    return (tmp * tmp * tmp);
}


double PathFitter::B1(double u)
{
    double tmp = 1.0 - u;
    return (3 * u * (tmp * tmp));
}

double PathFitter::B2(double u)
{
    double tmp = 1.0 - u;
    return (3 * u * u * tmp);
}

double PathFitter::B3(double u)
{
    return (u * u * u);
}



/*
 *  ComputeLeftTangent, ComputeRightTangent, ComputeCenterTangent :
 *  Approximate unit tangents at endpoints and "center" of digitized curve
 *  
 *  Vec2f       *d			Digitized points
 *  int         end         Index to "left" end of region 
 */
Vec2f PathFitter::computeLeftTangent(vector<Vec2f> const &d, int end)

{
    Vec2f	tHat1;
    //tHat1 = d[end+1] - d[end];
    
    if(end==0){
        tHat1 = d[end+1] - d[end];
    }else{
        tHat1 = d[end+1] - d[end-1];
    }
    tHat1.normalize();
    return tHat1;
}


//  Vec2f       *d;			Digitized points		
//  int         end;		Index to "right" end of region 
Vec2f PathFitter::computeRightTangent(vector<Vec2f> const &d, int end)

{
    Vec2f	tHat2;
    if(end == d.size()-1){
        tHat2 = d[end-1] - d[end];
    }else{
        tHat2 = d[end-1] - d[end+1];
    }
    tHat2.normalize();
    return tHat2;
}


//  Vec2f	*d              Digitized points			
//  int		center          Index to point inside region	
Vec2f PathFitter::computeCenterTangent(vector<Vec2f> const &d, int center)
{
    Vec2f	V1, V2, tHatCenter;
    
    V1 = d[center-1] - d[center];
    V2 = d[center] - d[center+1];
    tHatCenter.x = (V1.x + V2.x)/2.0;
    tHatCenter.y = (V1.y + V2.y)/2.0;
    tHatCenter.normalize();
    return tHatCenter;
}


/*
 *  ChordLengthParameterize :
 *	Assign parameter values to digitized points 
 *	using relative distances between points.
 *
 *  Point2	*d                  Array of digitized points 
 *  int		first, last         Indices defining region	
 */
void PathFitter::chordLengthParameterize(vector<Vec2f> const &d, int first, int last, vector<double> *u)
{
    int		i;  
    
    u->at(0) = 0.0;
    for (i = first+1; i <= last; i++) {
        u->at(i-first) = u->at(i-first-1) + d[i].distance(d[i-1]);
    }
    
    for (i = first + 1; i <= last; i++) {
		u->at(i-first) = u->at(i-first) = u->at(i-first) / u->at(last-first);
    }
}




/*
 *  ComputeMaxError :
 *	Find the maximum squared distance of digitized points
 *	to fitted curve.
 *
 *  Point2	*d;                 Array of digitized points
 *  int		first, last;		Indices defining region
 *  Vec2f	bezCurve;           Fitted Bezier curve
 *  double	*u;                 Parameterization of points
 *  int		*splitPoint;        Point of maximum error
 */

double PathFitter::computeMaxError(vector<Vec2f> const &d, int first, int last, vector<Vec2f> *bezCurve, vector<double> const &u, int *splitPoint)

{
    int		i;
    double	maxDist;        // Maximum error
    double	dist;           // Current error
    Vec2f	P;              // Point on curve
    Vec2f	v;              // Vector from point to curve
    
    *splitPoint = (last - first + 1)/2;
    maxDist = 0.0;
    for (i = first + 1; i < last; i++) {
		P = bezierII(3, *bezCurve, u[i-first]);
		v = P - d[i];
        dist = v.lengthSquared();
		if (dist >= maxDist) {
	    	maxDist = dist;
	    	*splitPoint = i;
		}
    }
    return (maxDist);
}






/**
 * Scales the input vector to the new length and returns it.
 */
Vec2f PathFitter::v2Scale(Vec2f v, float newlen) {
    float len = v.length();
    if (len != 0.0) { 
        v.x *= newlen/len;   
        v.y *= newlen/len;
    }
    return v;
}


