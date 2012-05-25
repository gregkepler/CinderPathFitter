//
//  PathFitter.h
//  PathFitter
//
//  Created by Greg Kepler on 4/2/12.
//  Copyright (c) 2012 The Barbarian Group. All rights reserved.
//

#pragma once
#include <stdio.h>
#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;


class BezierCurve {
  public:
    Vec2f   pt1;
    Vec2f   pt2;
    Vec2f   c1;
    Vec2f   c2;
};

class PathFitter{
public:
    static Path2d               FitCurve(vector<Vec2f> const &d, int startIndex, int endIndex, double error);
    
private:
    static	void            FitCubic(vector<Vec2f> const &d, vector<BezierCurve> *bezCurves, int first, int last, Vec2f tHat1, Vec2f tHat2, double error);
    static	vector<double>* reparameterize(vector<Vec2f> const &d, int first, int last, vector<double> const &u, vector<Vec2f> *bezCurve);
    static	double          newtonRaphsonRootFind(vector<Vec2f> const &Q, Vec2f P, double u);
    static	Vec2f           bezierII(int degree, vector<Vec2f> const &V, double t);
    
    static  double          B0(double u);
    static  double          B1(double u);
    static  double      	B2(double u);
    static  double      	B3(double u);
    
    static  Vec2f           computeLeftTangent(vector<Vec2f> const &d, int end);
    static	Vec2f           computeRightTangent(vector<Vec2f> const &d, int end);
    static	Vec2f			computeCenterTangent(vector<Vec2f> const &d, int end);
    static	double          computeMaxError(vector<Vec2f> const &d, int first, int last, vector<Vec2f> *bezCurve, vector<double> const &u, int *splitPoint);
    static	void			chordLengthParameterize(vector<Vec2f> const &d, int first, int last, vector<double> *u);
    static	void     	generateBezier(vector<Vec2f> const &d, vector<Vec2f> *bezCurve, int first, int last, vector<double> const &uPrime, Vec2f tHat1, Vec2f tHat2);
    
    static  void            addBezierCurve(vector<Vec2f> const &bezCurve, vector<BezierCurve> *bezCurves);
    
    static  Vec2f           v2Scale(Vec2f v, float newlen);

};
