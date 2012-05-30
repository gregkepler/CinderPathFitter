//
//  SmoothPath.h
//  PathFitter
//
//  Created by Greg Kepler on 4/5/12.
//  Copyright (c) 2012 The Barbarian Group. All rights reserved.
//

#pragma once
#include <stdio.h>
#include "cinder/app/AppBasic.h"
#include "PathFitter.h"
#include "cinder/cairo/Cairo.h"

using namespace ci;
using namespace ci::app;
using namespace std;


class SmoothPath{
public:
    SmoothPath(){};
    SmoothPath(Vec2f startPt);
    SmoothPath(Vec2f startPt, float drawDelay);
    void            init(Vec2f startPt, float drawDelay);
    void            addPoint(Vec2f pt);
    void            complete();
    void            update();
    //void          draw(cairo::Context &ctx, int mode);
    void            printVals();
    Path2d          getCurrentPath();
    vector<Vec2f>   getPathPoints();
    vector<int>     getEndPoints();
    
    bool            inProgress;
    
    
    
    
private:
    void    fixCtrlPts(Path2d *masterCurves, Path2d *pts, int ptId);
    void    updateDrawingPts(bool timeBased);
    int     findPathPt(Vec2f const &pt);
    void    combinePaths(Path2d *masterPath, Path2d const &newPath, int startIndex);
    void    adjustForCorners(Path2d *masterCurves, int startIndex);
    double  getTime();
    
    vector<Vec2f>           mTempPoints;            // add points as the path is being drawn
    vector<Vec2f>           mPoints;                // final points, once simplified
    vector<double>          mTempPointTimes;
    vector<double>          mPointTimes;
    vector<int>             endPoints;
    Path2d                  mCurvePath;
    
    int                     smoothIndex;
    int                     smoothInterval;
    float                   curveError = 3.0;
    int                     startIndex;
    int                     endIndex;
    double                  prevTime;
    float                   drawDelay;
};

