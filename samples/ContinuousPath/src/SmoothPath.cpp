//
//  SmoothPath.cpp
//  PathFitter
//
//  Created by Greg Kepler on 4/5/12.
//  Copyright (c) 2012 The Barbarian Group. All rights reserved.
//
// TODO: Figure out what the deal with parts of paths sometimes not showing up. It might have to do with Cairo


#include <iostream>
#include "SmoothPath.h"
#include <time.h>
#include "PathFitter.h"
#include "cinder/app/AppBasic.h"
#include "cinder/cairo/Cairo.h"
#include "PathSimplify.h"
#include <math.h>

using namespace std;
using namespace ci;
using namespace ci::app;

SmoothPath::SmoothPath(Vec2f startPt)
{
   init(startPt, 0);
}


SmoothPath::SmoothPath(Vec2f startPt, float delay)
{
    init(startPt, delay);
}


void SmoothPath::init(Vec2f startPt, float delay) {
    drawDelay = delay;
    prevTime = getTime();
    
    startIndex = 0;
    endIndex = 0;
    addPoint(startPt);
}


void SmoothPath::update(){
    if(!inProgress) return;
    
    // if now - prevTime > threshold, update the points
    double now = getTime();
    if(mPoints.size() > 0 && now - prevTime > drawDelay){
        updateDrawingPts(true);
        prevTime = now;
    }
}


void SmoothPath::addPoint(Vec2f pt)
{
    // there has to be more than 1 pixels of space between points
    if(mPoints.size()>0 && pt.distance(mPoints[mPoints.size()]) <= 1.5)
    {
        return;
    }
    inProgress = true;
    
    // Add the new point and current time to their arrays
    mPoints.push_back(pt);
    double now = getTime();
    mPointTimes.push_back(now);
}


void SmoothPath::complete()
{
    // call updateDrawingPts one last time
    if(mPoints.size() > 1) updateDrawingPts(false);
    inProgress = false;
}



// Takes the new points, analyzes them and finds the new curves
// It then merges the new curves with the ones that already exist.
// The timeBased parameter refers to whether the news points should be determined based
// on how much time has passed or just smooth all existing points.
void SmoothPath::updateDrawingPts(bool timeBased)
{
    int ptAmt = mPoints.size();
    if(timeBased){
        double now = getTime();
        for(int i=ptAmt-1; i>=0; i--) {
            long diff =  now - drawDelay;
            
            if(mPointTimes[i] < diff){
                endIndex = i;
                break;
            }
        }
    }else{
        endIndex = ptAmt;
    }
    
    
    
    // This is causing an error for when there is only one point
    // When I remove, it causes an error for other paths that are +1 point
    // I have a feeling that it's causing the last point to trail into the
    // corners when complete sometimes. Need to rethink this strategy.
    while (endIndex - startIndex == 1 && endIndex < ptAmt) {
        endIndex++;
    }
    
    // What happens for lines that end with a single point (and not with a calculated end path point)?
    // If this happens, then I have to make the start index the previous point
    // which means that the previous point is part of the last curve.
    // Which means that that curve has to be removed and then pass that first
    // point as the startIndex for which to recreatea a new curve with the
    // new point as the end point
    if(endIndex - startIndex == 1 && endIndex == ptAmt && mCurvePath.getNumPoints()>0){
        // set the startIndex to the first point of that curve
        startIndex = findPathPt(mCurvePath.getPoint(mCurvePath.getNumPoints()-4));
        
        // remove the last curve of mCurves
        mCurvePath.removeSegment(mCurvePath.getNumSegments()-1);
        
        console() << "THIS IS HAPPENING" << endl;
    }
    
    

    // If there is more than 1 point to analyze a curve for
    if(endIndex - startIndex > 1){
        
        // get the reduced pts based on the last index of points up the most recent within the time period
        // So I have to return a set of points based on the start and end index and then smooth it
        //vector<Vec2f> newSegmentPts = PathSimplify::getSubset(mPoints, startIndex, endIndex);
        //vector<Vec2f> reducedPts = PathSimplify::SimplifyPath(newSegmentPts, 1);
        
        Path2d newCurves = PathFitter::FitCurve(mPoints, startIndex, endIndex, 2.0 );
        
        if(mCurvePath.getNumPoints() > 1) fixCtrlPts(&mCurvePath, &newCurves, 0);
        int prevCurveAmt = mCurvePath.getNumSegments();
        
        // Loop through the new Curve and add the points to mCurvePath
        // combine paths starting at the second one. adjust the control pointsof the previous one
        combinePaths(&mCurvePath, newCurves, 0);
        adjustForCorners(&mCurvePath, prevCurveAmt);
        
        startIndex = endIndex-1;
    }
}



void SmoothPath::combinePaths(Path2d *masterPath, Path2d const &newPath, int startIndex)
{
    int pointIndex = 0;
    for (int i=startIndex; i<newPath.getNumSegments(); i++) {
        int segType = newPath.getSegmentType(i);
        
        // change jumpIndex depending on the type of segment
        switch(segType){
            case Path2d::CUBICTO:
                if(masterPath->getNumPoints() == 0) masterPath->moveTo(newPath.getPoint(pointIndex));
                // do a curve to using the next 2 points as the curves and the 3rd as the end point
                masterPath->curveTo(newPath.getPoint(pointIndex+1), newPath.getPoint(pointIndex+2), newPath.getPoint(pointIndex+3));
                pointIndex += 3;
                break;
            case Path2d::MOVETO:
                // don't do anything with this point
                if(masterPath->getNumPoints() == 0) masterPath->moveTo(newPath.getPoint(pointIndex));
                pointIndex += 0;
                break;
            default:
                pointIndex += 1;
                break;
        }
    }
}



void SmoothPath::fixCtrlPts(Path2d *masterCurves, Path2d *curves, int ptId)
{
    // Average the tangents of the control points to make the connection of the 
    // previous segment to new segment smoother.
    
    if(curves->getNumPoints()<=1) return;
    if(masterCurves->getSegmentType(masterCurves->getNumSegments()-1) != Path2d::CUBICTO) return;
    
    // Store the old distances.
    // Get the tangents for both control points and average them
    // Using the old distances, create the updated positions of the control points
    
    Vec2f newCtrlPt = curves->getPoint(1);
    
    // Get the prev segment's tangent and distance
    int prevPtIndex = masterCurves->getNumPoints()-1;
    Vec2f p1 = masterCurves->getPoint(prevPtIndex);
    Vec2f oldCtrlPt = masterCurves->getPoint(prevPtIndex-1);
    Vec2f origTangent = oldCtrlPt - p1;
    origTangent.normalize();
    float origDist = ((oldCtrlPt - p1)).length();
    
    // Get the new segment's tangent and distance
    Vec2f curTangent = newCtrlPt - p1;
    curTangent.normalize();
    float dist = (newCtrlPt-p1).length();
    if(dist==0 || isnan(dist)) return;
    
    // Average the tangents and adjust the control points with that tangent
    Vec2f newTangent = ((-1 * origTangent) + curTangent)/2;
    Vec2f newCtrlPtAdj = p1 + (dist * newTangent);
    Vec2f oldCtrlPtAdj = p1 + (origDist * (-1 * newTangent));
    
    // Set the control points to the newly adjusted ones
    curves->setPoint(1, newCtrlPtAdj);
    masterCurves->setPoint(prevPtIndex-1, oldCtrlPtAdj);
}



// Find any corners along the path and adjust the control points 
// so that it doesn't try to smooth out corners.
void SmoothPath::adjustForCorners(Path2d *masterCurves, int startIndex) {
    
    // For each of the curves, look to see if there are any corners.
    // If so, change the control points so that they keep the corners
    // instead of smoothing them out.
    if(masterCurves->getNumSegments() == 0) return;
    
    int pointIndex = 3;
    for(int i=1; i<masterCurves->getNumSegments()-1; i++){
        if (masterCurves->getSegmentType(0) == Path2d::CUBICTO) {
           // get the last point, the previous point and the next point   
            // calculate the angle based on those 3 points
            
            
            Vec2f pt2 = masterCurves->getPoint(pointIndex);
            int ptIndex = findPathPt(pt2);
            Vec2f pt1 = mPoints[ptIndex-1];
            Vec2f pt3 = mPoints[ptIndex+1];
            
            
            float p0c = pt1.distance(pt2);
            float p1c = pt3.distance(pt2);
            float p0p1 = pt1.distance(pt3);
            float a = acos((p1c*p1c+p0c*p0c-p0p1*p0p1)/(2*p1c*p0c));
            
            // IF the angle is smaller than a right angle, then adjust the control points
            if(a < M_PI/2){
                Vec2f t1 = pt1 - pt2;
                t1.normalize();
                Vec2f t2 = pt3 - pt2;
                t2.normalize();
                
                Vec2f ctrl1 = pt2 + (t1 * (p0c/3));
                Vec2f ctrl2 = pt2 + (t2 * (p1c/3));
                masterCurves -> setPoint(pointIndex-1, ctrl1);
                masterCurves -> setPoint(pointIndex+1, ctrl2);
            }
            
            pointIndex += 3;
        }
    }
}



int SmoothPath::findPathPt(Vec2f const &pt) {
    for (int i=mPoints.size()-1; i>=0; i--) {
        // if pt is the same as the mPath point, then return the point index
        if (mPoints[i] == pt) return i;
    }
    return 0;
}






double SmoothPath::getTime(){
    return getElapsedSeconds() * 1000;
}


void SmoothPath::printVals(){
    int pointIndex = 0;
    for (int i=0; i<mCurvePath.getNumSegments()-1; i++) {
        
        int segType = mCurvePath.getSegmentType(i);
        
        // change jumpIndex depending on the type of segment
        switch(segType){
            case Path2d::CUBICTO:
                // do a curve to using the next 2 points as the curves and the 3rd as the end point
                console() << "pt1: " << mCurvePath.getPoint(pointIndex) << endl;
                console() << "c1: " << mCurvePath.getPoint(pointIndex+1) << endl;
                console() << "c2: " << mCurvePath.getPoint(pointIndex+2) << endl;
                console() << "pt2: " << mCurvePath.getPoint(pointIndex+3) << endl;
                pointIndex += 3;
                break;
            
        }
    }
}


Path2d SmoothPath::getCurrentPath(){
    return mCurvePath;
}

vector<Vec2f> SmoothPath::getPathPoints(){
    return mPoints;
}
vector<int> SmoothPath::getEndPoints(){
    return endPoints;
}
