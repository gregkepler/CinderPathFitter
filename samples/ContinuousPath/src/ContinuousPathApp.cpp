#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/cairo/Cairo.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"
#include <iostream>

#include "PathFitter.h"
#include "SmoothPath.h"

using namespace ci;
using namespace ci::app;
using namespace std;



class ContinuousPathApp : public AppBasic {
    
public:
	void setup();
	void mouseDown( MouseEvent event );	
    void mouseUp(MouseEvent event);
    void mouseDrag( MouseEvent event );
	void update();
	void draw();
    void keyDown( KeyEvent event);
    
private:
    void                    drawPath(cairo::Context &ctx, SmoothPath *smoothPath, int mode);
    void                    drawCircle(cairo::Context &ctx, Vec2f const &pt,  double diameter, Color col);
    void                    drawCircle(cairo::Context &ctx, Vec2f const &pt,  double diameter, Color col, bool outline);
    void                    drawLine(cairo::Context &ctx, Vec2f const &pt1, Vec2f const &pt2, float thickness, Color col);
    void                    drawCurves(cairo::Context &ctx, Path2d path, float thickness, Color col);
    
    vector<BezierCurve>     curves;
    
    SmoothPath              *mSmPath;
    vector<SmoothPath*>     mSmPaths;
    bool                    mousePressed;
    int                     drawMode;
};



void ContinuousPathApp::setup()
{
    drawMode = 1;
    mousePressed = false;
    
    /*
     // create test curve
     float frequency = 1;
     int amt = 63;
     int testW = 350;
     for (int i=0; i<amt; i++) {
     float a = ((360*frequency)/amt * i) * M_PI/180;
     float _x = 10 + ((testW/amt)*i);
     float _y = 200 + sin(a) * 120;
     Vec2f p = Vec2f(_x, _y);
     
     if(i == 0){
     mSmPath = new SmoothPath(p); 
     }else{
     mSmPath->addPoint(p); 
     }
     }*/
    
    /*
     // create test randomly distanced zigzag
     Rand::randSeed(4);
     int direction = 1;
     float left = 10.0;
     float prev_x = left;
     float step = 40;
     for (int i=0; i<amt; i++) {
     //step = (rand() * 30.0);
     step = Rand::randFloat(20.0, 60.0);
     //float a = ((360*frequency)/amt * i) * M_PI/180;
     
     float _x = prev_x + (step * direction);
     if(_x>testW || _x<left){
     direction *= -1;
     _x += (direction * step)*2;
     }
     float _y = left + (i * 4);
     Vec2f p = Vec2f(_x, _y);
     
     if(i == 0){
     mSmPath = new SmoothPath(p); 
     }else{
     mSmPath->addPoint(p); 
     }
     prev_x = _x;
     }
     
     mSmPath->complete();
     mSmPaths.push_back(mSmPath);*/
}



void ContinuousPathApp::mouseDown( MouseEvent event )
{
    // create a new smooth path item and add it to the array of smooth paths to draw later
    mSmPath = new SmoothPath(event.getPos(), 60);
    mSmPaths.push_back(mSmPath);
    
    mousePressed = true;
}



void ContinuousPathApp::mouseUp(MouseEvent event)
{
    // complete the smooth path when the mouse is released
    mSmPath->complete();
    mousePressed = false;
}



void ContinuousPathApp::mouseDrag(MouseEvent event)
{
    if (mousePressed) {
        // Add the current point to the current smooth path object
        Vec2f pos = Vec2f(event.getPos().x, event.getPos().y);
        mSmPath->addPoint(pos);
    }
}



void ContinuousPathApp::keyDown(KeyEvent event)
{
    switch(event.getChar())
    {
        case 'R':
        case 'r':
            // reset the path vector
            mSmPaths.clear();
            break;
            
        case '1':
            // show just the smoothed path
            drawMode = 1;
            break;
            
        case '2':
            // Show smoothed path with the original points and bezier points
            drawMode = 2;
            break;
            
        case '3':
            // Show smoothed path with bezier anchor and control points
            drawMode = 3;
            break;
            
        case '4':
            // Show smoothed path with bezier anchor points and lines connecting bezier anchor points
            drawMode = 4;
            break;
            
        case '5':
            // Show unsmoothed path only
            drawMode = 5;
            break;
            
        case '6':
            // Show smoothed path with original points
            drawMode = 6;
            break;
            
        case 'p':
        case 'P':
            // print the smoothed path bezier point values to the console
            mSmPath->printVals();
            break;
            
    }
}



void ContinuousPathApp::update()
{
    // Update all of the smoothed paths
    for(int i=0; i<mSmPaths.size(); i++){
        
        mSmPaths[i]->update();
    }
    
}



void ContinuousPathApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );
    
    cairo::SurfaceImage surface( getWindowWidth(), getWindowHeight());
    cairo::Context ctx( surface );
    
    // draw each path
    for(int i=0; i<mSmPaths.size(); i++){
        // pass the cairo context to draw onto
        drawPath(ctx, mSmPaths[i], drawMode);
    }
    
    // draw the surface
    gl::Texture myTexture = surface.getSurface();
    gl::draw(myTexture);
    
}



// Draw each path onto the passed cairo context
void ContinuousPathApp::drawPath(cairo::Context &ctx, SmoothPath *smoothPath, int mode)
{
    // update the drawing points only if the line hasn't been completed
    
    Path2d path = smoothPath->getCurrentPath();
    vector<Vec2f> pathPoints = smoothPath->getPathPoints();
    vector<int>   endPoints = smoothPath->getEndPoints();
    
    if(path.getNumPoints() == 0) return;
    
    
    // initialize the line settings
    float thickness = 1.0;
    ctx.setLineWidth( 1.0f );
    ctx.setLineCap(cairo::LINE_CAP_ROUND);
    ctx.setLineJoin(cairo::LINE_JOIN_ROUND);
    
    Vec2f pt = path.getPoint(0);
    
    
    // draw bezier line
    if (mode != 5) {
        // draw the line based on bezier points
        drawCurves(ctx, path, thickness, Color(255.0, 0.0, 0.0));
    }
    
    
    // draw circles at each of the original path points
    if(mode != 1 && mode != 5){
        for (int i=0; i<pathPoints.size(); i++) {
            Vec2f pt = pathPoints[i];        
            drawCircle(ctx, pt, 1, Color( 1.0f, 1.0f, 1.0f ));
        }
    }
    
    
    int i;
    int pointIndex = 0;
    // draw circles at the bezier points
    for (i=0; i<path.getNumSegments(); i++) {
        
        int segType = path.getSegmentType(i);
        
        // change jumpIndex depending on the type of segment
        switch(segType){
            case Path2d::CUBICTO:
            {
                Vec2f c1 = path.getPoint(pointIndex+1);
                Vec2f c2 = path.getPoint(pointIndex+2);
                Vec2f pt1 = path.getPoint(pointIndex);
                Vec2f pt2 = path.getPoint(pointIndex+3);
                
                
                if (mode == 2 || mode == 3) {
                    if(mode == 2){
                        for(int j=0; j<endPoints.size(); j++){
                            if(endPoints[j] == pointIndex){
                                drawCircle(ctx, pt2, 8, Color(0.0f, 1.0f, 1.0f), true);
                            }
                        }
                    }
                    if(mode == 3){
                        // draw the control points and tangent lines
                        drawCircle(ctx, c1, 2, Color(1.0f, 1.0f, 0.0f), true);
                        drawCircle(ctx, c2, 2, Color(0.0f, 1.0f, 1.0f));
                        drawLine(ctx, c1, pt1, 1, Color(1.0, 1.0, 0.0 ));
                        drawLine(ctx, c2, pt2, 1, Color(0.0, 1.0, 1.0 ));
                    }
                    drawCircle(ctx, pt2, 2, Color(1.0f, 0.0f, 1.0f));
                    drawCircle(ctx, pt1, 2, Color(1.0f, 0.0f, 0.0f), true);
                }
                
                if (mode == 4) {
                    
                    drawLine(ctx, pt1, pt2, 1, Color(1.0, 1.0, 1.0 ));
                    drawCircle(ctx, pt1, 5, Color(1.0, 0.0, 0.0), true);
                    drawCircle(ctx, pt2, 3, Color(1.0, 0.0, 1.0));
                }
                
                pointIndex += 3;
                break;
            }
                
            case Path2d::MOVETO:
                // don't do anything with this point
                pointIndex += 0;
                break;
                
            default:
                pointIndex += 1;
                break;
        }
    }
    
    
    if (mode== 5) {
        // draw a line between the last bezier point and the current point
        for (i=1; i<pathPoints.size(); i++) {
            drawLine(ctx, pathPoints[i-1], pathPoints[i], 1.0, Color(1.0, 0.0, 0.0));
        }
    }
}



void ContinuousPathApp::drawCircle(cairo::Context &ctx, Vec2f const &pt, double diameter,  Color col, bool outline)
{
    ctx.setLineWidth(1);
    ctx.setSource( col );
    ctx.newSubPath();
    ctx.circle( pt.x, pt.y, diameter );
    ctx.closePath();
    
    if(outline){
        ctx.stroke();
    }else{
        ctx.fill();   
    }
}



// draws a circle with outline drawing at the default of false
void ContinuousPathApp::drawCircle(cairo::Context &ctx, Vec2f const &pt, double diameter,  Color col)
{
    drawCircle(ctx, pt, diameter, col, false);
}



void ContinuousPathApp::drawLine(cairo::Context &ctx, Vec2f const &pt1, Vec2f const &pt2, float thickness, Color col)
{
    
    ctx.setLineWidth(thickness);
    ctx.setSource(col);
    ctx.newSubPath();
    ctx.moveTo(pt1.x, pt1.y);
    ctx.lineTo(pt2.x, pt2.y);
    
    ctx.closePath();
    ctx.stroke();
}



void ContinuousPathApp::drawCurves(cairo::Context &ctx, Path2d path, float thickness, Color col)
{    
    ctx.setLineWidth(thickness);
    ctx.setSource(col);
    ctx.newSubPath();
    
    int pointIndex = 0;
    for (int i=0; i<path.getNumSegments(); i++) {
        
        int segType = path.getSegmentType(i);
        
        // change jumpIndex depending on the type of segment
        switch(segType){
            case Path2d::CUBICTO:
                if(i==0) ctx.moveTo(path.getPoint(pointIndex));
                // do a curve to using the next 2 points as the curves and the 3rd as the end point
                ctx.curveTo(path.getPoint(pointIndex+1), path.getPoint(pointIndex+2), path.getPoint(pointIndex+3));
                pointIndex += 3;
                break;
            case Path2d::MOVETO:
                // don't do anything with this point
                ctx.moveTo(path.getPoint(pointIndex));
                pointIndex += 0;
                break;
            default:
                pointIndex += 1;
                break;
        }
    }
    ctx.stroke();
}



CINDER_APP_BASIC( ContinuousPathApp, RendererGl )
