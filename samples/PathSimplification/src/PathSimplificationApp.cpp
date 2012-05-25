#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "PathFitter.h"
#include "SmoothPath.h"
#include "cinder/cairo/Cairo.h"
#include "cinder/Text.h"
#include <sstream>
#include <boost/lexical_cast.hpp>


using namespace ci;
using namespace ci::app;
using namespace std;

static const bool PREMULT = false;

class PathSimplificationApp : public AppBasic {
public:
	void setup();
	void mouseDown( MouseEvent event );
    void mouseUp( MouseEvent event );
    void mouseDrag( MouseEvent event );
    void keyDown( KeyEvent event );
	void draw();
    
private:
    void                    drawPath(cairo::Context &ctx, SmoothPath *smoothPath, int mode);
    void                    drawCircle(cairo::Context &ctx, Vec2f const &pt,  double diameter, Color col);
    void                    drawCircle(cairo::Context &ctx, Vec2f const &pt,  double diameter, Color col, bool outline);
    void                    drawLine(cairo::Context &ctx, Vec2f const &pt1, Vec2f const &pt2, float thickness, Color col);
    void                    drawCurves(cairo::Context &ctx, Path2d path, float thickness, Color col);
    
    SmoothPath              *mSmPath;
    vector<SmoothPath*>     mSmPaths;
    bool                    mousePressed;
    int                     drawMode;
    gl::Texture             mTextTexture;
};

void PathSimplificationApp::setup()
{
    drawMode = 1;
    mousePressed = false;
}

void PathSimplificationApp::mouseDown( MouseEvent event )
{
    // create a new smooth path item and add it to the array of smooth paths to draw later
    mSmPath = new SmoothPath(event.getPos());
    mSmPaths.push_back(mSmPath);
    
    mousePressed = true;
}



void PathSimplificationApp::mouseUp(MouseEvent event)
{
    // complete the smooth path when the mouse is released
    mSmPath->complete();
    mousePressed = false;
}

void PathSimplificationApp::mouseDrag(MouseEvent event)
{
    if (mousePressed) {
        // Add the current point to the current smooth path object
        Vec2f pos = Vec2f(event.getPos().x, event.getPos().y);
        mSmPath->addPoint(pos);
    }
}


void PathSimplificationApp::keyDown(KeyEvent event)
{
    switch(event.getChar())
    {
        case 'R':
        case 'r':
            // reset the path vector
            mSmPaths.clear();
            break;
    }
}




void PathSimplificationApp::draw()
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
    
    
    TextLayout layout;
    layout.clear(ColorA(0.1f,0.1f,0.1f,0.7f));
    layout.setColor( Color( 0.9f, 0.9f, 0.9f ) );
    layout.setFont( Font( "Arial", 14 ) );
    
    
    SmoothPath* lastPath;
    if(mSmPaths.size() > 0){
        lastPath = mSmPaths[mSmPaths.size()-1];
    }
    
    if(mSmPaths.size() == 0){
        layout.addCenteredLine("Click and drag to draw a line.");
        layout.addCenteredLine("Press 'R' to clear.");
    }else if(mSmPaths.size() > 0 && lastPath->inProgress){
        int segCount = (lastPath->getPathPoints().size()>0) ? lastPath->getPathPoints().size()-1 : 0;
        layout.addCenteredLine( "Segment Count: " + boost::lexical_cast<std::string>(segCount));
    }else if(mSmPaths.size() > 0 && !lastPath->inProgress){
        int oldSegCount = (lastPath->getPathPoints().size()>0) ? lastPath->getPathPoints().size()-1 : 0;
        int segCount = lastPath->getCurrentPath().getNumSegments();
        int diff = oldSegCount - segCount;
        float per = (float(diff)/float(oldSegCount)) * 100.0f;
        string msg = boost::lexical_cast<std::string>(diff) + " of " + boost::lexical_cast<std::string>(oldSegCount) + " segments were removed. Saving " +boost::lexical_cast<std::string>(per) + "%";
        layout.addCenteredLine(msg);
    }
    
    
    Surface8u rendered = layout.render( true, PREMULT );
    mTextTexture = gl::Texture( rendered );
    
    if( mTextTexture )
		gl::draw( mTextTexture,  Vec2f(10, 10)  );
}



// Draw each path onto the passed cairo context
void PathSimplificationApp::drawPath(cairo::Context &ctx, SmoothPath *smoothPath, int mode)
{
    // initialize the line settings
    float thickness = 1.0;
    ctx.setLineWidth( 1.0f );
    ctx.setLineCap(cairo::LINE_CAP_ROUND);
    ctx.setLineJoin(cairo::LINE_JOIN_ROUND);

    
    if(smoothPath->inProgress){
        // draw lines point to point
        vector<Vec2f> pathPoints = smoothPath->getPathPoints();
        for (int i=1; i<pathPoints.size(); i++) {
            drawLine(ctx, pathPoints[i-1], pathPoints[i], 1.0, Color(1.0, 0.0, 0.0));
            drawCircle(ctx, pathPoints[i], 2, Color(1.0, 0.0, 0.0));
        }
    }else{
        // draw smooth lines
        Path2d path = smoothPath->getCurrentPath();
        drawCurves(ctx, path, thickness, Color(255.0, 0.0, 0.0));
        
        int pointIndex = 0;
        // draw circles at the bezier points
        for (int i=0; i<path.getNumSegments(); i++) {
            Vec2f c1 = path.getPoint(pointIndex+1);
            Vec2f c2 = path.getPoint(pointIndex+2);
            Vec2f pt1 = path.getPoint(pointIndex);
            Vec2f pt2 = path.getPoint(pointIndex+3);
            drawCircle(ctx, c1, 1, Color(1.0f, 1.0f, 0.0f), true);
            drawCircle(ctx, c2, 1, Color(0.0f, 1.0f, 1.0f));
            drawLine(ctx, c1, pt1, 1, Color(1.0, 1.0, 0.0 ));
            drawLine(ctx, c2, pt2, 1, Color(0.0, 1.0, 1.0 ));
            pointIndex += 3;
        }
    }   
}



void PathSimplificationApp::drawCircle(cairo::Context &ctx, Vec2f const &pt, double diameter,  Color col, bool outline)
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
void PathSimplificationApp::drawCircle(cairo::Context &ctx, Vec2f const &pt, double diameter,  Color col)
{
    drawCircle(ctx, pt, diameter, col, false);
}



void PathSimplificationApp::drawLine(cairo::Context &ctx, Vec2f const &pt1, Vec2f const &pt2, float thickness, Color col)
{
    ctx.setLineWidth(thickness);
    ctx.setSource(col);
    ctx.newSubPath();
    ctx.moveTo(pt1.x, pt1.y);
    ctx.lineTo(pt2.x, pt2.y);
    
    ctx.closePath();
    ctx.stroke();
}


void PathSimplificationApp::drawCurves(cairo::Context &ctx, Path2d path, float thickness, Color col)
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



CINDER_APP_BASIC( PathSimplificationApp, RendererGl )
