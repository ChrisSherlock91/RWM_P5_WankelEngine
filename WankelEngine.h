//
//  WankelEngine.h
//  Box2D
//
//  Created by Chris Sherlock on 08/04/2015.
//
//

#include <math.h>
#include <vector>
#include <iostream>

#define METRESTOPIXELS 30
#define PIXELSTOMETRES 1/30.0f

#ifndef Box2D_WankelEngine_h
#define Box2D_WankelEngine_h

using namespace std;

class WankelEngine : public Test
{
public:
    
    double worldScale = 30;
    
    b2Body *theCog;
    b2RevoluteJointDef containerJoint;
    b2Body* ground;
    int motorSpeed = 1;
    
    
    int sideLength = 1200;
    int numCogsInside = 45;
    int numCogsOutside = 70;
    
    const int16	k_smallGroup = -1;
    
    WankelEngine()
    {
        // Ground body
        {
            b2BodyDef bd;
            ground = m_world->CreateBody(&bd);
            b2EdgeShape shape;
            shape.Set(b2Vec2(-100.0f, 0.0f), b2Vec2(100.0f, 0.0f));
            ground->CreateFixture(&shape, 0.0f);
        }
        
       //addCog(double pX, double pY, int r , int cogWidth,int cogHeight, int nCogs)
        addCog(95, 1200, 100, 25, 2, numCogsInside);
        
        
        createBodyShape(100,1100);
        
        
       createOuterShape();
        
        
        
    } //do nothing, no scene yet
    
void createOuterShape()
{
    b2Body *body;
    b2BodyDef bodydef;
    
    b2EdgeShape edgeShape;
    
    bodydef.type = b2_staticBody; //this will be a static body
    bodydef.position.Set(0 * PIXELSTOMETRES, 0 * PIXELSTOMETRES); //slightly lower position
    body = m_world->CreateBody(&bodydef); //add body to world
    
    b2FixtureDef fixtureDef;
    //boxFixtureDef.shape = &boxShape;
    fixtureDef.density = 1;
    fixtureDef.filter.groupIndex = -1;
    
    b2Vec2 btmLeft(-400 * PIXELSTOMETRES,830 * PIXELSTOMETRES);
    b2Vec2 topLeft(-400 * PIXELSTOMETRES,1700 * PIXELSTOMETRES);
    
    b2Vec2 topRight(650 * PIXELSTOMETRES,1700 * PIXELSTOMETRES);
    b2Vec2 btmRight(650 * PIXELSTOMETRES,830 * PIXELSTOMETRES);
    
    edgeShape.Set(btmLeft, topLeft);
    fixtureDef.shape = &edgeShape;
    body->CreateFixture(&fixtureDef);
    
    edgeShape.Set(topRight, btmRight);
    fixtureDef.shape = &edgeShape;
    body->CreateFixture(&fixtureDef);
    
    b2PolygonShape polygonShape;

    
    int bezierOffset = 15;
    int bezierX = (btmLeft.x + btmRight.x) / 2;
    int bezierY = (btmLeft.y + btmRight.y) / 2;
    bezierY = bezierY - bezierOffset;
    PlotBezier(btmLeft.x, btmLeft.y, bezierX, bezierY, btmRight.x, btmRight.y, polygonShape, fixtureDef, body);
    
    bezierOffset = 5;
    bezierX = (btmRight.x + topRight.x) / 2;
    bezierY = (btmRight.y + topRight.y) / 2;
    bezierX = bezierX + bezierOffset;
    PlotBezier(btmRight.x, btmRight.y, bezierX, bezierY, topRight.x, topRight.y, polygonShape, fixtureDef, body);
    
}
    
void createStaticBody(double pX, double pY, int width, int height)
{
    b2Body *body;
    b2BodyDef bodydef;
    
    b2PolygonShape boxShape;
    boxShape.SetAsBox(width * PIXELSTOMETRES,height * PIXELSTOMETRES);
    
    bodydef.type = b2_staticBody; //this will be a static body
    bodydef.position.Set(pX * PIXELSTOMETRES, pY * PIXELSTOMETRES); //slightly lower position
    body = m_world->CreateBody(&bodydef); //add body to world
    
    b2FixtureDef boxFixtureDef;
    boxFixtureDef.shape = &boxShape;
    boxFixtureDef.density = 1;
    boxFixtureDef.filter.groupIndex = k_smallGroup;
    body->CreateFixture(&boxFixtureDef);
}
    
void createBodyShape(double pX, double pY)
{
    
    //m_world->SetGravity(b2Vec2(0,0));
    
    
    b2Body *body;
    b2BodyDef bodyDef;
    bodyDef.position.Set( pX / worldScale , pY / worldScale);
    bodyDef.type = b2_dynamicBody;
    body = m_world->CreateBody(&bodyDef);
    
    b2FixtureDef fixtureDef;
    fixtureDef.restitution=0;
    fixtureDef.density=1;
    fixtureDef.filter.groupIndex = k_smallGroup;
    
    b2PolygonShape polygonShape;
    //Creates the internal "Cog" of the engine
    
    //reateCircleBoxes(float angleVar , int numOfBoxes , int boxWidth, int boxHeight
    
    createCircleBoxes(200, numCogsOutside , 20, 4 , polygonShape, body, fixtureDef);
    
    
    b2EdgeShape edgeShape;
    
    
    b2Vec2 topA(0,690 * PIXELSTOMETRES);
    b2Vec2 btmB(-500 * PIXELSTOMETRES, -285 * PIXELSTOMETRES);
    b2Vec2 btmA(500 * PIXELSTOMETRES, -285 * PIXELSTOMETRES);
    
    //Get third point in triangle
//    double s60 = sin(60 * M_PI / 180.0);
//    double c60 = cos(60 * M_PI / 180.0);
//    topA.x = c60 * (btmA.x - btmB.x) - s60 * (btmA.y - btmB.y) + btmB.x;
//    topA.y = s60 * (btmA.x - btmB.x) + c60 * (btmA.y - btmB.y) + btmB.y;
 
    
//    edgeShape.Set(btmA, btmB);
//    fixtureDef.shape = &edgeShape;
//    body->CreateFixture(&fixtureDef);
//    
//    edgeShape.Set(topA, btmB);
//    fixtureDef.shape = &edgeShape;
//    body->CreateFixture(&fixtureDef);
//    
//    edgeShape.Set(topA, btmA);
//    body->CreateFixture(&fixtureDef);
//    
    
    b2Vec2 norm(btmB.x - btmA.x,btmB.y - btmA.y);
    norm.Normalize();
    
    b2Vec2 lastPos;
    
    int bezierOffset = 6;
    int startX = btmA.x;
    int startY = btmA.y;
    int endX = btmB.x;
    int endY = btmB.y;
    int bezierX = (btmA.x + btmB.x) / 2;
    int bezierY = (btmA.y + btmB.y) / 2;
    bezierY = bezierY - bezierOffset;
    PlotBezier(startX, startY, bezierX, bezierY, endX, endY, polygonShape, fixtureDef, body);
    
    startX = btmA.x;
    startY = btmA.y;
    endX = topA.x;
    endY = topA.y;
    bezierX = (btmA.x + topA.x) / 2;
    bezierY = (btmA.y + topA.y) / 2;
    bezierX = bezierX + bezierOffset;
    PlotBezier(startX, startY, bezierX, bezierY, endX, endY, polygonShape, fixtureDef, body);
    
    startX = btmB.x;
    startY = btmB.y;
    endX = topA.x;
    endY = topA.y;
    bezierX = (btmB.x + topA.x) / 2;
    bezierY = (btmB.y + topA.y) / 2;
    bezierX = bezierX - bezierOffset;
    PlotBezier(startX, startY, bezierX, bezierY, endX, endY, polygonShape, fixtureDef, body);

    
    float32 v = 0;
    body->SetGravityScale(v);
    
//    b2Vec2 anchorA = b2Vec2(0,0);
//    b2Vec2 anchorB = b2Vec2(pX / worldScale , pY / worldScale);
//    
//    containerJoint.localAnchorA.Set(anchorA.x,anchorA.y);
//    containerJoint.localAnchorB.Set(anchorB.x,anchorB.y);
//    containerJoint.bodyA = body;
//    containerJoint.bodyB = ground;
//    containerJoint.enableMotor = true;
//    containerJoint.maxMotorTorque = 50000;
//    containerJoint.motorSpeed = motorSpeed;
//    motorSpeed = 3;
//    containerJoint.motorSpeed = motorSpeed;
//    m_world->CreateJoint(&containerJoint);

}
    
void PlotBezier( int startX, int startY , int bezierX , int bezierY, int endX, int endY , b2PolygonShape polygonShape ,
                b2FixtureDef fixtureDef , b2Body *body)
{
    for(double t=0.0;t<=1;t+=0.01)
    {
        float x = (float) (  (1-t)*(1-t)*startX + 2*(1-t)*t*bezierX+t*t*endX);
        float y = (float) (  (1-t)*(1-t)*startY + 2*(1-t)*t*bezierY+t*t*endY);
        polygonShape.SetAsBox( 20 * PIXELSTOMETRES / 2, 20 * PIXELSTOMETRES / 2 , b2Vec2(x,y) , 0);
        fixtureDef.shape = &polygonShape;
        fixtureDef.filter.groupIndex = k_smallGroup;
        body->CreateFixture(&fixtureDef);
    }
}
    
void createCircleBoxes(float angleVar , int numOfBoxes , int boxWidth, int boxHeight , b2PolygonShape polygonShape , b2Body *body , b2FixtureDef fixtureDef)
{
    for (int i = 0; i <  numOfBoxes; i++)
    {
        float angle = 2 * M_PI / numOfBoxes * i;
        polygonShape.SetAsBox( boxWidth / worldScale / 2, boxHeight / worldScale / 2 , b2Vec2(angleVar * cos(angle) /worldScale,angleVar*sin(angle)/worldScale),angle);
        fixtureDef.shape = &polygonShape;
        body->CreateFixture(&fixtureDef);
    }
}
    
    
void Keyboard(unsigned char key)
    {
        switch (key)
        {
            case '1':
                if(motorSpeed <= 20)
                {
                    //motorSpeed += 1;
                    //containerJoint.motorSpeed = motorSpeed;
                    //m_world->CreateJoint(&containerJoint);
                }
                break;
                break;
                
        }
    }


void addCog(double pX, double pY, int r , int cogWidth,int cogHeight, int nCogs)
{
        b2FixtureDef fixtureDef;
        fixtureDef.restitution=0;
        fixtureDef.density=1;
        
        b2CircleShape circleShape;
        circleShape.m_radius = r / worldScale;
        fixtureDef.shape = &circleShape;
        
        b2BodyDef bodyDef;
        bodyDef.position.Set( pX / worldScale , pY / worldScale);
        bodyDef.type = b2_dynamicBody;
        
        theCog = m_world->CreateBody(&bodyDef);
        theCog->CreateFixture(&fixtureDef);
        
        b2PolygonShape polygonShape;
        
        for (int i = 0; i < nCogs; i++)
        {
            float angle = 2 * M_PI / nCogs * i;
            polygonShape.SetAsBox(cogWidth/worldScale/2,cogHeight/worldScale/2, b2Vec2(r*cos(angle)/worldScale,r*sin(angle)/worldScale),angle);
            fixtureDef.shape = &polygonShape;
            theCog->CreateFixture(&fixtureDef);
        }
    
        b2Vec2 anchorA = b2Vec2(0,0);
        b2Vec2 anchorB = b2Vec2(pX / worldScale , pY / worldScale);
        containerJoint.localAnchorA.Set(anchorA.x,anchorA.y);
        containerJoint.localAnchorB.Set(anchorB.x,anchorB.y);
        containerJoint.bodyA = theCog;
        containerJoint.bodyB = ground;
        containerJoint.enableMotor = true;
        containerJoint.maxMotorTorque = 10000;
        containerJoint.motorSpeed = motorSpeed;
        motorSpeed = 5;
        containerJoint.motorSpeed = motorSpeed;
        m_world->CreateJoint(&containerJoint);
}
    
void Step(Settings* settings)
{
    //run the default physics and rendering
    Test::Step(settings);
    m_debugDraw.DrawString(5, m_textLine, "Chris Sherlock");
    m_textLine += 15;
        
    m_debugDraw.DrawString(5, m_textLine, "C00142296");
    m_textLine += 18;
}
    
static Test* Create()
{
    return new WankelEngine;
}
};

#endif
