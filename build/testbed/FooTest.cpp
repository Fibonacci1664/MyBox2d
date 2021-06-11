//#ifndef FOOTEST_H
//#define FOOTEST_H

#include "test.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

class FooTest : public Test
{
public:
	b2Body* dynamicBody;
	b2Body* staticBody;
	b2Body* kinematicBody;

	b2Vec2 pos;
	float angle;
	b2Vec2 vel;
	float angularVel;

	FooTest()
	{
		createBodies();
		setTransform();
		setVelocities();
		//destroyBodies();
	}

	void createBodies()
	{
		// Create dynamic body def
		b2BodyDef dynBodyDef;
		dynBodyDef.type = b2_dynamicBody;		// This will be a dynamic body.	
		dynBodyDef.position.Set(0, 20);			// Set a starting position.
		dynBodyDef.angle = 0;					// Set a starting angle.

		// Create static body def
		b2BodyDef staticBodyDef;
		staticBodyDef.type = b2_staticBody;		// This will be a static body.
		staticBodyDef.position.Set(0, 10);		// Set a starting position.
		staticBodyDef.angle = 0;				// Set a starting angle.

		// Create a kinematic body
		b2BodyDef kinBodyDef;
		kinBodyDef.type = b2_kinematicBody;		// This will be a kinematic body.
		kinBodyDef.position.Set(-18, 11);		// Set a starting position.
		kinBodyDef.angle = 0;					// Set a starting angle.

		// Create bodies and add to world
		dynamicBody = m_world->CreateBody(&dynBodyDef);
		staticBody = m_world->CreateBody(&staticBodyDef);
		kinematicBody = m_world->CreateBody(&kinBodyDef);

		// Define bodies shape
		b2PolygonShape boxShape;
		boxShape.SetAsBox(1, 1);				// Area = 1 * 1 = 1m^2

		// Define DYNAMIC body fixtures
		// http://www.iforce2d.net/b2dtut/bodies
		// Mass = (Area of Fixture * Density of Fixture)
		// Mass = 1 * 1 = 1kg
		b2FixtureDef boxFixDef;
		boxFixDef.shape = &boxShape;
		boxFixDef.density = 1;

		// Create body fixtures
		dynamicBody->CreateFixture(&boxFixDef);
		staticBody->CreateFixture(&boxFixDef);
		kinematicBody->CreateFixture(&boxFixDef);
	}

	void setTransform()
	{
		// Using rads
		//dynamicBody->SetTransform(b2Vec2(10, 20), 1);

		// Using degs
		dynamicBody->SetTransform(b2Vec2(10, 20), 45 * DEGTORAD);	// 45 deg counter-clockwise
	}

	void setVelocities()
	{
		dynamicBody->SetLinearVelocity(b2Vec2(-5, 5));		// Moving up and left 5 units/s
		dynamicBody->SetAngularVelocity(-90 * DEGTORAD);	// 90 deg per sec clockwise

		kinematicBody->SetLinearVelocity(b2Vec2(3, 0));		// Moving right 3 units per second
		kinematicBody->SetAngularVelocity(360 * DEGTORAD);	// 1 turn per second counter-clockwise
	}

	void getBodyInfo()
	{
		pos = dynamicBody->GetPosition();
		angle = dynamicBody->GetAngle();
		vel = dynamicBody->GetLinearVelocity();
		angularVel = dynamicBody->GetAngularVelocity();

		// To iterate over ALL bodies in the world
		//for (b2Body* body = m_world->GetBodyList(); body; body = body->GetNext())
		//{
		//	// Do whatever you need with each of the bodies
		//}
	}

	void destroyBodies()
	{
		// Destroying bodies like this ensure that all the fixtures and joints
		// are also destroyed that are attached to them
		m_world->DestroyBody(dynamicBody);
		m_world->DestroyBody(staticBody);
		m_world->DestroyBody(kinematicBody);
	}

	void Step(Settings& settings)
	{
		// Run the default physics and rendering
		Test::Step(settings);

		// Show some test in the main screen
		g_debugDraw.DrawString(5, m_textLine, "This is the foo test");
		m_textLine += 15;

		getBodyInfo();

		g_debugDraw.DrawString(5, m_textLine, "XPos: %.3f  YPos: %.3f Angle: %.3f", pos.x, pos.y, angle * RADTODEG);
		m_textLine += 15;
		g_debugDraw.DrawString(5, m_textLine, "Velocity: %.3f Angular Vel: %.3f", vel.x, vel.y, angularVel * RADTODEG);
		m_textLine += 15;
	}

	static Test* Create()
	{
		return new FooTest;
	}
};

//#endif // !FOOTEST_H

static int testIndex = RegisterTest("MyTests", "Foo Test", FooTest::Create);