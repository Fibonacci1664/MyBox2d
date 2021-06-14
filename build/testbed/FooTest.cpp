//#ifndef FOOTEST_H
//#define FOOTEST_H

#include "test.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

class FooTest : public Test
{
public:
	b2Body* dynamicFrictionBody;
	b2Body* staticEdgeLineBody;
	b2Body* dynamicCircleBody;
	b2Body* dynamicPolyBody;
	b2Body* dynamicBoxBody;
	b2Body* staticBody;
	b2Body* kinematicBody;

	b2Vec2 pos;
	float angle;
	b2Vec2 vel;
	float angularVel;

	FooTest()
	{
		/*createDynamicCircle();
		createDynamicPolygon();
		createDynamicBox();*/
		createFrictionTest();
		createStaticEdgeLine();

		/*createStaticBox();
		createKinematicBox();
		setTransform();
		setVelocities();*/
		//destroyBodies();
	}

	void cleanUpFixtures()
	{
		// If you want to remove a fixture from a body,
		// call the body’s DestroyFixture function:
		/*b2Fixture* myFixture = dynamicBody->CreateFixture(&myFixtureDef);
		...
		dynamicBody->DestroyFixture(myFixture);*/
	}

	void getFixtureList()
	{
		// To iterate over all the fixture on a particular body
		//for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext())
		//{
		//	//do something with the fixture 'f'
		//}

		// If you know there is only one fixture on the body,
		// you could just use the first element of the list:
		//b2Fixture* f = body->GetFixtureList();
		//do something with the fixture 'f'
	}

	void createFrictionTest()
	{
		b2BodyDef dynFrictionBodyDef;
		dynFrictionBodyDef.type = b2_dynamicBody;
		dynFrictionBodyDef.position.Set(0, 20);

		dynamicFrictionBody = m_world->CreateBody(&dynFrictionBodyDef);

		b2PolygonShape polygonShape;
		b2FixtureDef polyFrictionFixDef;
		polyFrictionFixDef.shape = &polygonShape;
		polyFrictionFixDef.density = 1;
		//polyFrictionFixDef.friction = 1;
		polyFrictionFixDef.restitution = 1;

		for (int i = 0; i < 4; ++i)
		{
			b2Vec2 pos(sinf(i * 90 * DEGTORAD), cosf(i * 90 * DEGTORAD));
			polygonShape.SetAsBox(1, 1, pos, 0);
			polyFrictionFixDef.friction = i / 4.0f;
			dynamicFrictionBody->CreateFixture(&polyFrictionFixDef);
		}
	}

	void createStaticEdgeLine()
	{
		b2BodyDef staticEdgeBodyDef;
		staticEdgeBodyDef.type = b2_staticBody;
		staticEdgeBodyDef.position.Set(0, 0);

		staticEdgeLineBody = m_world->CreateBody(&staticEdgeBodyDef);

		b2EdgeShape edgeLineShape;
		edgeLineShape.m_vertex1.Set(-15, 0);
		edgeLineShape.m_vertex2.Set(15, 3);

		b2FixtureDef edgeFixture;
		edgeFixture.shape = &edgeLineShape;

		staticEdgeLineBody->CreateFixture(&edgeFixture);
	}

	void createDynamicPolygon()
	{
		// There is a limit of 8 vertices per polygon, must be a covex shape and
		// vertices must be specified in counter-clockwise order
		// http://www.iforce2d.net/b2dtut/fixtures
		b2Vec2 vertices[5];
		vertices[0].Set(-1 + 10, 2);
		vertices[1].Set(-1 + 10, 0);
		vertices[2].Set(0 + 10, -3);
		vertices[3].Set(1 + 10, 0);
		vertices[4].Set(1 + 10, 1);

		b2BodyDef dynPolyBodyDef;
		dynPolyBodyDef.type = b2_dynamicBody;
		dynPolyBodyDef.position.Set(0, 20);

		dynamicPolyBody = m_world->CreateBody(&dynPolyBodyDef);

		b2PolygonShape polyShape;
		polyShape.Set(vertices, 5);		// Pass the array to the shape

		b2FixtureDef polyFixtureDef;
		polyFixtureDef.shape = &polyShape;

		// This will NOT appear to change the speed at which things fall
		// through the "air" as unless air resistance if also simulated then
		// the simulation is essentially a vacuum and ALL object regardless of
		// weight fall at exactly the same speed in a vacuum.
		// Feather and Bowling Ball experiment.
		polyFixtureDef.density = 300;

		//dynamicPolyBody->CreateFixture(&polyFixtureDef);
		dynamicCircleBody->CreateFixture(&polyFixtureDef);
	}

	void createDynamicCircle()
	{
		// Create dynamic body def
		b2BodyDef dynBodyDef;
		dynBodyDef.type = b2_dynamicBody;		// This will be a dynamic body.	
		dynBodyDef.position.Set(-10, 20);		// Set a starting position.
		//dynBodyDef.angle = 0;					// Set a starting angle.

		// Create bodies and add to world
		dynamicCircleBody = m_world->CreateBody(&dynBodyDef);
		
		// Define bodies shape
		b2CircleShape dynCircleShape;
		dynCircleShape.m_p.Set(0, 0);
		dynCircleShape.m_radius = 1;

		// Define circle fixture/s
		b2FixtureDef circleFixDef;
		circleFixDef.shape = &dynCircleShape;
		circleFixDef.density = 100;
		
		// Create the fixture
		dynamicCircleBody->CreateFixture(&circleFixDef);	
	}

	void createDynamicBox()
	{		
		// Create dynamic body def
		b2BodyDef dynBodyDef;
		dynBodyDef.type = b2_dynamicBody;		// This will be a dynamic body.	
		dynBodyDef.position.Set(10, 20);		// Set a starting position.
		dynBodyDef.angle = 0;					// Set a starting angle.

		// Create bodies and add to world
		dynamicBoxBody = m_world->CreateBody(&dynBodyDef);

		// Define bodies shape
		b2PolygonShape boxShape;
		boxShape.SetAsBox(2, 1, b2Vec2(20, 0), 0);				// Area = 1 * 1 = 1m^2

		// Define DYNAMIC body fixtures
		// http://www.iforce2d.net/b2dtut/bodies
		// Mass = (Area of Fixture * Density of Fixture)
		// Mass = 1 * 1 = 1kg

		// Define the boxes fixture/s
		b2FixtureDef boxFixDef;
		boxFixDef.shape = &boxShape;
		boxFixDef.density = 200;

		// Create body fixtures
		//dynamicBoxBody->CreateFixture(&boxFixDef);
		dynamicCircleBody->CreateFixture(&boxFixDef);
	}

	void createStaticBox()
	{
		// Create static body def
		b2BodyDef staticBodyDef;
		staticBodyDef.type = b2_staticBody;		// This will be a static body.
		staticBodyDef.position.Set(0, 10);		// Set a starting position.
		//staticBodyDef.angle = 0;				// Set a starting angle.

		// Create the body
		staticBody = m_world->CreateBody(&staticBodyDef);

		// Define bodies shape
		b2PolygonShape boxShape;
		boxShape.SetAsBox(1, 1);

		// Define static bodies fixture/s
		b2FixtureDef staticBoxFixDef;
		staticBoxFixDef.shape = &boxShape;
		staticBoxFixDef.density = 1;

		staticBody->CreateFixture(&staticBoxFixDef);
	}

	void createKinematicBox()
	{
		// Create a kinematic body
		b2BodyDef kinBodyDef;
		kinBodyDef.type = b2_kinematicBody;		// This will be a kinematic body.
		kinBodyDef.position.Set(-18, 11);		// Set a starting position.
		//kinBodyDef.angle = 0;					// Set a starting angle.

		// Create the kinematic body
		kinematicBody = m_world->CreateBody(&kinBodyDef);

		// Define bodies shape
		b2PolygonShape boxShape;
		boxShape.SetAsBox(1, 1);

		// Define static bodies fixture/s
		b2FixtureDef kinematicBoxFixDef;
		kinematicBoxFixDef.shape = &boxShape;
		kinematicBoxFixDef.density = 1;

		kinematicBody->CreateFixture(&kinematicBoxFixDef);
	}

	void setTransform()
	{
		// Using rads
		//dynamicBody->SetTransform(b2Vec2(10, 20), 1);

		// Using degs
		dynamicBoxBody->SetTransform(b2Vec2(10, 20), 45 * DEGTORAD);	// 45 deg counter-clockwise
	}

	void setVelocities()
	{
		dynamicBoxBody->SetLinearVelocity(b2Vec2(-5, 5));		// Moving up and left 5 units/s
		dynamicBoxBody->SetAngularVelocity(-90 * DEGTORAD);	// 90 deg per sec clockwise

		kinematicBody->SetLinearVelocity(b2Vec2(3, 0));		// Moving right 3 units per second
		kinematicBody->SetAngularVelocity(360 * DEGTORAD);	// 1 turn per second counter-clockwise
	}

	void getBodyInfo()
	{
		pos = dynamicBoxBody->GetPosition();
		angle = dynamicBoxBody->GetAngle();
		vel = dynamicBoxBody->GetLinearVelocity();
		angularVel = dynamicBoxBody->GetAngularVelocity();

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
		m_world->DestroyBody(dynamicBoxBody);
		/*m_world->DestroyBody(staticBody);
		m_world->DestroyBody(kinematicBody);*/
	}

	void Step(Settings& settings)
	{
		// Run the default physics and rendering
		Test::Step(settings);

		// Show some test in the main screen
		g_debugDraw.DrawString(5, m_textLine, "This is the foo test");
		m_textLine += 15;

		//getBodyInfo();

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