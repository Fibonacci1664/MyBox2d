//#ifndef FOOTEST_H
//#define FOOTEST_H

#include "test.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

class FooTest : public Test
{
public:
	b2Body* dynamicBody;

	FooTest()
	{
		createBody();
		setTransform();
		setVelocities();
	}

	void createBody()
	{
		// Create body def
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;	// This will be a dynamic body.
		myBodyDef.position.Set(0, 20);		// Set a starting position.
		myBodyDef.angle = 0;				// Set a starting angle.

		// Create body
		dynamicBody = m_world->CreateBody(&myBodyDef);

		// Define body shape
		b2PolygonShape boxShape;
		boxShape.SetAsBox(1, 1);			// Area = 1 * 1 = 1m^2

		// Define body fixtures
		// http://www.iforce2d.net/b2dtut/bodies
		// Mass = (Area of Fixture * Density of Fixture)
		// Mass = 1 * 1 = 1kg
		b2FixtureDef boxFixDef;
		boxFixDef.shape = &boxShape;
		boxFixDef.density = 1;

		// Create body fixtures
		dynamicBody->CreateFixture(&boxFixDef);
	}

	void setTransform()
	{
		// Using rads
		//dynamicBody->SetTransform(b2Vec2(10, 20), 1);

		// Using degs
		dynamicBody->SetTransform(b2Vec2(10, 20), 45 * DEGTORAD);	// 45 deg counter clockwise
	}

	void setVelocities()
	{
		dynamicBody->SetLinearVelocity(b2Vec2(-5, 5));		// Moving up and left 5 units/s
		dynamicBody->SetAngularVelocity(-90 * DEGTORAD);	// 90 deg per sec clockwise
	}

	void Step(Settings& settings)
	{
		// Run the default physics and rendering
		Test::Step(settings);

		// Show some test in the main screen
		g_debugDraw.DrawString(5, m_textLine, "This is the foo test");
		m_textLine += 15;
	}

	static Test* Create()
	{
		return new FooTest;
	}
};

//#endif // !FOOTEST_H

static int testIndex = RegisterTest("MyTests", "Foo Test", FooTest::Create);