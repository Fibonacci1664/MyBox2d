//#ifndef FOOTEST_H
//#define FOOTEST_H

#include "test.h"

class FooTest : public Test
{
public:
	FooTest()
	{
		createBody();
	}

	void createBody()
	{
		// Create body def.
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;	// This will be a dynamic body.
		myBodyDef.position.Set(0, 20);		// Set a starting position.
		myBodyDef.angle = 0;				// Set a starting angle.

		// Create body.
		b2Body* dynamicBody = m_world->CreateBody(&myBodyDef);

		// Define body shape.
		b2PolygonShape boxShape;
		boxShape.SetAsBox(1, 1);			// Area = 1 * 1 = 1m^2

		// Define body fixtures.
		// http://www.iforce2d.net/b2dtut/bodies
		// Mass = (Area of Fixture * Density of Fixture)
		// Mass = 1 * 1 = 1kg
		b2FixtureDef boxFixDef;
		boxFixDef.shape = &boxShape;
		boxFixDef.density = 1;

		// Create body fixtures.
		dynamicBody->CreateFixture(&boxFixDef);
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