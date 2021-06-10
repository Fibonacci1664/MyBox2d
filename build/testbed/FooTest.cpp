//#ifndef FOOTEST_H
//#define FOOTEST_H

#include "test.h"

class FooTest : public Test
{
public:
	FooTest() {}

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