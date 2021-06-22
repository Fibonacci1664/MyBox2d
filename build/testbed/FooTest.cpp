//#ifndef FOOTEST_H
//#define FOOTEST_H

#include "test.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

enum class MoveState
{
	MS_STOP, MS_LEFT, MS_RIGHT
};

class FooTest : public Test
{
public:
	b2Vec2 clickedPoint;
	b2Body* dynRotationBody;

	b2Body* constantSpeedBody;
	MoveState ms;

	// Keep track of 3 bodies
	b2Body* forcesBodies[3];

	b2Body* dynamicFrictionBody;
	b2Body* staticEdgeLineBody;
	b2Body* dynamicCircleBody;
	b2Body* dynamicPolyBody;
	b2Body* dynamicBoxBody;
	b2Body* staticBody;
	b2Body* kinematicBody;

	bool forceOn;
	bool torqueOn;
	b2Vec2 pos;
	float angle;
	b2Vec2 vel;
	float angularVel;

	FooTest()
	{
		forceOn = false;
		torqueOn = false;
		clickedPoint = b2Vec2(0, 20);		// Initial starting point

		rotationTest();

		//constantSpeedTest();
		//createWalls();

		/*createDynamicCircle();
		createDynamicPolygon();
		createDynamicBox();*/
		//createFrictionTest();
		/*forcesImpulseTest();
		createStaticEdgeLine();*/

		/*createStaticBox();
		createKinematicBox();
		setTransform();
		setVelocities();*/
		//destroyBodies();
	}

	void MouseDown(const b2Vec2& point)
	{
		// Store the last mouse down position
		clickedPoint = point;

		// Carry out default(normal) behaviour
		Test::MouseDown(point);
	}

	void rotationTest()
	{
		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;

		b2PolygonShape polyShape;
		b2Vec2 vertices[6];

		for (int i = 0; i < 6; ++i)
		{
			float angle = -i / 6.0f * 360 * DEGTORAD;
			vertices[i].Set(sinf(angle), cosf(angle));
		}

		vertices[0].Set(0, 4);		// Change one o fthe verices to be pointy
		polyShape.Set(vertices, 6);

		b2FixtureDef fixDef;
		fixDef.shape = &polyShape;
		fixDef.density = 1;

		bodyDef.position.Set(0, 10);
		dynRotationBody = m_world->CreateBody(&bodyDef);
		dynRotationBody->CreateFixture(&fixDef);

		m_world->SetGravity(b2Vec2(0, 0));
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

	virtual void Keyboard(int key)
	{
		switch (key)
		{
			case 'Q':
			{
				//forceOn = !forceOn;
				// 
				// Move left
				ms = MoveState::MS_LEFT;
				break;
			}
			case 'W':
			{
				// Apply immediate force upwards
				//forcesBodies[1]->ApplyLinearImpulse(b2Vec2(0, 50), forcesBodies[1]->GetWorldPoint(b2Vec2(1, 1)), true);
				
				// Stop moving
				ms = MoveState::MS_STOP;
				break;
			}
			case 'E':
			{
				// Teleport or 'warp' to a new location
				//forcesBodies[2]->SetTransform(b2Vec2(10, 20), 0);

				// Move right
				ms = MoveState::MS_RIGHT;
				break;
			}
			case 'A':
			{
				torqueOn = !torqueOn;
				break;
			}
			case 'S':
			{
				// Apply immediate spin counter-clockwise
				forcesBodies[1]->ApplyAngularImpulse(20, true);
				break;
			}
			default:
			{
				// Run default behaviour
				Test::Keyboard(key);
			}
		}
	}

	

	void createWalls()
	{
		// Add 4 walls
		// Body def
		b2BodyDef constSpeedBodyDef;
		constSpeedBodyDef.type = b2_staticBody;
		constSpeedBodyDef.position.Set(0, 0);
		b2Body* staticBody = m_world->CreateBody(&constSpeedBodyDef);

		// Shape def
		b2PolygonShape polyShape;
		polyShape.SetAsBox(1, 1);		// A 2x2 rect, half height/width remember

		// Fixture def
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &polyShape;
		fixtureDef.density = 1;

		polyShape.SetAsBox(20, 1, b2Vec2(0, 0), 0);			// Ground
		staticBody->CreateFixture(&fixtureDef);

		polyShape.SetAsBox(20, 1, b2Vec2(0, 40), 0);		// Roof
		staticBody->CreateFixture(&fixtureDef);

		polyShape.SetAsBox(1, 20, b2Vec2(-20, 20), 0);		// Left wall
		staticBody->CreateFixture(&fixtureDef);

		polyShape.SetAsBox(1, 20, b2Vec2(20, 20), 0);		// Right wall
		staticBody->CreateFixture(&fixtureDef);

		ms = MoveState::MS_STOP;
	}

	void constantSpeedTest()
	{
		// Body def
		b2BodyDef constSpeedBodyDef;
		constSpeedBodyDef.type = b2_dynamicBody;

		// Shape def
		b2PolygonShape polyShape;
		polyShape.SetAsBox(1, 1);		// A 2x2 rect, half height/width remember

		// Fixture def
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &polyShape;
		fixtureDef.density = 1;

		// Create dynamic body
		constSpeedBodyDef.position.Set(0, 10);
		constantSpeedBody = m_world->CreateBody(&constSpeedBodyDef);
		constantSpeedBody->CreateFixture(&fixtureDef);
	}

	void forcesImpulseTest()
	{
		// Bodies def
		b2BodyDef dynForcesBodyDef;
		dynForcesBodyDef.type = b2_dynamicBody;

		// Shape def
		b2PolygonShape polyShape;
		polyShape.SetAsBox(1, 1);		// A 2x2 rect, half height/width remember

		// Fixture def
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &polyShape;
		fixtureDef.density = 1;

		// Create identical bodies but in different positions
		for (int i = 0; i < 3; ++i)
		{
			dynForcesBodyDef.position.Set(-10 + (i * 10), 20);
			forcesBodies[i] = m_world->CreateBody(&dynForcesBodyDef);
			forcesBodies[i]->CreateFixture(&fixtureDef);
		}
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
		//polyFrictionFixDef.restitution = 1;

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
		edgeLineShape.m_vertex2.Set(15, 0);

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

	void checkForceOn()
	{
		if (forceOn)
		{
			// Apply gradual force upwards
			forcesBodies[0]->ApplyForce(b2Vec2(0, 50), forcesBodies[0]->GetWorldPoint(b2Vec2(1, 1)), true);
		}
	}

	void checkTorqueOn()
	{
		if (torqueOn)
		{
			// Apply gradual torque counter-clockwise
			forcesBodies[0]->ApplyTorque(20, true);
		}
	}

	void adjustGravity()
	{
		// Cancel gravity for one of the 3 bodies
		// This is achieved by simply applying a force equal and opposite to that of
		// the gravitational pull, this also MUST happen BEFORE the time step
		// or the simulation will get a chance to apply gravity for a fraction of a second
		// before cancelling it, resulting in a slow descent each step
		// Use this method before Box2D v2.1.2
		forcesBodies[1]->ApplyForce(forcesBodies[1]->GetMass() * -m_world->GetGravity(), forcesBodies[1]->GetWorldCenter(), true);

		// Use this method after Box2D v2.1.2
		// 0 will cancel gravity
		// -1 will reverse gravity
		forcesBodies[1]->SetGravityScale(0);
	}

	void move_DirectVelocity()
	{
		b2Vec2 velocity = constantSpeedBody->GetLinearVelocity();

		// Use this for instantaneous accell/decell
		// Setting velocity directly should not be done if wanting
		// to simulate a realisitc physics simulation, you should be using forces!
		/*switch (ms)
		{
			case MoveState::MS_LEFT:
			{
				velocity.x = -5;
				break;
			}
			case MoveState::MS_STOP:
			{
				velocity.x = 0;
				break;
			}
			case MoveState::MS_RIGHT:
			{
				velocity.x = 5;
				break;
			}
		}*/

		/*
		This will increase the velocity linearly by 0.1 per time step to a maximum of 5
		in the direction of travel - with the default testbed framerate of 60fps the body
		will take 50 frames or just under a second to reach top speed. When coming to a stop
		the speed is reduced to 98% of the previous frame's speed, which comes to about
		0.98^60 = a factor of about 0.3 per second. An advantage of this method is that these
		acceleration characteristics can be easily tuned.

		// http://www.iforce2d.net/b2dtut/constant-speed
		*/

		// Use this for a gradual accell/decell
		switch (ms)
		{
		case MoveState::MS_LEFT:  velocity.x = b2Max(velocity.x - 0.1f, -5.0f); break;
		case MoveState::MS_STOP: velocity.x *= 0.98; break;
		case MoveState::MS_RIGHT: velocity.x = b2Min(velocity.x + 0.1f, 5.0f); break;
		}

		constantSpeedBody->SetLinearVelocity(velocity);
	}

	void move_UsingForces()
	{
		b2Vec2 velocity = constantSpeedBody->GetLinearVelocity();
		float force = 0;
		float desiredVel = 0;
		float impulse = 0;

		// Use this for instantaneous accell/decell while remaining true
		// as a realistic physics simulation
		/*switch (ms)
		{
		case MoveState::MS_LEFT: desiredVel = -5; break;
		case MoveState::MS_STOP: desiredVel = 0; break;
		case MoveState::MS_RIGHT: desiredVel = 5; break;
		}*/

		// Use this also for a gradual accell/decell while remaining true
		// as a realistic physics simulation
		switch (ms)
		{
		case MoveState::MS_LEFT: desiredVel = b2Max(velocity.x - 0.1f, -5.0f); break;
		case MoveState::MS_STOP: desiredVel = velocity.x * 0.98f; break;
		case MoveState::MS_RIGHT: desiredVel = b2Min(velocity.x + 0.1f, 5.0f); break;
		}

		// Use this for a gradual accell/decell while remaining true
		// as a realistic physics simulation
		/*switch (ms)
		{
		case MoveState::MS_LEFT:  if (velocity.x > -5) force = -50; break;
		case MoveState::MS_STOP: force = velocity.x * -10; break;
		case MoveState::MS_RIGHT: if (velocity.x < 5) force = 50; break;
		}*/

		float velChange = desiredVel - velocity.x;
		force = constantSpeedBody->GetMass() * velChange / (1 / 60.0f);		// f = mv/t

		// Disregard the time factor as impulses already account for the length of the simulation timestep
		impulse = constantSpeedBody->GetMass() * velChange;

		//constantSpeedBody->ApplyForce(b2Vec2(force, 0), constantSpeedBody->GetWorldCenter(), true);
		constantSpeedBody->ApplyLinearImpulse(b2Vec2(impulse, 0), constantSpeedBody->GetWorldCenter(), true);
	}

	void getRotationalInfo()
	{
		float bodyAngle = dynRotationBody->GetAngle();
		b2Vec2 toTarget = clickedPoint - dynRotationBody->GetPosition();
		float desiredAngle = atan2f(-toTarget.x, toTarget.y);		// This is -x/y instead of typical tan function of y/x, this is so 0 deg is north, not east
		float totalRotation = desiredAngle - bodyAngle;
		float change = 1 * DEGTORAD;		// Allow 1 geree rotation per time step
		float newAngle = bodyAngle + b2Min(change, b2Max(-change, totalRotation));
		float nextAngle = bodyAngle + dynRotationBody->GetAngularVelocity() / 4.0f;	// Smaller numer, means faster tracking to clicked point direction
		totalRotation = desiredAngle - nextAngle;

		while (totalRotation < -180 * DEGTORAD)
		{
			totalRotation += 360 * DEGTORAD;
		}

		while (totalRotation > 180 * DEGTORAD)
		{
			totalRotation -= 360 * DEGTORAD;
		}

		// Setting transform and angle directly means the body is not participating in the physics sim correctly
		//dynRotationBody->SetTransform(dynRotationBody->GetPosition(), desiredAngle);

		// Apply rotation properly by applying torque
		dynRotationBody->ApplyTorque(totalRotation < 0 ? -10 : 10, true);
		//dynRotationBody->SetAngularVelocity(0);

		g_debugDraw.DrawString(5, m_textLine, "Body Angle: %.3f", bodyAngle * RADTODEG);
		m_textLine += 15;
		g_debugDraw.DrawString(5, m_textLine, "Target Angel: %.3f", desiredAngle * RADTODEG);
		m_textLine += 15;
	}

	void Step(Settings& settings)
	{
		// Needs to happen before timeStep
		//adjustGravity();

		// Run the default physics and rendering
		Test::Step(settings);

		g_debugDraw.DrawCircle(b2Vec2(clickedPoint.x, clickedPoint.y), 0.2, b2Color(0, 1.0, 0));

		//checkForceOn();
		//checkTorqueOn();
		
		//move_DirectVelocity();
		//move_UsingForces();

		// Show some test in the main screen
		g_debugDraw.DrawString(5, m_textLine, "This is the foo test");
		m_textLine += 15;

		//getBodyInfo();

		g_debugDraw.DrawString(5, m_textLine, "XPos: %.3f  YPos: %.3f Angle: %.3f", pos.x, pos.y, angle * RADTODEG);
		m_textLine += 15;
		g_debugDraw.DrawString(5, m_textLine, "Velocity: %.3f Angular Vel: %.3f", vel.x, vel.y, angularVel * RADTODEG);
		m_textLine += 15;

		getRotationalInfo();
	}

	static Test* Create()
	{
		return new FooTest;
	}
};

//#endif // !FOOTEST_H

static int testIndex = RegisterTest("MyTests", "Foo Test", FooTest::Create);