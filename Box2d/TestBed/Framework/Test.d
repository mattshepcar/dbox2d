/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

module box2d.testbed.framework.test;
import box2d.common.b2Math;
import box2d.common.b2Draw;
import box2d.collision.b2Collision;
import box2d.collision.shapes.b2CircleShape;
import box2d.dynamics.b2WorldCallbacks;
import box2d.dynamics.b2Fixture;
import box2d.dynamics.b2Joint;
import box2d.dynamics.b2MouseJoint;
import box2d.dynamics.b2Contact;
import box2d.dynamics.b2World;
import box2d.dynamics.b2Body;
import box2d.dynamics.b2TimeStep;
import box2d.testbed.framework.render;
import std.random;

alias Test function() TestCreateFcn;

/// Random number in range [-1,1]
float32 RandomFloat()
{
	return uniform!("[]")(-1.0f, 1.0f);
}

/// Random floating point number in range [lo, hi]
float32 RandomFloat(float32 lo, float32 hi)
{
	return uniform!("[]")(lo, hi);
}

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
	b2Vec2 viewCenter = {0.0f, 20.0f};
	float32 hz = 60.0f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;
	int32 drawShapes = 1;
	int32 drawJoints = 1;
	int32 drawAABBs = 0;
	int32 drawContactPoints = 0;
	int32 drawContactNormals = 0;
	int32 drawContactImpulse = 0;
	int32 drawFrictionImpulse = 0;
	int32 drawCOMs = 0;
	int32 drawStats = 0;
	int32 drawProfile = 0;
	int32 enableWarmStarting = 1;
	int32 enableContinuous = 0;
	int32 enableSubStepping = 0;
	int32 enableSleep = 1;
	int32 pause = 0;
	int32 singleStep = 0;
}

struct TestEntry
{
	string name;
	TestCreateFcn createFcn;
};

TestEntry g_testEntries[];

// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class DestructionListener : b2DestructionListener
{
public:
	override void SayGoodbye(b2Fixture fixture) 
	{
	}
	override void SayGoodbye(b2Joint joint)
	{
		if (test.m_mouseJoint == joint)
		{
			test.m_mouseJoint = null;
		}
		else
		{
			test.JointDestroyed(joint);
		}
	}

	Test test;
}

const int32 k_maxContactPoints = 2048;

struct ContactPoint
{
	b2Fixture fixtureA;
	b2Fixture fixtureB;
	b2Vec2 normal;
	b2Vec2 position;
	b2PointState state;
	float32 normalImpulse;
	float32 tangentImpulse;
}

class Test : b2ContactListener
{
public:
	this()
	{
		b2Vec2 gravity;
		gravity.Set(0.0f, -10.0f);
		m_world = new b2World(gravity);
		m_textLine = 30;

		m_destructionListener = new DestructionListener;
		m_destructionListener.test = this;
		m_world.SetDestructionListener(m_destructionListener);
		m_world.SetContactListener(this);
		m_debugDraw = new DebugDraw;
		m_world.SetDebugDraw(m_debugDraw);
		m_bombSpawning = false;
		m_stepCount = 0;

		b2BodyDef bodyDef;
		m_groundBody = m_world.CreateBody(bodyDef);
	}

	void SetTextLine(int32 line) { m_textLine = line; }
    void DrawTitle(int x, int y, string s)
	{
		m_debugDraw.DrawString(x, y, s);
	}
	void Step(Settings settings)
	{
		float32 timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

		if (settings.pause)
		{
			if (settings.singleStep)
			{
				settings.singleStep = 0;
			}
			else
			{
				timeStep = 0.0f;
			}

			m_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
			m_textLine += 15;
		}

		uint32 flags = 0;
		flags += settings.drawShapes * b2Draw.e_shapeBit;
		flags += settings.drawJoints * b2Draw.e_jointBit;
		flags += settings.drawAABBs * b2Draw.e_aabbBit;
		flags += settings.drawCOMs * b2Draw.e_centerOfMassBit;
		m_debugDraw.SetFlags(flags);

		m_world.SetAllowSleeping(settings.enableSleep > 0);
		m_world.SetWarmStarting(settings.enableWarmStarting > 0);
		m_world.SetContinuousPhysics(settings.enableContinuous > 0);
		m_world.SetSubStepping(settings.enableSubStepping > 0);

		m_pointCount = 0;

		m_world.Step(timeStep, settings.velocityIterations, settings.positionIterations);

		m_world.DrawDebugData();

		if (timeStep > 0.0f)
		{
			++m_stepCount;
		}

		if (settings.drawStats)
		{
			int32 bodyCount = m_world.GetBodyCount();
			int32 contactCount = m_world.GetContactCount();
			int32 jointCount = m_world.GetJointCount();
			m_debugDraw.DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount);
			m_textLine += 15;

			int32 proxyCount = m_world.GetProxyCount();
			int32 height = m_world.GetTreeHeight();
			int32 balance = m_world.GetTreeBalance();
			float32 quality = m_world.GetTreeQuality();
			m_debugDraw.DrawString(5, m_textLine, "proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
			m_textLine += 15;
		}

		// Track maximum profile times
		{
			b2Profile p = m_world.GetProfile();
			m_maxProfile.step = b2Max(m_maxProfile.step, p.step);
			m_maxProfile.collide = b2Max(m_maxProfile.collide, p.collide);
			m_maxProfile.solve = b2Max(m_maxProfile.solve, p.solve);
			m_maxProfile.solveInit = b2Max(m_maxProfile.solveInit, p.solveInit);
			m_maxProfile.solveVelocity = b2Max(m_maxProfile.solveVelocity, p.solveVelocity);
			m_maxProfile.solvePosition = b2Max(m_maxProfile.solvePosition, p.solvePosition);
			m_maxProfile.solveTOI = b2Max(m_maxProfile.solveTOI, p.solveTOI);
			m_maxProfile.broadphase = b2Max(m_maxProfile.broadphase, p.broadphase);

			m_totalProfile.step += p.step;
			m_totalProfile.collide += p.collide;
			m_totalProfile.solve += p.solve;
			m_totalProfile.solveInit += p.solveInit;
			m_totalProfile.solveVelocity += p.solveVelocity;
			m_totalProfile.solvePosition += p.solvePosition;
			m_totalProfile.solveTOI += p.solveTOI;
			m_totalProfile.broadphase += p.broadphase;
		}

		if (settings.drawProfile)
		{
			const b2Profile p = m_world.GetProfile();

			b2Profile aveProfile;
			if (m_stepCount > 0)
			{
				float32 scale = 1.0f / m_stepCount;
				aveProfile.step = scale * m_totalProfile.step;
				aveProfile.collide = scale * m_totalProfile.collide;
				aveProfile.solve = scale * m_totalProfile.solve;
				aveProfile.solveInit = scale * m_totalProfile.solveInit;
				aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
				aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
				aveProfile.solveTOI = scale * m_totalProfile.solveTOI;
				aveProfile.broadphase = scale * m_totalProfile.broadphase;
			}

			m_debugDraw.DrawString(5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase);
			m_textLine += 15;
		}

		if (m_mouseJoint)
		{
			b2Vec2 p1 = m_mouseJoint.GetAnchorB();
			b2Vec2 p2 = m_mouseJoint.GetTarget();

			b2Color c;
			c.Set(0.0f, 1.0f, 0.0f);
			m_debugDraw.DrawPoint(p1, 4.0f, c);
			m_debugDraw.DrawPoint(p2, 4.0f, c);

			c.Set(0.8f, 0.8f, 0.8f);
			m_debugDraw.DrawSegment(p1, p2, c);
		}

		if (m_bombSpawning)
		{
			b2Color c;
			c.Set(0.0f, 0.0f, 1.0f);
			m_debugDraw.DrawPoint(m_bombSpawnPoint, 4.0f, c);

			c.Set(0.8f, 0.8f, 0.8f);
			m_debugDraw.DrawSegment(m_mouseWorld, m_bombSpawnPoint, c);
		}

		if (settings.drawContactPoints)
		{
			const float32 k_impulseScale = 0.1f;
			const float32 k_axisScale = 0.3f;

			for (int32 i = 0; i < m_pointCount; ++i)
			{
				ContactPoint* point = &m_points[i];

				if (point.state == b2PointState.b2_addState)
				{
					// Add
					m_debugDraw.DrawPoint(point.position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
				}
				else if (point.state == b2PointState.b2_persistState)
				{
					// Persist
					m_debugDraw.DrawPoint(point.position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
				}

				if (settings.drawContactNormals == 1)
				{
					b2Vec2 p1 = point.position;
					b2Vec2 p2 = p1 + k_axisScale * point.normal;
					m_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
				}
				else if (settings.drawContactImpulse == 1)
				{
					b2Vec2 p1 = point.position;
					b2Vec2 p2 = p1 + k_impulseScale * point.normalImpulse * point.normal;
					m_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
				}

				if (settings.drawFrictionImpulse == 1)
				{
					b2Vec2 tangent = b2Cross(point.normal, 1.0f);
					b2Vec2 p1 = point.position;
					b2Vec2 p2 = p1 + k_impulseScale * point.tangentImpulse * tangent;
					m_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
				}
			}
		}
	}
	void Keyboard(ubyte key) {  }
	void KeyboardUp(ubyte key) {  }
	void ShiftMouseDown(b2Vec2 p)
	{
		m_mouseWorld = p;

		if (m_mouseJoint)
		{
			return;
		}

		SpawnBomb(p);
	}
	void MouseDown(b2Vec2 p)
	{
		m_mouseWorld = p;

		if (m_mouseJoint)
		{
			return;
		}

		// Make a small box.
		b2AABB aabb;
		b2Vec2 d;
		d.Set(0.001f, 0.001f);
		aabb.lowerBound = p - d;
		aabb.upperBound = p + d;

		// Query the world for overlapping shapes.
		QueryCallback callback = new QueryCallback(p);
		m_world.QueryAABB(callback, aabb);

		if (callback.m_fixture)
		{
			b2Body b = callback.m_fixture.GetBody();
			auto md = new b2MouseJointDef;
			md.bodyA = m_groundBody;
			md.bodyB = b;
			md.target = p;
			md.maxForce = 1000.0f * b.GetMass();
			m_mouseJoint = cast(b2MouseJoint)m_world.CreateJoint(md);
			b.SetAwake(true);
		}
	}
	void MouseUp(b2Vec2 p)
	{
		if (m_mouseJoint)
		{
			m_world.DestroyJoint(m_mouseJoint);
			m_mouseJoint = null;
		}

		if (m_bombSpawning)
		{
			CompleteBombSpawn(p);
		}
	}
	void MouseMove(b2Vec2 p)
	{
		m_mouseWorld = p;

		if (m_mouseJoint)
		{
			m_mouseJoint.SetTarget(p);
		}
	}
	void LaunchBomb()
	{
		b2Vec2 p = {RandomFloat(-15.0f, 15.0f), 30.0f};
		b2Vec2 v = -5.0f * p;
		LaunchBomb(p, v);
	}
	void LaunchBomb(b2Vec2 position, b2Vec2 velocity)
	{
		if (m_bomb)
		{
			m_world.DestroyBody(m_bomb);
			m_bomb = null;
		}

		b2BodyDef bd;
		bd.type = b2BodyType.b2_dynamicBody;
		bd.position = position;
		bd.bullet = true;
		m_bomb = m_world.CreateBody(bd);
		m_bomb.SetLinearVelocity(velocity);

		b2CircleShape circle = new b2CircleShape;
		circle.m_radius = 0.3f;

		b2FixtureDef fd;
		fd.shape = circle;
		fd.density = 20.0f;
		fd.restitution = 0.0f;

		b2Vec2 minV = position - b2Vec2(0.3f,0.3f);
		b2Vec2 maxV = position + b2Vec2(0.3f,0.3f);

		b2AABB aabb;
		aabb.lowerBound = minV;
		aabb.upperBound = maxV;

		m_bomb.CreateFixture(fd);
	}
	
	void SpawnBomb(b2Vec2 worldPt)
	{
		m_bombSpawnPoint = worldPt;
		m_bombSpawning = true;
	}
	void CompleteBombSpawn(b2Vec2 p)
	{
		if (m_bombSpawning == false)
		{
			return;
		}

		const float multiplier = 30.0f;
		b2Vec2 vel = m_bombSpawnPoint - p;
		vel *= multiplier;
		LaunchBomb(m_bombSpawnPoint,vel);
		m_bombSpawning = false;
	}

	// Let derived tests know that a joint was destroyed.
	void JointDestroyed(b2Joint joint) { }

	// Callbacks for derived classes.
	override void BeginContact(b2Contact contact) {}
	override void EndContact(b2Contact contact) {}
	override void PreSolve(b2Contact contact, b2Manifold oldManifold)
	{
		const b2Manifold* manifold = contact.GetManifold();

		if (manifold.pointCount == 0)
		{
			return;
		}

		b2Fixture fixtureA = contact.GetFixtureA();
		b2Fixture fixtureB = contact.GetFixtureB();

		b2PointState[b2_maxManifoldPoints] state1;
		b2PointState[b2_maxManifoldPoints] state2;
		b2GetPointStates(state1, state2, oldManifold, *manifold);

		b2WorldManifold worldManifold;
		contact.GetWorldManifold(worldManifold);

		for (int32 i = 0; i < manifold.pointCount && m_pointCount < k_maxContactPoints; ++i)
		{
			ContactPoint* cp = &m_points[m_pointCount];
			cp.fixtureA = fixtureA;
			cp.fixtureB = fixtureB;
			cp.position = worldManifold.points[i];
			cp.normal = worldManifold.normal;
			cp.state = state2[i];
			cp.normalImpulse = manifold.points[i].normalImpulse;
			cp.tangentImpulse = manifold.points[i].tangentImpulse;
			++m_pointCount;
		}
	}
	override void PostSolve(b2Contact contact, b2ContactImpulse impulse)
	{
	}

	void ShiftOrigin(b2Vec2 newOrigin)
	{
		m_world.ShiftOrigin(newOrigin);
	}

protected:
	b2Body m_groundBody;
	b2AABB m_worldAABB;
	ContactPoint[k_maxContactPoints] m_points;
	int32 m_pointCount;
	DestructionListener m_destructionListener;
	DebugDraw m_debugDraw;
	int32 m_textLine;
	b2World m_world;
	b2Body m_bomb;
	b2MouseJoint m_mouseJoint;
	b2Vec2 m_bombSpawnPoint;
	bool m_bombSpawning;
	b2Vec2 m_mouseWorld;
	int32 m_stepCount;

	b2Profile m_maxProfile;
	b2Profile m_totalProfile;
}

private
{
	class QueryCallback : b2QueryCallback
	{
	public:
		this(b2Vec2 point)
		{
			m_point = point;			
		}

		override bool ReportFixture(b2Fixture fixture)
		{
			b2Body b = fixture.GetBody();
			if (b.GetType() == b2BodyType.b2_dynamicBody)
			{
				bool inside = fixture.TestPoint(m_point);
				if (inside)
				{
					m_fixture = fixture;

					// We are done, terminate the query.
					return false;
				}
			}

			// Continue the query.
			return true;
		}

		b2Vec2 m_point;
		b2Fixture m_fixture;
	}
}