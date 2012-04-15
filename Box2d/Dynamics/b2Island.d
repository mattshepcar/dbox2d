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

module box2d.dynamics.b2Island;

import box2d.common.b2Math;
import box2d.common.b2Timer;
import box2d.dynamics.b2Body;
import box2d.dynamics.b2Contact;
import box2d.dynamics.b2ContactSolver;
import box2d.dynamics.b2Joint;
import box2d.dynamics.b2TimeStep;
import box2d.dynamics.b2WorldCallbacks;

/// This is an internal class.
class b2Island
{
public:
	this(int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity,
		 b2ContactListener listener)
	{
		m_bodies = new b2Body[bodyCapacity];
		m_contacts = new b2Contact[contactCapacity];
		m_joints = new b2Joint[jointCapacity];
		m_velocities = new b2Velocity[bodyCapacity];
		m_positions = new b2Position[bodyCapacity];
		m_listener = listener;
	}

	void Clear()
	{
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void Solve(ref b2Profile profile, b2TimeStep step, b2Vec2 gravity, bool allowSleep)
	{
		b2Timer timer;

		float32 h = step.dt;

		// Integrate velocities and apply damping. Initialize the b state.
		for (int32 i = 0; i < m_bodyCount; ++i)
		{
			b2Body b = m_bodies[i];

			b2Vec2 c = b.m_sweep.c;
			float32 a = b.m_sweep.a;
			b2Vec2 v = b.m_linearVelocity;
			float32 w = b.m_angularVelocity;

			// Store positions for continuous collision.
			b.m_sweep.c0 = b.m_sweep.c;
			b.m_sweep.a0 = b.m_sweep.a;

			if (b.m_type == b2BodyType.b2_dynamicBody)
			{
				// Integrate velocities.
				v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
				w += h * b.m_invI * b.m_torque;

				// Apply damping.
				// ODE: dv/dt + c * v = 0
				// Solution: v(t) = v0 * exp(-c * t)
				// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
				// v2 = exp(-c * dt) * v1
				// Taylor expansion:
				// v2 = (1.0f - c * dt) * v1
				v *= b2Clamp(1.0f - h * b.m_linearDamping, 0.0f, 1.0f);
				w *= b2Clamp(1.0f - h * b.m_angularDamping, 0.0f, 1.0f);
			}

			m_positions[i].c = c;
			m_positions[i].a = a;
			m_velocities[i].v = v;
			m_velocities[i].w = w;
		}

		timer.Reset();

		// Solver data
		b2SolverData solverData;
		solverData.step = step;
		solverData.positions = m_positions;
		solverData.velocities = m_velocities;

		// Initialize velocity constraints.
		b2ContactSolverDef contactSolverDef;
		contactSolverDef.step = step;
		contactSolverDef.contacts = m_contacts;
		contactSolverDef.count = m_contactCount;
		contactSolverDef.positions = m_positions;
		contactSolverDef.velocities = m_velocities;

		auto contactSolver = new b2ContactSolver(contactSolverDef);
		contactSolver.InitializeVelocityConstraints();

		if (step.warmStarting)
		{
			contactSolver.WarmStart();
		}

		for (int32 i = 0; i < m_jointCount; ++i)
		{
			m_joints[i].InitVelocityConstraints(solverData);
		}

		profile.solveInit = timer.GetMilliseconds();

		// Solve velocity constraints
		timer.Reset();
		for (int32 i = 0; i < step.velocityIterations; ++i)
		{
			for (int32 j = 0; j < m_jointCount; ++j)
			{
				m_joints[j].SolveVelocityConstraints(solverData);
			}

			contactSolver.SolveVelocityConstraints();
		}

		// Store impulses for warm starting
		contactSolver.StoreImpulses();
		profile.solveVelocity = timer.GetMilliseconds();

		// Integrate positions
		for (int32 i = 0; i < m_bodyCount; ++i)
		{
			b2Vec2 c = m_positions[i].c;
			float32 a = m_positions[i].a;
			b2Vec2 v = m_velocities[i].v;
			float32 w = m_velocities[i].w;

			// Check for large velocities
			b2Vec2 translation = h * v;
			if (b2Dot(translation, translation) > b2_maxTranslationSquared)
			{
				float32 ratio = b2_maxTranslation / translation.Length();
				v *= ratio;
			}

			float32 rotation = h * w;
			if (rotation * rotation > b2_maxRotationSquared)
			{
				float32 ratio = b2_maxRotation / b2Abs(rotation);
				w *= ratio;
			}

			// Integrate
			c += h * v;
			a += h * w;

			m_positions[i].c = c;
			m_positions[i].a = a;
			m_velocities[i].v = v;
			m_velocities[i].w = w;
		}

		// Solve position constraints
		timer.Reset();
		bool positionSolved = false;
		for (int32 i = 0; i < step.positionIterations; ++i)
		{
			bool contactsOkay = contactSolver.SolvePositionConstraints();

			bool jointsOkay = true;
			for (int32 j = 0; j < m_jointCount; ++j)
			{
				bool jointOkay = m_joints[j].SolvePositionConstraints(solverData);
				jointsOkay = jointsOkay && jointOkay;
			}

			if (contactsOkay && jointsOkay)
			{
				// Exit early if the position errors are small.
				positionSolved = true;
				break;
			}
		}

		// Copy state buffers back to the bodies
		for (int32 i = 0; i < m_bodyCount; ++i)
		{
			b2Body b = m_bodies[i];
			b.m_sweep.c = m_positions[i].c;
			b.m_sweep.a = m_positions[i].a;
			b.m_linearVelocity = m_velocities[i].v;
			b.m_angularVelocity = m_velocities[i].w;
			b.SynchronizeTransform();
		}

		profile.solvePosition = timer.GetMilliseconds();

		Report(contactSolver.m_velocityConstraints);

		if (allowSleep)
		{
			float32 minSleepTime = b2_maxFloat;

			const float32 linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
			const float32 angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

			for (int32 i = 0; i < m_bodyCount; ++i)
			{
				b2Body b = m_bodies[i];
				if (b.GetType() == b2BodyType.b2_staticBody)
				{
					continue;
				}

				if ((b.m_flags & b2Body.e_autoSleepFlag) == 0 ||
					b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
					b2Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr)
				{
					b.m_sleepTime = 0.0f;
					minSleepTime = 0.0f;
				}
				else
				{
					b.m_sleepTime += h;
					minSleepTime = b2Min(minSleepTime, b.m_sleepTime);
				}
			}

			if (minSleepTime >= b2_timeToSleep && positionSolved)
			{
				for (int32 i = 0; i < m_bodyCount; ++i)
				{
					b2Body b = m_bodies[i];
					b.SetAwake(false);
				}
			}
		}
	}

	void SolveTOI(b2TimeStep subStep, int32 toiIndexA, int32 toiIndexB)
	{
		assert(toiIndexA < m_bodyCount);
		assert(toiIndexB < m_bodyCount);

		// Initialize the b state.
		for (int32 i = 0; i < m_bodyCount; ++i)
		{
			b2Body b = m_bodies[i];
			m_positions[i].c = b.m_sweep.c;
			m_positions[i].a = b.m_sweep.a;
			m_velocities[i].v = b.m_linearVelocity;
			m_velocities[i].w = b.m_angularVelocity;
		}

		b2ContactSolverDef contactSolverDef;
		contactSolverDef.contacts = m_contacts;
		contactSolverDef.count = m_contactCount;
		contactSolverDef.step = subStep;
		contactSolverDef.positions = m_positions;
		contactSolverDef.velocities = m_velocities;
		auto contactSolver = new b2ContactSolver(contactSolverDef);

		// Solve position constraints.
		for (int32 i = 0; i < subStep.positionIterations; ++i)
		{
			bool contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
			if (contactsOkay)
			{
				break;
			}
		}

		version(none) {
		// Is the new position really safe?
		for (int32 i = 0; i < m_contactCount; ++i)
		{
			b2Contact c = m_contacts[i];
			b2Fixture fA = c.GetFixtureA();
			b2Fixture fB = c.GetFixtureB();

			b2Body bA = fA.GetBody();
			b2Body bB = fB.GetBody();

			int32 indexA = c.GetChildIndexA();
			int32 indexB = c.GetChildIndexB();

			b2DistanceInput input;
			input.proxyA.Set(fA.GetShape(), indexA);
			input.proxyB.Set(fB.GetShape(), indexB);
			input.transformA = bA.GetTransform();
			input.transformB = bB.GetTransform();
			input.useRadii = false;

			b2DistanceOutput output;
			b2SimplexCache cache;
			cache.count = 0;
			b2Distance(&output, &cache, &input);

			if (output.distance == 0 || cache.count == 3)
			{
				cache.count += 0;
			}
		}
		}

		// Leap of faith to new safe state.
		m_bodies[toiIndexA].m_sweep.c0 = m_positions[toiIndexA].c;
		m_bodies[toiIndexA].m_sweep.a0 = m_positions[toiIndexA].a;
		m_bodies[toiIndexB].m_sweep.c0 = m_positions[toiIndexB].c;
		m_bodies[toiIndexB].m_sweep.a0 = m_positions[toiIndexB].a;

		// No warm starting is needed for TOI events because warm
		// starting impulses were applied in the discrete solver.
		contactSolver.InitializeVelocityConstraints();

		// Solve velocity constraints.
		for (int32 i = 0; i < subStep.velocityIterations; ++i)
		{
			contactSolver.SolveVelocityConstraints();
		}

		// Don't store the TOI contact forces for warm starting
		// because they can be quite large.

		float32 h = subStep.dt;

		// Integrate positions
		for (int32 i = 0; i < m_bodyCount; ++i)
		{
			b2Vec2 c = m_positions[i].c;
			float32 a = m_positions[i].a;
			b2Vec2 v = m_velocities[i].v;
			float32 w = m_velocities[i].w;

			// Check for large velocities
			b2Vec2 translation = h * v;
			if (b2Dot(translation, translation) > b2_maxTranslationSquared)
			{
				float32 ratio = b2_maxTranslation / translation.Length();
				v *= ratio;
			}

			float32 rotation = h * w;
			if (rotation * rotation > b2_maxRotationSquared)
			{
				float32 ratio = b2_maxRotation / b2Abs(rotation);
				w *= ratio;
			}

			// Integrate
			c += h * v;
			a += h * w;

			m_positions[i].c = c;
			m_positions[i].a = a;
			m_velocities[i].v = v;
			m_velocities[i].w = w;

			// Sync bodies
			b2Body b = m_bodies[i];
			b.m_sweep.c = c;
			b.m_sweep.a = a;
			b.m_linearVelocity = v;
			b.m_angularVelocity = w;
			b.SynchronizeTransform();
		}

		Report(contactSolver.m_velocityConstraints);
	}

	void Add(b2Body b)
	{
		b.m_islandIndex = m_bodyCount;
		m_bodies[m_bodyCount++] = b;
	}

	void Add(b2Contact contact)
	{
		m_contacts[m_contactCount++] = contact;
	}

	void Add(b2Joint joint)
	{
		m_joints[m_jointCount++] = joint;
	}

	void Report(b2ContactVelocityConstraint[] constraints)
	{
		if (!m_listener)
		{
			return;
		}

		for (int32 i = 0; i < m_contactCount; ++i)
		{
			b2Contact c = m_contacts[i];

			const b2ContactVelocityConstraint* vc = &constraints[i];

			b2ContactImpulse impulse;
			impulse.count = vc.pointCount;
			for (int32 j = 0; j < vc.pointCount; ++j)
			{
				impulse.normalImpulses[j] = vc.points[j].normalImpulse;
				impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
			}

			m_listener.PostSolve(c, impulse);
		}
	}

	b2ContactListener m_listener;

	b2Body[] m_bodies;
	b2Contact[] m_contacts;
	b2Joint[] m_joints;

	b2Position[] m_positions;
	b2Velocity[] m_velocities;

	int32 m_bodyCount = 0;
	int32 m_jointCount = 0;
	int32 m_contactCount = 0;
}