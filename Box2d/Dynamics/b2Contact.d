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

module box2d.dynamics.b2Contact;

import box2d.common.b2Math;
import box2d.collision.b2Collision;
import box2d.collision.shapes.b2Shape;
import box2d.dynamics.b2Body;
import box2d.dynamics.b2Fixture;
import box2d.dynamics.b2WorldCallbacks;
import box2d.dynamics.b2CircleContact;
import box2d.dynamics.b2PolygonContact;
import box2d.dynamics.b2PolygonAndCircleContact;
import box2d.dynamics.b2EdgeAndPolygonContact;
import box2d.dynamics.b2EdgeAndCircleContact;
import box2d.dynamics.b2ChainAndPolygonContact;
import box2d.dynamics.b2ChainAndCircleContact;

/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
float32 b2MixFriction(float32 friction1, float32 friction2)
{
	return sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
float32 b2MixRestitution(float32 restitution1, float32 restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

alias b2Contact function(b2Fixture fixtureA, int32 indexA,
						 b2Fixture fixtureB, int32 indexB) b2ContactCreateFcn;
alias void function(b2Contact contact) b2ContactDestroyFcn;

struct b2ContactRegister
{
	b2ContactCreateFcn createFcn;
	bool primary;
};

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct b2ContactEdge
{
	b2Body other;			///< provides quick access to the other body attached.
	b2Contact contact;		///< the contact
	b2ContactEdge* prev;	///< the previous contact edge in the body's contact list
	b2ContactEdge* next;	///< the next contact edge in the body's contact list
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class b2Contact
{
public:

	/// Get the contact manifold. Do not modify the manifold unless you understand the
	/// internals of Box2D.
	inout(b2Manifold*) GetManifold() inout
	{
	    return &m_manifold;
	}

	/// Get the world manifold.
	void GetWorldManifold(ref b2WorldManifold worldManifold) const
	{
		const b2Body bodyA = m_fixtureA.GetBody();
		const b2Body bodyB = m_fixtureB.GetBody();
		const b2Shape shapeA = m_fixtureA.GetShape();
		const b2Shape shapeB = m_fixtureB.GetShape();

		worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}

	/// Is this contact touching?
	bool IsTouching() const
	{
		return (m_flags & e_touchingFlag) == e_touchingFlag;
	}

	/// Enable/disable this contact. This can be used inside the pre-solve
	/// contact listener. The contact is only disabled for the current
	/// time step (or sub-step in continuous collisions).
	void SetEnabled(bool flag)
	{
		if (flag)
		{
			m_flags |= e_enabledFlag;
		}
		else
		{
			m_flags &= ~e_enabledFlag;
		}
	}

	/// Has this contact been disabled?
	bool IsEnabled() const
	{
		return (m_flags & e_enabledFlag) == e_enabledFlag;
	}

	/// Get the next contact in the world's contact list.
	inout(b2Contact) GetNext() inout
	{
	    return m_next;
	}

	/// Get fixture A in this contact.
	inout(b2Fixture) GetFixtureA() inout
	{
	    return m_fixtureA;
	}

	/// Get the child primitive index for fixture A.
	int32 GetChildIndexA() const
	{
		return m_indexA;
	}

	/// Get fixture B in this contact.
	inout(b2Fixture) GetFixtureB() inout
	{
	    return m_fixtureB;
	}

	/// Get the child primitive index for fixture B.
	int32 GetChildIndexB() const
	{
		return m_indexB;
	}

	/// Override the default friction mixture. You can call this in b2ContactListener.PreSolve.
	/// This value persists until set or reset.
	void SetFriction(float32 friction)
	{
		m_friction = friction;
	}

	/// Get the friction.
	float32 GetFriction() const
	{
		return m_friction;
	}

	/// Reset the friction mixture to the default value.
	void ResetFriction()
	{
		m_friction = b2MixFriction(m_fixtureA.GetFriction(), m_fixtureB.GetFriction());
	}

	/// Override the default restitution mixture. You can call this in b2ContactListener.PreSolve.
	/// The value persists until you set or reset.
	void SetRestitution(float32 restitution)
	{
		m_restitution = restitution;
	}

	/// Get the restitution.
	float32 GetRestitution() const
	{
		return m_restitution;
	}

	/// Reset the restitution to the default value.
	void ResetRestitution()
	{
		m_restitution = b2MixRestitution(m_fixtureA.GetRestitution(), m_fixtureB.GetRestitution());
	}

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	void SetTangentSpeed(float32 speed)
	{
		m_tangentSpeed = speed;
	}

	/// Get the desired tangent speed. In meters per second.
	float32 GetTangentSpeed() const
	{
		return m_tangentSpeed;
	}

	/// Evaluate this contact with your own manifold and transforms.
	abstract void Evaluate(ref b2Manifold manifold, b2Transform xfA, b2Transform xfB);

protected:
	// Flags stored in m_flags
	enum
	{
		// Used when crawling contact graph when forming islands.
		e_islandFlag		= 0x0001,

        // Set when the shapes are touching.
		e_touchingFlag		= 0x0002,

		// This contact can be disabled (by user)
		e_enabledFlag		= 0x0004,

		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag		= 0x0008,

		// This bullet contact had a TOI event
		e_bulletHitFlag		= 0x0010,

		// This contact has a valid TOI in m_toi
		e_toiFlag			= 0x0020
	};

	/// Flag this contact for filtering. Filtering will occur the next time step.
	package void FlagForFiltering()
	{
		m_flags |= e_filterFlag;
	}

	static void AddType(b2ContactCreateFcn createFcn, 
						b2Shape.Type type1, b2Shape.Type type2)
	{
		assert(0 <= type1 && type1 < b2Shape.Type.e_typeCount);
		assert(0 <= type2 && type2 < b2Shape.Type.e_typeCount);

		s_registers[type1][type2].createFcn = createFcn;
		s_registers[type1][type2].primary = true;

		if (type1 != type2)
		{
			s_registers[type2][type1].createFcn = createFcn;
			s_registers[type2][type1].primary = false;
		}
	}
	static void InitializeRegisters()
	{
		AddType(&b2CircleContact.Create,			b2Shape.Type.e_circle,	b2Shape.Type.e_circle);
		AddType(&b2PolygonAndCircleContact.Create,	b2Shape.Type.e_polygon, b2Shape.Type.e_circle);
		AddType(&b2PolygonContact.Create,			b2Shape.Type.e_polygon, b2Shape.Type.e_polygon);
		AddType(&b2EdgeAndCircleContact.Create,		b2Shape.Type.e_edge,	b2Shape.Type.e_circle);
		AddType(&b2EdgeAndPolygonContact.Create,	b2Shape.Type.e_edge,	b2Shape.Type.e_polygon);
		AddType(&b2ChainAndCircleContact.Create,	b2Shape.Type.e_chain,	b2Shape.Type.e_circle);
		AddType(&b2ChainAndPolygonContact.Create,	b2Shape.Type.e_chain,	b2Shape.Type.e_polygon);
	}

	static b2Contact Create(b2Fixture fixtureA, int32 indexA, b2Fixture fixtureB, int32 indexB)
	{
		if (s_initialized == false)
		{
			InitializeRegisters();
			s_initialized = true;
		}

		b2Shape.Type type1 = fixtureA.GetType();
		b2Shape.Type type2 = fixtureB.GetType();

		assert(0 <= type1 && type1 < b2Shape.Type.e_typeCount);
		assert(0 <= type2 && type2 < b2Shape.Type.e_typeCount);

		b2ContactCreateFcn createFcn = s_registers[type1][type2].createFcn;
		if (createFcn)
		{
			if (s_registers[type1][type2].primary)
			{
				return createFcn(fixtureA, indexA, fixtureB, indexB);
			}
			else
			{
				return createFcn(fixtureB, indexB, fixtureA, indexA);
			}
		}
		else
		{
			return null;
		}
	}
	static void Destroy(b2Contact contact)
	{
		assert(s_initialized == true);

		b2Fixture fixtureA = contact.m_fixtureA;
		b2Fixture fixtureB = contact.m_fixtureB;

		if (contact.m_manifold.pointCount > 0 &&
			fixtureA.IsSensor() == false &&
			fixtureB.IsSensor() == false)
		{
			fixtureA.GetBody().SetAwake(true);
			fixtureB.GetBody().SetAwake(true);
		}

		b2Shape.Type typeA = fixtureA.GetType();
		b2Shape.Type typeB = fixtureB.GetType();

		assert(0 <= typeA && typeB < b2Shape.Type.e_typeCount);
		assert(0 <= typeA && typeB < b2Shape.Type.e_typeCount);
	}

	this(b2Fixture fA, int32 indexA, b2Fixture fB, int32 indexB)
	{
		m_flags = e_enabledFlag;
		m_fixtureA = fA;
		m_fixtureB = fB;
		m_indexA = indexA;
		m_indexB = indexB;

		m_manifold.pointCount = 0;
		m_toiCount = 0;

		m_friction = b2MixFriction(m_fixtureA.GetFriction(), m_fixtureB.GetFriction());
		m_restitution = b2MixRestitution(m_fixtureA.GetRestitution(), m_fixtureB.GetRestitution());

		m_tangentSpeed = 0.0f;
	}

	package void Update(b2ContactListener listener)
	{
		b2Manifold oldManifold = m_manifold;

		// Re-enable this contact.
		m_flags |= e_enabledFlag;

		bool touching = false;
		bool wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;

		bool sensorA = m_fixtureA.IsSensor();
		bool sensorB = m_fixtureB.IsSensor();
		bool sensor = sensorA || sensorB;

		b2Body bodyA = m_fixtureA.GetBody();
		b2Body bodyB = m_fixtureB.GetBody();
		b2Transform xfA = bodyA.GetTransform();
		b2Transform xfB = bodyB.GetTransform();

		// Is this contact a sensor?
		if (sensor)
		{
			const b2Shape shapeA = m_fixtureA.GetShape();
			const b2Shape shapeB = m_fixtureB.GetShape();
			touching = b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

			// Sensors don't generate manifolds.
			m_manifold.pointCount = 0;
		}
		else
		{
			Evaluate(m_manifold, xfA, xfB);
			touching = m_manifold.pointCount > 0;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (int32 i = 0; i < m_manifold.pointCount; ++i)
			{
				b2ManifoldPoint* mp2 = &m_manifold.points[i];
				mp2.normalImpulse = 0.0f;
				mp2.tangentImpulse = 0.0f;
				b2ContactID id2 = mp2.id;

				for (int32 j = 0; j < oldManifold.pointCount; ++j)
				{
					b2ManifoldPoint* mp1 = &oldManifold.points[j];

					if (mp1.id.key == id2.key)
					{
						mp2.normalImpulse = mp1.normalImpulse;
						mp2.tangentImpulse = mp1.tangentImpulse;
						break;
					}
				}
			}

			if (touching != wasTouching)
			{
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}

		if (touching)
		{
			m_flags |= e_touchingFlag;
		}
		else
		{
			m_flags &= ~e_touchingFlag;
		}

		if (wasTouching == false && touching == true && listener)
		{
			listener.BeginContact(this);
		}

		if (wasTouching == true && touching == false && listener)
		{
			listener.EndContact(this);
		}

		if (sensor == false && touching && listener)
		{
			listener.PreSolve(this, oldManifold);
		}
	}

	static b2ContactRegister[b2Shape.Type.e_typeCount][b2Shape.Type.e_typeCount] s_registers;
	static bool s_initialized;

	package uint32 m_flags;

	// World pool and list pointers.
	package b2Contact m_prev;
	package b2Contact m_next;

	// Nodes for connecting bodies.
	package b2ContactEdge m_nodeA;
	package b2ContactEdge m_nodeB;

	b2Fixture m_fixtureA;
	b2Fixture m_fixtureB;

	int32 m_indexA;
	int32 m_indexB;

	b2Manifold m_manifold;

	package int32 m_toiCount;
	package float32 m_toi;

	float32 m_friction;
	float32 m_restitution;

	float32 m_tangentSpeed;
}
