/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

module box2d.dynamics.b2Joint;

import box2d.dynamics.b2DistanceJoint;
import box2d.dynamics.b2FrictionJoint;
import box2d.dynamics.b2GearJoint;
import box2d.dynamics.b2PrismaticJoint;
import box2d.dynamics.b2PulleyJoint;
import box2d.dynamics.b2RevoluteJoint;
import box2d.dynamics.b2MotorJoint;
import box2d.dynamics.b2MouseJoint;
import box2d.dynamics.b2RopeJoint;
import box2d.dynamics.b2WeldJoint;
import box2d.dynamics.b2WheelJoint;

import box2d.common.b2Settings;
import box2d.common.b2Math;
import box2d.dynamics.b2Body;
import box2d.dynamics.b2TimeStep;

enum b2JointType
{
	e_unknownJoint,
	e_revoluteJoint,
	e_prismaticJoint,
	e_distanceJoint,
	e_pulleyJoint,
	e_mouseJoint,
	e_gearJoint,
	e_wheelJoint,
    e_weldJoint,
	e_frictionJoint,
	e_ropeJoint,
	e_motorJoint
}

enum b2LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
}

struct b2Jacobian
{
	b2Vec2 linear;
	float32 angularA;
	float32 angularB;
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
struct b2JointEdge
{
	b2Body other;				///< provides quick access to the other body attached.
	b2Joint joint;				///< the joint
	b2JointEdge* prev = null;	///< the previous joint edge in the body's joint list
	b2JointEdge* next = null;	///< the next joint edge in the body's joint list
}

/// Joint definitions are used to construct joints.
class b2JointDef
{
	/// The joint type is set automatically for concrete joint types.
	b2JointType type = b2JointType.e_unknownJoint;

	/// Use this to attach application specific data to your joints.
	void* userData = null;

	/// The first attached body.
	b2Body bodyA = null;

	/// The second attached body.
	b2Body bodyB = null;

	/// Set this flag to true if the attached bodies should collide.
	bool collideConnected = false;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
class b2Joint
{
public:

	/// Get the type of the concrete joint.
	b2JointType GetType() const
	{
		return m_type;
	}

	/// Get the first body attached to this joint.
	b2Body GetBodyA()
	{
		return m_bodyA;
	}

	/// Get the second body attached to this joint.
	b2Body GetBodyB()
	{
		return m_bodyB;
	}

	/// Get the anchor point on bodyA in world coordinates.
	abstract b2Vec2 GetAnchorA() const;

	/// Get the anchor point on bodyB in world coordinates.
	abstract b2Vec2 GetAnchorB() const;

	/// Get the reaction force on bodyB at the joint anchor in Newtons.
	abstract b2Vec2 GetReactionForce(float32 inv_dt) const;

	/// Get the reaction torque on bodyB in N*m.
	abstract float32 GetReactionTorque(float32 inv_dt) const;

	/// Get the next joint the world joint list.
	inout(b2Joint) GetNext() inout
	{
		return m_next;
	}

	/// Get the user data pointer.
	void* GetUserData()
	{
		return m_userData;
	}

	/// Set the user data pointer.
	void SetUserData(void* data)
	{
		m_userData = data;
	}

	/// Short-cut function to determine if either body is inactive.
	bool IsActive() const
	{
		return m_bodyA.IsActive() && m_bodyB.IsActive();
	}

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	bool GetCollideConnected() const
	{
		return m_collideConnected;
	}

	/// Dump this joint to the log file.
	void Dump() { b2Log("// Dump is not supported for this joint type.\n"); }

	/// Shift the origin for any points stored in world coordinates.
	void ShiftOrigin(b2Vec2 newOrigin) {}

protected:
	static b2Joint Create(const b2JointDef def)
	{
		b2Joint joint;

		switch (def.type)
		{
			case b2JointType.e_distanceJoint:
				joint = new b2DistanceJoint(cast(b2DistanceJointDef)def);
				break;

			case b2JointType.e_mouseJoint:
				joint = new b2MouseJoint(cast(b2MouseJointDef)def);
				break;

			case b2JointType.e_prismaticJoint:
				joint = new b2PrismaticJoint(cast(b2PrismaticJointDef)def);
				break;

			case b2JointType.e_revoluteJoint:
				joint = new b2RevoluteJoint(cast(b2RevoluteJointDef)def);
				break;

			case b2JointType.e_pulleyJoint:
				joint = new b2PulleyJoint(cast(b2PulleyJointDef)def);
				break;

			case b2JointType.e_gearJoint:
				joint = new b2GearJoint(cast(b2GearJointDef)def);
				break;

			case b2JointType.e_wheelJoint:
		        joint = new b2WheelJoint(cast(b2WheelJointDef)def);
			    break;
			
			case b2JointType.e_weldJoint:
		        joint = new b2WeldJoint(cast(b2WeldJointDef)def);
			    break;
			
			case b2JointType.e_frictionJoint:
		        joint = new b2FrictionJoint(cast(b2FrictionJointDef) def);
			    break;
			
			case b2JointType.e_ropeJoint:
				joint = new b2RopeJoint(cast(b2RopeJointDef)def);
				break;

			case b2JointType.e_motorJoint:
				joint = new b2MotorJoint(cast(b2MotorJointDef)def);
				break;

			default:
				assert(false);
		}

		return joint;
	}

	this(b2JointDef def)
	{
		assert(def.bodyA != def.bodyB);

		m_type = def.type;
		m_bodyA = def.bodyA;
		m_bodyB = def.bodyB;
		m_index = 0;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		m_userData = def.userData;
	}

public:
	abstract void InitVelocityConstraints(b2SolverData data);
	abstract void SolveVelocityConstraints(b2SolverData data);

	// This returns true if the position errors are within tolerance.
	abstract bool SolvePositionConstraints(b2SolverData data);

private:
	b2JointType m_type;

package:
	b2Joint m_prev;
	b2Joint m_next;
	b2JointEdge m_edgeA;
	b2JointEdge m_edgeB;
	b2Body m_bodyA;
	b2Body m_bodyB;

	int32 m_index;

	bool m_islandFlag;
	bool m_collideConnected;

	void* m_userData;
};
