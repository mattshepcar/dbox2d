/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

module box2d.dynamics.b2GearJoint;

import box2d.common.b2Math;
import box2d.dynamics.b2Body;
import box2d.dynamics.b2Joint;
import box2d.dynamics.b2RevoluteJoint;
import box2d.dynamics.b2PrismaticJoint;
import box2d.dynamics.b2TimeStep;

/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
class b2GearJointDef : b2JointDef
{
	this()
	{
		type = b2JointType.e_gearJoint;
	}

	/// The first revolute/prismatic joint attached to the gear joint.
	b2Joint joint1;

	/// The second revolute/prismatic joint attached to the gear joint.
	b2Joint joint2;

	/// The gear ratio.
	/// @see b2GearJoint for explanation.
	float32 ratio = 1.0f;
};

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning You have to manually destroy the gear joint if joint1 or joint2
/// is destroyed.
class b2GearJoint : b2Joint
{
public:
	override b2Vec2 GetAnchorA() const
	{
		return m_bodyA.GetWorldPoint(m_localAnchorA);
	}
	override b2Vec2 GetAnchorB() const
	{
		return m_bodyB.GetWorldPoint(m_localAnchorB);
	}

	override b2Vec2 GetReactionForce(float32 inv_dt) const
	{
		b2Vec2 P = m_impulse * m_JvAC;
		return inv_dt * P;
	}
	override float32 GetReactionTorque(float32 inv_dt) const
	{
		float32 L = m_impulse * m_JwA;
		return inv_dt * L;
	}

	/// Get the first joint.
	b2Joint GetJoint1() { return m_joint1; }

	/// Get the second joint.
	b2Joint GetJoint2() { return m_joint2; }

	/// Set/Get the gear ratio.
	void SetRatio(float32 ratio)
	{
		assert(b2IsValid(ratio));
		m_ratio = ratio;
	}
	float32 GetRatio() const
	{
		return m_ratio;
	}

	/// Dump joint to dmLog
	override void Dump()
	{
		int32 indexA = m_bodyA.m_islandIndex;
		int32 indexB = m_bodyB.m_islandIndex;

		int32 index1 = m_joint1.m_index;
		int32 index2 = m_joint2.m_index;

		b2Log("  b2GearJointDef jd;\n");
		b2Log("  jd.bodyA = bodies[%d];\n", indexA);
		b2Log("  jd.bodyB = bodies[%d];\n", indexB);
		b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
		b2Log("  jd.joint1 = joints[%d];\n", index1);
		b2Log("  jd.joint2 = joints[%d];\n", index2);
		b2Log("  jd.ratio = %.15lef;\n", m_ratio);
		b2Log("  joints[%d] = m_world.CreateJoint(&jd);\n", m_index);
	}

protected:
	package this(b2GearJointDef def)
	{
		super(def);

		m_joint1 = def.joint1;
		m_joint2 = def.joint2;

		m_typeA = m_joint1.GetType();
		m_typeB = m_joint2.GetType();

		assert(m_typeA == b2JointType.e_revoluteJoint || m_typeA == b2JointType.e_prismaticJoint);
		assert(m_typeB == b2JointType.e_revoluteJoint || m_typeB == b2JointType.e_prismaticJoint);

		float32 coordinateA, coordinateB;

		// TODO_ERIN there might be some problem with the joint edges in b2Joint.

		m_bodyC = m_joint1.GetBodyA();
		m_bodyA = m_joint1.GetBodyB();

		// Get geometry of joint1
		b2Transform xfA = m_bodyA.m_xf;
		float32 aA = m_bodyA.m_sweep.a;
		b2Transform xfC = m_bodyC.m_xf;
		float32 aC = m_bodyC.m_sweep.a;

		if (m_typeA == b2JointType.e_revoluteJoint)
		{
			b2RevoluteJoint revolute = cast(b2RevoluteJoint)def.joint1;
			m_localAnchorC = revolute.GetLocalAnchorA();
			m_localAnchorA = revolute.GetLocalAnchorB();
			m_referenceAngleA = revolute.GetReferenceAngle();
			m_localAxisC.SetZero();

			coordinateA = aA - aC - m_referenceAngleA;
		}
		else
		{
			b2PrismaticJoint prismatic = cast(b2PrismaticJoint)def.joint1;
			m_localAnchorC = prismatic.GetLocalAnchorA();
			m_localAnchorA = prismatic.GetLocalAnchorB();
			m_referenceAngleA = prismatic.GetReferenceAngle();
			m_localAxisC = prismatic.GetLocalAxisA();

			b2Vec2 pC = m_localAnchorC;
			b2Vec2 pA = b2MulT(xfC.q, b2Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
			coordinateA = b2Dot(pA - pC, m_localAxisC);
		}

		m_bodyD = m_joint2.GetBodyA();
		m_bodyB = m_joint2.GetBodyB();

		// Get geometry of joint2
		b2Transform xfB = m_bodyB.m_xf;
		float32 aB = m_bodyB.m_sweep.a;
		b2Transform xfD = m_bodyD.m_xf;
		float32 aD = m_bodyD.m_sweep.a;

		if (m_typeB == b2JointType.e_revoluteJoint)
		{
			b2RevoluteJoint revolute = cast(b2RevoluteJoint)def.joint2;
			m_localAnchorD = revolute.GetLocalAnchorA();
			m_localAnchorB = revolute.GetLocalAnchorB();
			m_referenceAngleB = revolute.GetReferenceAngle();
			m_localAxisD.SetZero();

			coordinateB = aB - aD - m_referenceAngleB;
		}
		else
		{
			b2PrismaticJoint prismatic = cast(b2PrismaticJoint)def.joint2;
			m_localAnchorD = prismatic.GetLocalAnchorA();
			m_localAnchorB = prismatic.GetLocalAnchorB();
			m_referenceAngleB = prismatic.GetReferenceAngle();
			m_localAxisD = prismatic.GetLocalAxisA();

			b2Vec2 pD = m_localAnchorD;
			b2Vec2 pB = b2MulT(xfD.q, b2Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
			coordinateB = b2Dot(pB - pD, m_localAxisD);
		}

		m_ratio = def.ratio;

		m_constant = coordinateA + m_ratio * coordinateB;

		m_impulse = 0.0f;
	}

	override void InitVelocityConstraints(b2SolverData data)
	{
		m_indexA = m_bodyA.m_islandIndex;
		m_indexB = m_bodyB.m_islandIndex;
		m_indexC = m_bodyC.m_islandIndex;
		m_indexD = m_bodyD.m_islandIndex;
		m_lcA = m_bodyA.m_sweep.localCenter;
		m_lcB = m_bodyB.m_sweep.localCenter;
		m_lcC = m_bodyC.m_sweep.localCenter;
		m_lcD = m_bodyD.m_sweep.localCenter;
		m_mA = m_bodyA.m_invMass;
		m_mB = m_bodyB.m_invMass;
		m_mC = m_bodyC.m_invMass;
		m_mD = m_bodyD.m_invMass;
		m_iA = m_bodyA.m_invI;
		m_iB = m_bodyB.m_invI;
		m_iC = m_bodyC.m_invI;
		m_iD = m_bodyD.m_invI;

		b2Vec2 cA = data.positions[m_indexA].c;
		float32 aA = data.positions[m_indexA].a;
		b2Vec2 vA = data.velocities[m_indexA].v;
		float32 wA = data.velocities[m_indexA].w;

		b2Vec2 cB = data.positions[m_indexB].c;
		float32 aB = data.positions[m_indexB].a;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float32 wB = data.velocities[m_indexB].w;

		b2Vec2 cC = data.positions[m_indexC].c;
		float32 aC = data.positions[m_indexC].a;
		b2Vec2 vC = data.velocities[m_indexC].v;
		float32 wC = data.velocities[m_indexC].w;

		b2Vec2 cD = data.positions[m_indexD].c;
		float32 aD = data.positions[m_indexD].a;
		b2Vec2 vD = data.velocities[m_indexD].v;
		float32 wD = data.velocities[m_indexD].w;

		b2Rot qA = b2Rot(aA);
		b2Rot qB = b2Rot(aB);
		b2Rot qC = b2Rot(aC);
		b2Rot qD = b2Rot(aD);

		m_mass = 0.0f;

		if (m_typeA == b2JointType.e_revoluteJoint)
		{
			m_JvAC.SetZero();
			m_JwA = 1.0f;
			m_JwC = 1.0f;
			m_mass += m_iA + m_iC;
		}
		else
		{
			b2Vec2 u = b2Mul(qC, m_localAxisC);
			b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
			b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
			m_JvAC = u;
			m_JwC = b2Cross(rC, u);
			m_JwA = b2Cross(rA, u);
			m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
		}

		if (m_typeB == b2JointType.e_revoluteJoint)
		{
			m_JvBD.SetZero();
			m_JwB = m_ratio;
			m_JwD = m_ratio;
			m_mass += m_ratio * m_ratio * (m_iB + m_iD);
		}
		else
		{
			b2Vec2 u = b2Mul(qD, m_localAxisD);
			b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
			b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
			m_JvBD = m_ratio * u;
			m_JwD = m_ratio * b2Cross(rD, u);
			m_JwB = m_ratio * b2Cross(rB, u);
			m_mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB;
		}

		// Compute effective mass.
		m_mass = m_mass > 0.0f ? 1.0f / m_mass : 0.0f;

		if (data.step.warmStarting)
		{
			vA += (m_mA * m_impulse) * m_JvAC;
			wA += m_iA * m_impulse * m_JwA;
			vB += (m_mB * m_impulse) * m_JvBD;
			wB += m_iB * m_impulse * m_JwB;
			vC -= (m_mC * m_impulse) * m_JvAC;
			wC -= m_iC * m_impulse * m_JwC;
			vD -= (m_mD * m_impulse) * m_JvBD;
			wD -= m_iD * m_impulse * m_JwD;
		}
		else
		{
			m_impulse = 0.0f;
		}

		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
		data.velocities[m_indexC].v = vC;
		data.velocities[m_indexC].w = wC;
		data.velocities[m_indexD].v = vD;
		data.velocities[m_indexD].w = wD;
	}
	override void SolveVelocityConstraints(b2SolverData data)
	{
		b2Vec2 vA = data.velocities[m_indexA].v;
		float32 wA = data.velocities[m_indexA].w;
		b2Vec2 vB = data.velocities[m_indexB].v;
		float32 wB = data.velocities[m_indexB].w;
		b2Vec2 vC = data.velocities[m_indexC].v;
		float32 wC = data.velocities[m_indexC].w;
		b2Vec2 vD = data.velocities[m_indexD].v;
		float32 wD = data.velocities[m_indexD].w;

		float32 Cdot = b2Dot(m_JvAC, vA - vC) + b2Dot(m_JvBD, vB - vD);
		Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);

		float32 impulse = -m_mass * Cdot;
		m_impulse += impulse;

		vA += (m_mA * impulse) * m_JvAC;
		wA += m_iA * impulse * m_JwA;
		vB += (m_mB * impulse) * m_JvBD;
		wB += m_iB * impulse * m_JwB;
		vC -= (m_mC * impulse) * m_JvAC;
		wC -= m_iC * impulse * m_JwC;
		vD -= (m_mD * impulse) * m_JvBD;
		wD -= m_iD * impulse * m_JwD;

		data.velocities[m_indexA].v = vA;
		data.velocities[m_indexA].w = wA;
		data.velocities[m_indexB].v = vB;
		data.velocities[m_indexB].w = wB;
		data.velocities[m_indexC].v = vC;
		data.velocities[m_indexC].w = wC;
		data.velocities[m_indexD].v = vD;
		data.velocities[m_indexD].w = wD;
	}
	override bool SolvePositionConstraints(b2SolverData data)
	{
		b2Vec2 cA = data.positions[m_indexA].c;
		float32 aA = data.positions[m_indexA].a;
		b2Vec2 cB = data.positions[m_indexB].c;
		float32 aB = data.positions[m_indexB].a;
		b2Vec2 cC = data.positions[m_indexC].c;
		float32 aC = data.positions[m_indexC].a;
		b2Vec2 cD = data.positions[m_indexD].c;
		float32 aD = data.positions[m_indexD].a;

		b2Rot qA = b2Rot(aA);
		b2Rot qB = b2Rot(aB);
		b2Rot qC = b2Rot(aC);
		b2Rot qD = b2Rot(aD);

		float32 linearError = 0.0f;

		float32 coordinateA, coordinateB;

		b2Vec2 JvAC, JvBD;
		float32 JwA, JwB, JwC, JwD;
		float32 mass = 0.0f;

		if (m_typeA == b2JointType.e_revoluteJoint)
		{
			JvAC.SetZero();
			JwA = 1.0f;
			JwC = 1.0f;
			mass += m_iA + m_iC;

			coordinateA = aA - aC - m_referenceAngleA;
		}
		else
		{
			b2Vec2 u = b2Mul(qC, m_localAxisC);
			b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
			b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
			JvAC = u;
			JwC = b2Cross(rC, u);
			JwA = b2Cross(rA, u);
			mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;

			b2Vec2 pC = m_localAnchorC - m_lcC;
			b2Vec2 pA = b2MulT(qC, rA + (cA - cC));
			coordinateA = b2Dot(pA - pC, m_localAxisC);
		}

		if (m_typeB == b2JointType.e_revoluteJoint)
		{
			JvBD.SetZero();
			JwB = m_ratio;
			JwD = m_ratio;
			mass += m_ratio * m_ratio * (m_iB + m_iD);

			coordinateB = aB - aD - m_referenceAngleB;
		}
		else
		{
			b2Vec2 u = b2Mul(qD, m_localAxisD);
			b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
			b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
			JvBD = m_ratio * u;
			JwD = m_ratio * b2Cross(rD, u);
			JwB = m_ratio * b2Cross(rB, u);
			mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * JwD * JwD + m_iB * JwB * JwB;

			b2Vec2 pD = m_localAnchorD - m_lcD;
			b2Vec2 pB = b2MulT(qD, rB + (cB - cD));
			coordinateB = b2Dot(pB - pD, m_localAxisD);
		}

		float32 C = (coordinateA + m_ratio * coordinateB) - m_constant;

		float32 impulse = 0.0f;
		if (mass > 0.0f)
		{
			impulse = -C / mass;
		}

		cA += m_mA * impulse * JvAC;
		aA += m_iA * impulse * JwA;
		cB += m_mB * impulse * JvBD;
		aB += m_iB * impulse * JwB;
		cC -= m_mC * impulse * JvAC;
		aC -= m_iC * impulse * JwC;
		cD -= m_mD * impulse * JvBD;
		aD -= m_iD * impulse * JwD;

		data.positions[m_indexA].c = cA;
		data.positions[m_indexA].a = aA;
		data.positions[m_indexB].c = cB;
		data.positions[m_indexB].a = aB;
		data.positions[m_indexC].c = cC;
		data.positions[m_indexC].a = aC;
		data.positions[m_indexD].c = cD;
		data.positions[m_indexD].a = aD;

		// TODO_ERIN not implemented
		return linearError < b2_linearSlop;
	}

	b2Joint m_joint1;
	b2Joint m_joint2;

	b2JointType m_typeA;
	b2JointType m_typeB;

	// Body A is connected to body C
	// Body B is connected to body D
	b2Body m_bodyC;
	b2Body m_bodyD;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Vec2 m_localAnchorC;
	b2Vec2 m_localAnchorD;

	b2Vec2 m_localAxisC;
	b2Vec2 m_localAxisD;

	float32 m_referenceAngleA;
	float32 m_referenceAngleB;

	float32 m_constant;
	float32 m_ratio;

	float32 m_impulse;

	// Solver temp
	int32 m_indexA, m_indexB, m_indexC, m_indexD;
	b2Vec2 m_lcA, m_lcB, m_lcC, m_lcD;
	float32 m_mA, m_mB, m_mC, m_mD;
	float32 m_iA, m_iB, m_iC, m_iD;
	b2Vec2 m_JvAC, m_JvBD;
	float32 m_JwA, m_JwB, m_JwC, m_JwD;
	float32 m_mass;
}

