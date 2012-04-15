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

module box2d.dynamics.b2ChainAndPolygonContact;

import box2d.common.b2Math;

import box2d.collision.b2Collision;
import box2d.collision.b2CollideEdge;
import box2d.collision.shapes.b2Shape;
import box2d.collision.shapes.b2ChainShape;
import box2d.collision.shapes.b2EdgeShape;
import box2d.collision.shapes.b2PolygonShape;

import box2d.dynamics.b2Contact;
import box2d.dynamics.b2Fixture;


class b2ChainAndPolygonContact : b2Contact
{
public:
	static b2ChainAndPolygonContact Create(b2Fixture fixtureA, int32 indexA,
								b2Fixture fixtureB, int32 indexB)
	{
		return new b2ChainAndPolygonContact(fixtureA, indexA, fixtureB, indexB);
	}

	this(b2Fixture fixtureA, int32 indexA, b2Fixture fixtureB, int32 indexB)
	{
		assert(fixtureA.GetType() == b2Shape.Type.e_chain);
		assert(fixtureB.GetType() == b2Shape.Type.e_polygon);
		super(fixtureA, indexA, fixtureB, indexB);
	}

	override void Evaluate(ref b2Manifold manifold, b2Transform xfA, b2Transform xfB)
	{
		auto chain = cast(b2ChainShape)m_fixtureA.GetShape();
		auto edge = chain.GetChildEdge(m_indexA);
		auto polygon = cast(b2PolygonShape)m_fixtureB.GetShape();
		b2CollideEdgeAndPolygon(manifold, edge, xfA, polygon, xfB);
	}
}
