/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

module box2d.collision.shapes.b2ChainShape;

import box2d.common.b2Math;
import box2d.collision.b2Collision;
import box2d.collision.shapes.b2Shape;
import box2d.collision.shapes.b2EdgeShape;

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
class b2ChainShape : b2Shape
{
public:
	this()
	{
		m_type = Type.e_chain;
		m_radius = b2_polygonRadius;
	}
	this(const b2ChainShape other)
	{
		this();
		m_vertices = other.m_vertices.dup;
		m_prevVertex = other.m_prevVertex;
		m_nextVertex = other.m_nextVertex;
		m_hasPrevVertex = other.m_hasPrevVertex;
		m_hasNextVertex = other.m_hasNextVertex;
	}

	/// Create a loop. This automatically adjusts connectivity.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	void CreateLoop(const b2Vec2[] vertices)
	{
		assert(!m_vertices.length);
		assert(vertices.length >= 3);
		for (int32 i = 1; i < vertices.length; ++i)
		{
			b2Vec2 v1 = vertices[i-1];
			b2Vec2 v2 = vertices[i];
			b2Vec2 e = v2 - v1;
			// If the code crashes here, it means your vertices are too close together.
			assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
		}

		m_vertices = new b2Vec2[vertices.length + 1];
		m_vertices[0..$-1] = vertices;
		m_vertices[$-1] = vertices[0];
		m_prevVertex = m_vertices[$ - 2];
		m_nextVertex = m_vertices[1];
		m_hasPrevVertex = true;
		m_hasNextVertex = true;
	}

	/// Create a chain with isolated end vertices.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	void CreateChain(const b2Vec2[] vertices)
	{
		assert(!m_vertices.length);
		assert(vertices.length >= 2);
		for (int32 i = 1; i < vertices.length; ++i)
		{
			b2Vec2 v1 = vertices[i-1];
			b2Vec2 v2 = vertices[i];
			b2Vec2 e = v2 - v1;
			// If the code crashes here, it means your vertices are too close together.
			assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
		}

		m_vertices = vertices.dup;
		m_hasPrevVertex = false;
		m_hasNextVertex = false;
	}

	/// Establish connectivity to a vertex that precedes the first vertex.
	/// Don't call this for loops.
	void SetPrevVertex(b2Vec2 prevVertex)
	{
		m_prevVertex = prevVertex;
		m_hasPrevVertex = true;
	}

	/// Establish connectivity to a vertex that follows the last vertex.
	/// Don't call this for loops.
	void SetNextVertex(b2Vec2 nextVertex)
	{
		m_nextVertex = nextVertex;
		m_hasNextVertex = true;
	}

	/// Implement b2Shape. Vertices are cloned using b2Alloc.
	override b2ChainShape Clone() const
	{
		return new b2ChainShape(this);
	}

	/// @see b2Shape::GetChildCount
	override int32 GetChildCount() const
	{
		return m_vertices.length - 1;
	}

	/// Get a child edge.
	b2EdgeShapeDef GetChildEdge(int32 index) const
	{
		b2EdgeShapeDef edge;
		edge.m_vertex1 = m_vertices[index + 0];
		edge.m_vertex2 = m_vertices[index + 1];

		if (index > 0)
		{
			edge.m_vertex0 = m_vertices[index - 1];
			edge.m_hasVertex0 = true;
		}
		else
		{
			edge.m_vertex0 = m_prevVertex;
			edge.m_hasVertex0 = m_hasPrevVertex;
		}

		if (index < m_vertices.length - 2)
		{
			edge.m_vertex3 = m_vertices[index + 2];
			edge.m_hasVertex3 = true;
		}
		else
		{
			edge.m_vertex3 = m_nextVertex;
			edge.m_hasVertex3 = m_hasNextVertex;
		}
		return edge;
	}

	/// This always return false.
	/// @see b2Shape::TestPoint
	override bool TestPoint(b2Transform transform, b2Vec2 p) const
	{
		return false;
	}

	/// Implement b2Shape.
	override bool RayCast(ref b2RayCastOutput output, b2RayCastInput input,
						  b2Transform xf, int32 childIndex) const
	{
		auto edgeShape = new b2EdgeShape;

		int32 i1 = childIndex;
		int32 i2 = childIndex + 1;
		if (i2 == m_vertices.length)
		{
			i2 = 0;
		}

		edgeShape.m_vertex1 = m_vertices[i1];
		edgeShape.m_vertex2 = m_vertices[i2];

		return edgeShape.RayCast(output, input, xf, 0);
	}

	/// @see b2Shape::ComputeAABB
	override void ComputeAABB(ref b2AABB aabb, b2Transform xf, int32 childIndex) const
	{
		int32 i1 = childIndex;
		int32 i2 = childIndex + 1;
		if (i2 == m_vertices.length)
		{
			i2 = 0;
		}

		b2Vec2 v1 = b2Mul(xf, m_vertices[i1]);
		b2Vec2 v2 = b2Mul(xf, m_vertices[i2]);

		aabb.lowerBound = b2Min(v1, v2);
		aabb.upperBound = b2Max(v1, v2);
	}

	/// Chains have zero mass.
	/// @see b2Shape::ComputeMass
	override void ComputeMass(ref b2MassData massData, float32 density) const
	{
		massData.mass = 0.0f;
		massData.center.SetZero();
		massData.I = 0.0f;
	}

	/// The vertices. Owned by this class.
	b2Vec2[] m_vertices;

	b2Vec2 m_prevVertex, m_nextVertex;
	bool m_hasPrevVertex = false;
	bool m_hasNextVertex = false;
}

