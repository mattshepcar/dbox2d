module box2d.testbed.framework.render;

import box2d.common.b2Draw;
import box2d.common.b2Math;
import box2d.collision.b2Collision;

import graphics.opengl.imm2d;

uint ToU32(b2Color color)
{
	uint r = cast(uint)(color.r * 255);
	uint g = cast(uint)(color.g * 255);
	uint b = cast(uint)(color.b * 255);
	uint a = 255;
	return (a<<24)|(b<<16)|(g<<8)|(r<<0);
}

class DebugDraw : b2Draw
{
	override void DrawPolygon(const b2Vec2[] vertices, b2Color color)
	{
		g_Imm2d.Poly(vertices, color.ToU32());
	}

	override void DrawSolidPolygon(const b2Vec2[] vertices, b2Color color)
	{
		g_Imm2d.Poly(vertices, color.ToU32());
	}

	override void DrawCircle(b2Vec2 center, float32 radius, b2Color color)
	{
		b2Vec2[32] verts;
		foreach(i; 0..32)
		{
			b2Rot r = b2Rot(i * b2_pi / 16.0f);
			verts[i] = center + r.GetXAxis() * radius;
		}
		g_Imm2d.Poly(verts, color.ToU32());
	}

	override void DrawSolidCircle(b2Vec2 center, float32 radius, b2Vec2 axis, b2Color color)
	{
		DrawCircle(center, radius, color);
		DrawSegment(center, center + axis * radius, color);
	}

	override void DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color)
	{
		g_Imm2d.Line(p1.x, p1.y, p2.x, p2.y, color.ToU32());
	}

	override void DrawTransform(b2Transform xf)
	{
	}

    void DrawPoint(b2Vec2 p, float32 size, b2Color color)
	{
	}

	void DrawString(int x, int y, string s, ...)
	{
	}

    void DrawString(b2Vec2 p, string s, ...)
	{
	}

    void DrawAABB(b2AABB aabb, b2Color color)
	{
	}
}
