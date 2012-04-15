module graphics.opengl.imm2d;

import graphics.opengl.vertexbuffer;
import derelict.opengl3.gl3; 

import std.array;

struct ImmVertex
{
	float x, y;
	uint colour;
};

class Imm2d
{
	int m_curBuffer = 0;
	VertexArray[2]				m_vao;
	VertexBuffer[2]				m_vbo;
	Appender!(ImmVertex[])		m_verts;
	Appender!(uint[])			m_lines;

	this()
	{
		foreach(i; 0..2)
		{
			m_vao[i] = new VertexArray;
			m_vbo[i] = new VertexBuffer;
			m_vao[i].Bind();
			m_vbo[i].Bind();
			ImmVertex* v = null;
			glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, ImmVertex.sizeof, &v.x);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, ImmVertex.sizeof, &v.colour); 
			glEnableVertexAttribArray(1);
			VertexArray.Unbind();
		}
		m_verts.reserve(1024);
		m_lines.reserve(1024);

		m_vao[0].Bind();
	}

	private void Vertex(ref Appender!(uint[]) indices, ImmVertex v)
	{
		indices.put(m_verts.data.length);
		m_verts.put(v);
	}

	void Line(float x0, float y0, float x1, float y1, uint colour)
	{
		Vertex(m_lines, ImmVertex(x0, y0, colour));
		Vertex(m_lines, ImmVertex(x1, y1, colour));
	}
	void Box(float x0, float y0, float x1, float y1, uint colour)
	{
		Vertex(m_lines, ImmVertex(x0, y0, colour));
		Vertex(m_lines, ImmVertex(x1, y0, colour));
		Vertex(m_lines, ImmVertex(x1, y1, colour));
		Vertex(m_lines, ImmVertex(x0, y1, colour));
	}
	void Poly(T)(const(T)[] verts, uint colour = 0xFFFFFFFF)
	{
		int firstVert = m_verts.data.length;
		foreach(v; verts)
		{
			m_verts.put(ImmVertex(v.x, v.y, colour));
		}
		m_lines.put(firstVert);
		foreach(i; 1..verts.length)
		{
			m_lines.put(firstVert + i);
			m_lines.put(firstVert + i);
		}
		m_lines.put(firstVert);
	}
	void Draw()
	{
		int i = m_curBuffer;
		//m_curBuffer ^= 1;
		m_vbo[i].Set(m_verts.data, true);

		//m_vao[i].Bind();
		glDrawElements(GL_LINES, m_lines.data.length, GL_UNSIGNED_INT, m_lines.data.ptr);
		//VertexArray.Unbind();

		m_lines.clear();
		m_verts.clear();
	}
}

Imm2d g_Imm2d;
