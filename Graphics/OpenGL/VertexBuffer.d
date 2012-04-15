module graphics.opengl.vertexbuffer;

import derelict.opengl3.gl3; 

class VertexBuffer
{
	this()
	{
		glGenBuffers(1, &m_handle);
		assert(m_handle && glIsBuffer(m_handle));
	}
	~this()
	{
		glDeleteBuffers(1, &m_handle);		
	}

	void Set(T)(const(T)[] data, bool dynamic)
	{
		Bind();

		uint size = data.length * data[0].sizeof;
		if (!dynamic)
		{
			glBufferData(GL_ARRAY_BUFFER, size, data.ptr, GL_STATIC_DRAW);
		}
		else
		{
			glBufferData(GL_ARRAY_BUFFER, size, data.ptr, GL_STREAM_DRAW);

			//if (size > m_curSize)
			//{
			//    glBufferData(GL_ARRAY_BUFFER, size, data.ptr, GL_DYNAMIC_DRAW);
			//    m_curSize = size;
			//}
			//else
			//{
			//    T* buf = cast(T*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
			//    buf[0..data.length] = data;
			//    glUnmapBuffer(GL_ARRAY_BUFFER);
			//}
		}				
	}	

	void Bind()
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_handle);
	}
	static void Unbind()
	{
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	private int m_curSize = 0;
	private GLuint m_handle;
}

class VertexArray
{
	this()
	{
		glGenVertexArrays(1, &m_handle);
		assert(m_handle && glIsVertexArray(m_handle));
	}
	~this()
	{
		glDeleteVertexArrays(1, &m_handle);		
	}

	void Bind()
	{
		glBindVertexArray(m_handle);
	}
	static void Unbind()
	{
		glBindVertexArray(0);
	}

	private GLuint m_handle;
}
