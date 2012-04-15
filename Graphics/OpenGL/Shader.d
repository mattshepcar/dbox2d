module graphics.opengl.shader;

import derelict.opengl3.gl3; 

private string Log(GLuint handle, da_glGetShaderiv getparam, da_glGetShaderInfoLog getlog)
{
	int infoLogLen = 0;
	getparam(handle, GL_INFO_LOG_LENGTH, &infoLogLen);
	GLchar[] infoLog;
	if (infoLogLen > 0)
	{
		infoLog.length = infoLogLen - 1;
		int charsWritten = 0;
		getlog(handle, infoLogLen, &charsWritten, &infoLog[0]);
		assert(infoLog.length == charsWritten);
	}
	return infoLog.idup;
}

class Shader
{
	this(uint type, string src = "")
	{
		assert(type == GL_VERTEX_SHADER || type == GL_FRAGMENT_SHADER);
		m_handle = glCreateShader(type);
		assert(m_handle != 0 && glIsShader(m_handle));
		if (src.length)
			Compile(src);
	}

	~this()
	{
		glDeleteShader(m_handle);
	}

	void Compile(string src)
	{
		GLint len = src.length;
		const(char*) strings[] = [std.string.toStringz(src)];
		glShaderSource(m_handle, 1, &strings[0], &len);
		glCompileShader(m_handle);
		if (!IsCompiled())
			throw new Exception(Log(m_handle, glGetShaderiv, glGetShaderInfoLog));
	}

	bool IsCompiled() const
	{
		GLint compiled;
		glGetShaderiv(m_handle, GL_COMPILE_STATUS, &compiled);
		return compiled != 0;
	}

	int Type() const
	{
		GLint type = 0;
		glGetShaderiv(m_handle, GL_SHADER_TYPE, &type);
		return type;
	}

	private GLuint m_handle;
}

class VShader : Shader
{
	this(string src = "") {super(GL_VERTEX_SHADER, src);}

}

class FShader : Shader
{
	this(string src = "") {super(GL_FRAGMENT_SHADER, src);}
}

class Program
{
	this(VShader vertex, FShader fragment)
	{
		m_handle = glCreateProgram();
		assert(m_handle && glIsProgram(m_handle));
		Link(vertex, fragment);
	}
	~this()
	{
		glDeleteProgram(m_handle);
	}

	void Link(VShader vertex, FShader fragment)
	{
		assert(vertex.IsCompiled() && fragment.IsCompiled());
		glBindAttribLocation(m_handle, 0, "in_Position");
		glBindAttribLocation(m_handle, 1, "in_Color");
		glAttachShader(m_handle, vertex.m_handle);
		glAttachShader(m_handle, fragment.m_handle);
		glLinkProgram(m_handle);
		GLint status = 0;
		glGetProgramiv(m_handle, GL_LINK_STATUS, &status);
		if (!status)
			throw new Exception(Log(m_handle, glGetProgramiv, glGetProgramInfoLog));
	}

	void Bind()
	{
		glUseProgram(m_handle);
	}

	private GLuint m_handle;
}
