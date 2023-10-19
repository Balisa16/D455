#include <shader.hpp>

std::string Shader::load_file(const char *filename)
{
    std::ifstream file_streamer;
    std::string data;
    file_streamer.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
    {
        file_streamer.open(filename);
        std::stringstream shader_stream;
        shader_stream << file_streamer.rdbuf();
        file_streamer.close();
        data = shader_stream.str();
    }
    catch (std::ifstream::failure& e)
    {
        std::cout << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << e.what() << std::endl;
        throw;
    }
    return data;
}

Shader::Shader(){}

void Shader::init(std::string vert_file, std::string frag_file)
{
	const char *vert_source = load_file(vert_file.c_str()).c_str();
	const char *frag_source = load_file(frag_file.c_str()).c_str();

	// Vertex Shader
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vert_source, NULL);
    glCompileShader(vertexShader);
    check_compile("Vertex Shader");

    // Fragment Shader
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &frag_source, NULL);
    glCompileShader(fragmentShader);
    check_compile("Fragment Shader");

    // Link shaders
    ID = glCreateProgram();
    glAttachShader(ID, vertexShader);
    glAttachShader(ID, fragmentShader);
    glLinkProgram(ID);
    check_compile("Program");
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Shader::use()
{
	glUseProgram(ID);
}

void Shader::check_compile(std::string type, bool program)
{
	GLint success;
	if(program)
		glGetProgramiv(ID, GL_LINK_STATUS, &success);
	else
		glGetShaderiv(ID, GL_COMPILE_STATUS, &success);

	if(!success)
		std::cout << "Error Compile " << type << '\n';
}

void Shader::setI(const std::string name, int value)
{
    glUniform1i(glGetUniformLocation(ID, name.c_str()), value); 
}

void Shader::setM4(const std::string name, const glm::mat4 &mat) const
{
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

Shader::~Shader()
{
	glDeleteProgram(ID);
}