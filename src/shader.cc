#include <shader.hpp>

char* Shader::load_file(const char *filename)
{
	FILE *fptr;
    long length;
    char *buf;

    fptr = fopen(filename, "rb");
    if (!fptr)
        return NULL;
    fseek(fptr, 0, SEEK_END);
    length = ftell(fptr);
    buf = (char*)malloc(length+1);
    fseek(fptr, 0, SEEK_SET);
    fread(buf, length, 1, fptr);
    fclose(fptr);
    buf[length] = 0;

    return buf;
}

Shader::Shader()
{}

void Shader::init(std::string vert_file, std::string frag_file)
{
	const char *vert_source = load_file(vert_file.c_str());
	const char *frag_source = load_file(frag_file.c_str());

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
		std::cout << "Error Compile " << type << " [" << success << "]\n";
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