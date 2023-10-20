#ifndef SHADER_HPP
#define SHADER_HPP

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
// #include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glad/glad.h>

class Shader
{
private:
    unsigned int ID;
    char* load_file(const char *filename);
    void check_compile(std::string type, bool program = false);
public:
    Shader();
    void init(std::string vert_file, std::string frag_file);
    void use();
    void setI(const std::string name, int value);
    void setM4(const std::string name, const glm::mat4 &mat) const;
    ~Shader();
};

#endif
