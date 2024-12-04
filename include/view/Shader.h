#pragma once
#include <string>
#include <GL/glew.h>

class Shader {
public:
    GLuint Program;
    Shader(const char* vertexPath, const char* fragmentPath);
    void Use();
};