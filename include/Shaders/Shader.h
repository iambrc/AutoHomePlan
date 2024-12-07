#pragma once

#include <glad/glad.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader {
public:
    unsigned int ID;

    Shader(const std::string& vertexPath, const std::string& fragmentPath);

    Shader();

    void use() const;
    
    void init(const std::string& vertexPath, const std::string& fragmentPath);

    void setBool(const std::string& name, bool value) const;
    void setInt(const std::string& name, int value) const;
    void setFloat(const std::string& name, float value) const;
    void setMat4(const std::string& name, const glm::mat4& matrix) const;
    void setVec3(const std::string& name, const glm::vec3& value) const;

private:
    void checkCompileErrors(unsigned int shader, const std::string& type);

    std::string readFile(const std::string& filePath);
};