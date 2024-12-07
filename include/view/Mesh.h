#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Shaders/Shader.h"

#include <string>
#include <vector>

#define MAX_BONE_INFLUENCE 4

struct Vertex {
    // position
    glm::vec3 Position;
    // normal
    glm::vec3 Normal;
    // texCoords
    glm::vec2 TexCoords;
    // tangent
    glm::vec3 Tangent;
    // bitangent
    glm::vec3 Bitangent;
	//bone indexes which will influence this vertex
	int m_BoneIDs[MAX_BONE_INFLUENCE];
	//weights from each bone
	float m_Weights[MAX_BONE_INFLUENCE];
};

struct Texture {
    unsigned int id;
    std::string type;
    std::string path;
};

struct Material {
    glm::vec3 diffuseColor;
    glm::vec3 specularColor;
    glm::vec3 ambientColor;
    float shininess;
    std::vector<Texture> textures;
};

class Mesh {
public:
    // mesh Data
    std::vector<Vertex>       vertices;
    std::vector<unsigned int> indices;
    Material                  material;
    unsigned int VAO;

    // constructor
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices, Material material);

    // render the mesh
    void Draw(Shader &shader);

private:
    // render data 
    unsigned int VBO, EBO;

    // initializes all the buffer objects/arrays
    void setupMesh();
};