#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/texture.h>
#include <string>
#include <iostream>

#include "Shaders/Shader.h"

class SceneViewer {
public:
    SceneViewer();
    ~SceneViewer();

    void initshader(const std::string& vertexPath, const std::string& fragmentPath);
    void loadModel(const std::string& path);
    void renderModel(glm::mat4 view, glm::mat4 projection);
    void rendersky(glm::mat4 view, glm::mat4 projection);

private:
    void ProcessNode(aiNode* node);
    void RenderMesh(aiMesh* mesh);
    void LoadMaterialTextures(aiMaterial* mat, aiTextureType type);
    void loadCubemap(std::vector<std::string> faces);

    Assimp::Importer importer;
    const aiScene* scene;
    std::vector<unsigned int> textures;
    unsigned int cubemapTexture;
    Shader shader, skyboxshader;
};