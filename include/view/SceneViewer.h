#pragma once

#include "Model.h"

class SceneViewer {
public:
    SceneViewer();
    ~SceneViewer();

    void initshader(const std::string& vertexPath, const std::string& fragmentPath);
    void LoadModel(const std::string& path);
    void renderModel(glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos);
    void rendersky(glm::mat4 view, glm::mat4 projection);

private:
    void loadCubemap(std::vector<std::string> faces);

    std::vector<Model> models;
    unsigned int cubemapTexture;
    Shader shader, skyboxshader;
};