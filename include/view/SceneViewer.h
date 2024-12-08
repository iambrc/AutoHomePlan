#pragma once

#include "Model.h"
#include <GLFW/glfw3.h>

class SceneViewer {
public:
    SceneViewer();
    ~SceneViewer();

    void initshader(const std::string& vertexPath, const std::string& fragmentPath);
    void LoadModel(const std::string& path);
    void renderModel(glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos);
    void rendersky(glm::mat4 view, glm::mat4 projection);

    bool SelectModelAt(double xpos, double ypos, glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos);
    bool RayIntersectsModel(const glm::vec3& ray, const glm::vec3& origin, const Model& model, float& distance);
    bool RayIntersectsAABB(const glm::vec3& rayOrigin, const glm::vec3& rayDir, const glm::vec3& boxMin, const glm::vec3& boxMax, float& distance);
    void DeleteSelectedModel();

    glm::vec3 lightPos;
    glm::vec3 lightColor;

private:
    void loadCubemap(std::vector<std::string> faces);

    std::vector<Model> models;
    unsigned int cubemapTexture;
    Shader shader, skyboxshader;
    int selectedModelIndex;
};