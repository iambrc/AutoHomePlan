#pragma once

#include "Model.h"
#include "Components/SceneGraph.h"
#include "Components/InputScene.h"
#include <GLFW/glfw3.h>

class SceneViewer {
public:
    SceneViewer();
    ~SceneViewer();

    void initshader(const std::string& vertexPath, const std::string& fragmentPath);
    void LoadModel(const std::string& path);
    void renderModel(glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos);
    void rendersky(glm::mat4 view, glm::mat4 projection);
    void renderOtherMesh(glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos);

    bool SelectModelAt(double xpos, double ypos, glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos);
    bool RayIntersectsModel(const glm::vec3& ray, const glm::vec3& origin, const Model& model, float& distance);
    bool RayIntersectsAABB(const glm::vec3& rayOrigin, const glm::vec3& rayDir, const glm::vec3& boxMin, const glm::vec3& boxMax, float& distance);
    void DeleteSelectedModel();

    void setupRooms(SceneGraph g, float bmsize);
    void setupOneRoom(SceneGraph g, Boundary b);

    void reset();

    glm::vec3 lightPos;
    glm::vec3 lightColor;

    float wallWidth;

private:
    void loadCubemap(std::vector<std::string> faces);

    Mesh GenerateSquare(const glm::vec3& pos, const glm::vec3& size, Material mat, glm::vec3 normal,float scalefactor);
    Mesh GenerateFloor(std::vector<double> pos, std::vector<double> size, float scalefactor);
    std::vector<Mesh> GenerateWall(std::vector<std::vector<double>> points, float height, float scalefactor);

    Mesh GenerateCube(const glm::vec3& pos, const glm::vec3& size, Material mat, float scalefactor);

    std::vector<Model> models;
    std::vector<Mesh> othermeshes;
    unsigned int cubemapTexture;
    Shader shader, skyboxshader;
    int selectedModelIndex;
};