#include "view/SceneViewer.h"
#include <stb_image.h>

SceneViewer::SceneViewer() {
    lightPos = glm::vec3(0.0f, 0.0f, 0.0f);
    lightColor = glm::vec3(1.0f, 1.0f, 1.0f);
    selectedModelIndex = -1;
}

SceneViewer::~SceneViewer() {}

void SceneViewer::LoadModel(const std::string& path) {
    models.push_back(Model(path));
}

void SceneViewer::initshader(const std::string& vertexPath, const std::string& fragmentPath) {
    shader.init(vertexPath, fragmentPath);
    skyboxshader.init("../../AutoHomePlan/src/Shaders/skybox.vs", "../../AutoHomePlan/src/Shaders/skybox.fs");
    std::vector<std::string> faces
    {
        "../../AutoHomePlan/Assets/skybox/right.jpg",
        "../../AutoHomePlan/Assets/skybox/left.jpg",
        "../../AutoHomePlan/Assets/skybox/top.jpg",
        "../../AutoHomePlan/Assets/skybox/bottom.jpg",
        "../../AutoHomePlan/Assets/skybox/front.jpg",
        "../../AutoHomePlan/Assets/skybox/back.jpg"
    };
    loadCubemap(faces);
    skyboxshader.use();
    skyboxshader.setInt("skybox", 0);
}

void SceneViewer::renderModel(glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos) {
    for (size_t i = 0; i < models.size(); i++) {
        shader.use();
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setMat4("model", models[i].modelmatrix);
        shader.setVec3("viewPos", cameraPos);
        shader.setVec3("lightPos", lightPos);
        shader.setVec3("lightColor", lightColor);

        if (i == selectedModelIndex) {
            // Render selected model with a different color or outline
            shader.setVec3("highlightColor", glm::vec3(0.8, 0.902, 1.0)); // blue color for highlight
        } else {
            shader.setVec3("highlightColor", glm::vec3(1.0, 1.0, 1.0)); // Normal color
        }
        models[i].Draw(shader);
    }
}

void SceneViewer::loadCubemap(std::vector<std::string> faces)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrChannels;
    for (unsigned int i = 0; i < faces.size(); i++)
    {
        unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            stbi_image_free(data);
        }
        else
        {
            std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
            stbi_image_free(data);
        }
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    cubemapTexture = textureID;
}

void SceneViewer::rendersky(glm::mat4 view, glm::mat4 projection)
{
    float skyboxVertices[] = {
        // positions          
        -10.0f,  10.0f, -10.0f,
        -10.0f, -10.0f, -10.0f,
         10.0f, -10.0f, -10.0f,
         10.0f, -10.0f, -10.0f,
         10.0f,  10.0f, -10.0f,
        -10.0f,  10.0f, -10.0f,

        -10.0f, -10.0f,  10.0f,
        -10.0f, -10.0f, -10.0f,
        -10.0f,  10.0f, -10.0f,
        -10.0f,  10.0f, -10.0f,
        -10.0f,  10.0f,  10.0f,
        -10.0f, -10.0f,  10.0f,

         10.0f, -10.0f, -10.0f,
         10.0f, -10.0f,  10.0f,
         10.0f,  10.0f,  10.0f,
         10.0f,  10.0f,  10.0f,
         10.0f,  10.0f, -10.0f,
         10.0f, -10.0f, -10.0f,

        -10.0f, -10.0f,  10.0f,
        -10.0f,  10.0f,  10.0f,
         10.0f,  10.0f,  10.0f,
         10.0f,  10.0f,  10.0f,
         10.0f, -10.0f,  10.0f,
        -10.0f, -10.0f,  10.0f,

        -10.0f,  10.0f, -10.0f,
         10.0f,  10.0f, -10.0f,
         10.0f,  10.0f,  10.0f,
         10.0f,  10.0f,  10.0f,
        -10.0f,  10.0f,  10.0f,
        -10.0f,  10.0f, -10.0f,

        -10.0f, -10.0f, -10.0f,
        -10.0f, -10.0f,  10.0f,
         10.0f, -10.0f, -10.0f,
         10.0f, -10.0f, -10.0f,
        -10.0f, -10.0f,  10.0f,
         10.0f, -10.0f,  10.0f
    };
    unsigned int skyboxVAO, skyboxVBO;
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
    skyboxshader.use();
    skyboxshader.setMat4("view", view);
    skyboxshader.setMat4("projection", projection);
    // skybox cube
    //glBindVertexArray(skyboxVAO);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
    glDepthFunc(GL_LESS); // set depth function back to default

    glDeleteVertexArrays(1, &skyboxVAO);
    glDeleteBuffers(1, &skyboxVBO);
}

bool SceneViewer::SelectModelAt(double xpos, double ypos, glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos) {
    // Get the viewport dimensions
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    // Convert mouse coordinates to normalized device coordinates
    float x = (2.0f * xpos) / viewport[2] - 1.0f;
    float y = 1.0f - (2.0f * ypos) / viewport[3];
    float z = 1.0f;
    glm::vec3 ray_nds = glm::vec3(x, y, z);

    // Convert to homogeneous clip coordinates
    glm::vec4 ray_clip = glm::vec4(ray_nds.x, ray_nds.y, -1.0, 1.0);

    // Convert to eye coordinates
    glm::vec4 ray_eye = glm::inverse(projection) * ray_clip;
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);

    // Convert to world coordinates
    glm::vec3 ray_world = glm::normalize(glm::vec3(glm::inverse(view) * ray_eye));

    // Check intersection with models
    float minDistance = FLT_MAX;
    int to_select = -1;
    for (size_t i = 0; i < models.size(); i++) {
        float distance;
        if (RayIntersectsModel(ray_world, cameraPos, models[i], distance)) {
            if (distance < minDistance) {
                minDistance = distance;
                to_select = i;
            }
        }
    }
    if (to_select != -1 && to_select != selectedModelIndex) {
        selectedModelIndex = to_select;
        return true;
    } 
    else {
        selectedModelIndex = -1;
        return false;
    }
}

bool SceneViewer::RayIntersectsModel(const glm::vec3& ray, const glm::vec3& origin, const Model& model, float& distance) {
    return RayIntersectsAABB(origin, ray, model.minBounds, model.maxBounds, distance);
}

bool SceneViewer::RayIntersectsAABB(const glm::vec3& rayOrigin, const glm::vec3& rayDir, const glm::vec3& boxMin, const glm::vec3& boxMax, float& distance) {
    float tMin = (boxMin.x - rayOrigin.x) / rayDir.x;
    float tMax = (boxMax.x - rayOrigin.x) / rayDir.x;

    if (tMin > tMax) std::swap(tMin, tMax);

    float tyMin = (boxMin.y - rayOrigin.y) / rayDir.y;
    float tyMax = (boxMax.y - rayOrigin.y) / rayDir.y;

    if (tyMin > tyMax) std::swap(tyMin, tyMax);

    if ((tMin > tyMax) || (tyMin > tMax))
        return false;

    if (tyMin > tMin)
        tMin = tyMin;
    if (tyMax < tMax)
        tMax = tyMax;

    float tzMin = (boxMin.z - rayOrigin.z) / rayDir.z;
    float tzMax = (boxMax.z - rayOrigin.z) / rayDir.z;

    if (tzMin > tzMax) std::swap(tzMin, tzMax);

    if ((tMin > tzMax) || (tzMin > tMax))
        return false;

    if (tzMin > tMin)
        tMin = tzMin;
    if (tzMax < tMax)
        tMax = tzMax;

    distance = tMin;  // intersection distance
    return true;
}

void SceneViewer::DeleteSelectedModel() {
    if (selectedModelIndex != -1) {
        models.erase(models.begin() + selectedModelIndex);
        selectedModelIndex = -1; // Reset selection
    }
}