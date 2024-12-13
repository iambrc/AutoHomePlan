#include "view/SceneViewer.h"
#include <stb_image.h>

SceneViewer::SceneViewer() {
    lightPos = glm::vec3(5.0f, 5.0f, 5.0f);
    lightColor = glm::vec3(1.0f, 1.0f, 1.0f);
    selectedModelIndex = -1;
    wallWidth = 0.02f;
    othermeshes = {};
}

SceneViewer::~SceneViewer() {}

void SceneViewer::LoadModel(const std::string& path) {
    models.push_back(Model(path));
}

void SceneViewer::initshader(const std::string& vertexPath, const std::string& fragmentPath) {
    shader.init(vertexPath, fragmentPath);
    skyboxshader.init("../../../src/Shaders/skybox.vs", "../../../src/Shaders/skybox.fs");
    std::vector<std::string> faces
    {
        "../../../Assets/skybox/right.jpg",
        "../../../Assets/skybox/left.jpg",
        "../../../Assets/skybox/top.jpg",
        "../../../Assets/skybox/bottom.jpg",
        "../../../Assets/skybox/front.jpg",
        "../../../Assets/skybox/back.jpg"
    };
    loadCubemap(faces);
    skyboxshader.use();
    skyboxshader.setInt("skybox", 0);
}

void SceneViewer::renderOtherMesh(glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos) {
    shader.use();
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);
    shader.setMat4("model", glm::mat4(1.0f));
    shader.setVec3("viewPos", cameraPos);
    shader.setVec3("lightPos", lightPos);
    shader.setVec3("lightColor", lightColor);
    shader.setVec3("highlightColor", glm::vec3(1.0, 1.0, 1.0));
    for (auto i = 0; i < othermeshes.size(); ++i) {
        othermeshes[i].Draw(shader);
    }
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

void SceneViewer::setupRooms(SceneGraph g, float bmsize)
{
    float scalefactor = 10.0f / bmsize;
    VertexIterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        othermeshes.push_back(GenerateFloor(g[*vi].pos, g[*vi].size, scalefactor));
    }
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; vi += 2) {
        std::vector<std::vector<double>> points = {
            { g[*vi + 1].pos[0] - g[*vi + 1].size[0] / 2, g[*vi + 1].pos[1] - g[*vi + 1].size[1] / 2 },
            { g[*vi + 1].pos[0] + g[*vi + 1].size[0] / 2, g[*vi + 1].pos[1] - g[*vi + 1].size[1] / 2 },
            { g[*vi + 1].pos[0] + g[*vi + 1].size[0] / 2, g[*vi + 1].pos[1] + g[*vi + 1].size[1] / 2 },
            { g[*vi].pos[0] + g[*vi].size[0] / 2, g[*vi].pos[1] - g[*vi].size[1] / 2 },
            { g[*vi].pos[0] + g[*vi].size[0] / 2, g[*vi].pos[1] + g[*vi].size[1] / 2 },
            { g[*vi].pos[0] - g[*vi].size[0] / 2, g[*vi].pos[1] + g[*vi].size[1] / 2 },
            { g[*vi].pos[0] - g[*vi].size[0] / 2, g[*vi].pos[1] - g[*vi].size[1] / 2 },
            { g[*vi + 1].pos[0] - g[*vi + 1].size[0] / 2, g[*vi + 1].pos[1] + g[*vi + 1].size[1] / 2 },
        };
        if (std::fabs(g[*vi].pos[0] - g[*vi].size[0] / 2 - g[*vi + 1].pos[0] - g[*vi + 1].size[0] / 2) < 1e-3) {
            points[2] = { g[*vi].pos[0] - g[*vi].size[0] / 2, g[*vi].pos[1] - g[*vi].size[1] / 2 };
            points[6] = { g[*vi + 1].pos[0] + g[*vi + 1].size[0] / 2, g[*vi + 1].pos[1] + g[*vi + 1].size[1] / 2 };
        }
        float height = g[*vi].size[2];
        auto wallmeshes = GenerateWall(points, height, scalefactor);
        for (auto& wall : wallmeshes)
            othermeshes.push_back(wall);
    }
}

Mesh SceneViewer::GenerateFloor(std::vector<double> pos, std::vector<double> size, float scalefactor)
{
    Material mat;
    mat.diffuseColor = glm::vec3(0.6f, 0.4f, 0.2f);
    mat.ambientColor = glm::vec3(0.3f, 0.3f, 0.1f);
    mat.specularColor = glm::vec3(0.1f, 0.1f, 0.1f);
    mat.shininess = 32.0f;
    mat.textures = {};
    return GenerateSquare(glm::vec3(pos[1], 0.00001f, pos[0]), glm::vec3(size[1], 0.00001f, size[0]), mat, glm::vec3(0.0f, 1.0f, 0.0f), scalefactor);
}

Mesh SceneViewer::GenerateSquare(const glm::vec3& pos, const glm::vec3& size, Material mat, glm::vec3 normal,float scalefactor)
{
    glm::vec3 p1 = pos - size / 2.0f;
    glm::vec3 p2 = pos + glm::vec3(size.x / 2.0f, size.y / 2.0f, -size.z / 2.0f);
    glm::vec3 p3 = pos + size / 2.0f;
    glm::vec3 p4 = pos + glm::vec3(-size.x / 2.0f, -size.y / 2.0f, size.z / 2.0f);
    if (size.z <= 1e-6) {
        p2 = pos + glm::vec3(size.x / 2.0f, -size.y / 2.0f, 0.0f);
        p4 = pos + glm::vec3(-size.x / 2.0f, size.y / 2.0f, 0.0f);
    }
    std::vector<Vertex> vertices = {
        Vertex({ p1 * scalefactor, normal, glm::vec2(0.0f, 0.0f) }),
        Vertex({ p2 * scalefactor, normal, glm::vec2(1.0f, 0.0f) }),
        Vertex({ p3 * scalefactor, normal, glm::vec2(1.0f, 1.0f) }),
        Vertex({ p4 * scalefactor, normal, glm::vec2(0.0f, 1.0f) }),
    };
    std::vector<unsigned int> indices = {
        0, 1, 3,
        1, 2, 3
    };
    return Mesh(vertices, indices, mat);
}

std::vector<Mesh> SceneViewer::GenerateWall(std::vector<std::vector<double>> points, float height, float scalefactor)
{
    Material mat;
    mat.diffuseColor = glm::vec3(0.7f, 0.7f, 0.7f);
    mat.ambientColor = glm::vec3(0.2f, 0.2f, 0.2f);
    mat.specularColor = glm::vec3(0.2f, 0.2f, 0.2f);
    mat.shininess = 32.0f;
    mat.textures = {};
    
    std::vector<Mesh> res = {};
    for (auto i = 0; i < points.size(); ++i) {
        glm::vec2 p1 = { points[i][0], points[i][1] };
        glm::vec2 p2 = { points[(i + 1) % points.size()][0], points[(i + 1) % points.size()][1] };
        if (glm::abs(p1.y - p2.y) < 1e-6) {
            res.push_back(GenerateSquare(glm::vec3(p1.y, height, (p1.x + p2.x) / 2), glm::vec3(wallWidth, 0.0f, glm::abs(p1.x - p2.x) + wallWidth), mat, glm::vec3(0.0f, 1.0f, 0.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3(p1.y, -0.00001f, (p1.x + p2.x) / 2), glm::vec3(wallWidth, 0.0f, glm::abs(p1.x - p2.x) + wallWidth), mat, glm::vec3(0.0f, -1.0f, 0.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3(p1.y + wallWidth / 2, height / 2, (p1.x + p2.x) / 2), glm::vec3(0.0f, height, glm::abs(p1.x - p2.x) + wallWidth), mat, glm::vec3(1.0f, 0.0f, 0.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3(p1.y - wallWidth / 2, height / 2, (p1.x + p2.x) / 2), glm::vec3(0.0f, height, glm::abs(p1.x - p2.x) + wallWidth), mat, glm::vec3(-1.0f, 0.0f, 0.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3(p1.y, height / 2, p1.x - wallWidth / 2), glm::vec3(wallWidth, height, 0.0f), mat, glm::vec3(0.0f, 0.0f, -1.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3(p1.y, height / 2, p2.x + wallWidth / 2), glm::vec3(wallWidth, height, 0.0f), mat, glm::vec3(0.0f, 0.0f, 1.0f), scalefactor));
        }
        else {
            res.push_back(GenerateSquare(glm::vec3((p1.y + p2.y) / 2, height, p1.x), glm::vec3(glm::abs(p1.y - p2.y) + wallWidth, 0.0f, wallWidth), mat, glm::vec3(0.0f, 1.0f, 0.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3((p1.y + p2.y) / 2, 0.0f, p1.x), glm::vec3(glm::abs(p1.y - p2.y) + wallWidth, 0.0f, wallWidth), mat, glm::vec3(0.0f, -1.0f, 0.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3((p1.y + p2.y) / 2, height / 2, p1.x + wallWidth / 2), glm::vec3(glm::abs(p1.y - p2.y) + wallWidth, height, 0.0f), mat, glm::vec3(0.0f, 0.0f, 1.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3((p1.y + p2.y) / 2, height / 2, p1.x - wallWidth / 2), glm::vec3(glm::abs(p1.y - p2.y) + wallWidth, height, 0.0f), mat, glm::vec3(0.0f, 0.0f, -1.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3(p1.y - wallWidth / 2, height / 2, p1.x), glm::vec3(0.0f, height, wallWidth), mat, glm::vec3(-1.0f, 0.0f, 0.0f), scalefactor));
            res.push_back(GenerateSquare(glm::vec3(p2.y + wallWidth / 2, height / 2, p1.x), glm::vec3(0.0f, height, wallWidth), mat, glm::vec3(1.0f, 0.0f, 0.0f), scalefactor));
        }
    }
    return res;
}

void SceneViewer::setupOneRoom(SceneGraph g, Boundary b)
{
    float scalefactor = 10.0f / std::max(b.size[0], std::max(b.size[1], b.size[2]));
    Material mat;
    mat.diffuseColor = glm::vec3(0.4f, 0.4f, 0.4f);
    mat.ambientColor = glm::vec3(0.4f, 0.4f, 0.4f);
    mat.specularColor = glm::vec3(0.1f, 0.1f, 0.1f);
    mat.shininess = 32.0f;
    mat.textures = {};
    VertexIterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        glm::vec3 pos = glm::vec3(g[*vi].pos[1], g[*vi].pos[2], g[*vi].pos[0]);
        glm::vec3 size = glm::vec3(g[*vi].size[1], g[*vi].size[2], g[*vi].size[0]);
        othermeshes.push_back(GenerateCube(pos, size, mat, scalefactor));
    }
    auto wallmeshes = GenerateWall(b.points, b.size[2], scalefactor);
    for (auto& wall : wallmeshes)
        othermeshes.push_back(wall);
    othermeshes.push_back(GenerateFloor({b.origin_pos[0] + b.size[0] / 2, b.origin_pos[1] + b.size[1] / 2}, {b.size[0], b.size[1]}, scalefactor));
}

Mesh SceneViewer::GenerateCube(const glm::vec3& pos, const glm::vec3& size, Material mat, float scalefactor)
{
    glm::vec3 p1 = pos - size / 2.0f;
    glm::vec3 p2 = pos + size / 2.0f;
    glm::vec3 n = glm::normalize(size);
    std::vector<Vertex> vertices = {
        Vertex({ p1 * scalefactor, -n, glm::vec2(0.0f, 0.0f) }),
        Vertex({ glm::vec3(p2.x, p1.y, p1.z) * scalefactor, glm::vec3(n.x, -n.y, -n.z), glm::vec2(0.0f, 0.0f) }),
        Vertex({ glm::vec3(p2.x, p2.y, p1.z) * scalefactor, glm::vec3(n.x, n.y, -n.z), glm::vec2(0.0f, 0.0f) }),
        Vertex({ glm::vec3(p1.x, p2.y, p1.z) * scalefactor, glm::vec3(-n.x, n.y, -n.z), glm::vec2(0.0f, 0.0f) }),

        Vertex({ glm::vec3(p1.x, p1.y, p2.z) * scalefactor, glm::vec3(-n.x, -n.y, n.z), glm::vec2(0.0f, 0.0f) }),
        Vertex({ glm::vec3(p2.x, p1.y, p2.z) * scalefactor, glm::vec3(n.x, -n.y, n.z), glm::vec2(0.0f, 0.0f) }),
        Vertex({ p2 * scalefactor, n, glm::vec2(1.0f, 1.0f) }),
        Vertex({ glm::vec3(p1.x, p2.y, p2.z) * scalefactor, glm::vec3(-n.x, n.y, n.z), glm::vec2(0.0f, 0.0f) }),
    };
    std::vector<unsigned int> indices = {
        0, 1, 2,
        0, 2, 3,
        0, 4, 5,
        0, 5, 1,
        1, 5, 6,
        1, 6, 2,
        2, 6, 7,
        2, 7, 3,
        3, 7, 4,
        3, 4, 0,
        4, 7, 6,
        4, 6, 5
    };
    return Mesh(vertices, indices, mat);
}