#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/postprocess.h>
#include <assimp/texture.h>

#include "Mesh.h"

#include <map>
#include <vector>

unsigned int TextureFromFile(const char *path, const std::string &directory, bool gamma = false);

class Model 
{
public:
    // model data 
    std::vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    std::vector<Mesh>    meshes;
    std::string directory;
    bool gammaCorrection;

    // constructor, expects a filepath to a 3D model.
    Model(std::string const &path, bool gamma = false) : gammaCorrection(gamma)
    {
        minBounds = glm::vec3(FLT_MAX);
        maxBounds = glm::vec3(-FLT_MAX);
        modelmatrix = glm::mat4(1.0f);
        loadModel(path);
        updateAABB(modelmatrix);
    }

    ~Model() {}

    // draws the model, and thus all its meshes
    void Draw(Shader &shader);

    // Model Matrix
    glm::mat4 modelmatrix;
    glm::vec3 minBounds, maxBounds;

    void updateAABB(const glm::mat4& newModelMatrix);
    
private:
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(std::string const &path);

    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene);

    Mesh processMesh(aiMesh *mesh, const aiScene *scene);

    // checks all material textures of a given type and loads the textures if they're not loaded yet.
    // the required info is returned as a Texture struct.
    std::vector<Texture> loadMaterialTexture(aiMaterial *mat, aiTextureType type, std::string typeName);
    Material loadMaterial(aiMaterial *mat);
};