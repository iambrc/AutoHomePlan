#include "view/Model.h"
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

unsigned int TextureFromFile(const char *path, const std::string &directory, bool gamma)
{
    std::string filename = std::string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

void Model::Draw(Shader &shader)
{
    for(unsigned int i = 0; i < meshes.size(); i++)
        meshes[i].Draw(shader);
}

void Model::loadModel(std::string const &path)
{
    // read file via ASSIMP
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
    // check for errors
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
    {
        std::cerr << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        return;
    }
    // retrieve the directory path of the filepath
    directory = path.substr(0, path.find_last_of('/'));
    // process ASSIMP's root node recursively
    processNode(scene->mRootNode, scene);
    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[i];
        for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
            aiVector3D vertex = mesh->mVertices[j];
            minBounds = glm::min(minBounds, glm::vec3(vertex.x, vertex.y, vertex.z));
            maxBounds = glm::max(maxBounds, glm::vec3(vertex.x, vertex.y, vertex.z));
        }
    }
    modelmatrix = glm::translate(modelmatrix, -(minBounds + maxBounds) / 2.0f);
    float scale = 2.0f / glm::max(maxBounds.x - minBounds.x, glm::max(maxBounds.y - minBounds.y, maxBounds.z - minBounds.z));
    modelmatrix = glm::scale(modelmatrix, glm::vec3(scale));
}

void Model::processNode(aiNode *node, const aiScene *scene)
{
    // process each mesh located at the current node
    for(unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        // the node object only contains indices to index the actual objects in the scene. 
        // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes.push_back(processMesh(mesh, scene));
    }
    // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
    for(unsigned int i = 0; i < node->mNumChildren; i++)
    {
        processNode(node->mChildren[i], scene);
    }
}

Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene)
{
    // data to fill
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;

    // walk through each of the mesh's vertices
    for(unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        Vertex vertex;
        glm::vec3 vector; // we declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
        // positions
        vector.x = mesh->mVertices[i].x;
        vector.y = mesh->mVertices[i].y;
        vector.z = mesh->mVertices[i].z;
        vertex.Position = vector;

        // normals
        if (mesh->HasNormals())
        {
            vector.x = mesh->mNormals[i].x;
            vector.y = mesh->mNormals[i].y;
            vector.z = mesh->mNormals[i].z;
            vertex.Normal = vector;
        }
        // texture coordinates
        if(mesh->mTextureCoords[0]) // does the mesh contain texture coordinates?
        {
            glm::vec2 vec;
            // a vertex can contain up to 8 different texture coordinates. We thus make the assumption that we won't 
            // use models where a vertex can have multiple texture coordinates so we always take the first set (0).
            vec.x = mesh->mTextureCoords[0][i].x; 
            vec.y = mesh->mTextureCoords[0][i].y;
            vertex.TexCoords = vec;
            // tangent
            vector.x = mesh->mTangents[i].x;
            vector.y = mesh->mTangents[i].y;
            vector.z = mesh->mTangents[i].z;
            vertex.Tangent = vector;
            // bitangent
            vector.x = mesh->mBitangents[i].x;
            vector.y = mesh->mBitangents[i].y;
            vector.z = mesh->mBitangents[i].z;
            vertex.Bitangent = vector;
        }
        else
            vertex.TexCoords = glm::vec2(0.0f, 0.0f);
        vertices.push_back(vertex);
    }

    // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
    for(unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace face = mesh->mFaces[i];
        // retrieve all indices of the face and store them in the indices vector
        for(unsigned int j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);        
    }
    // process materials
    aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];    
    // we assume a convention for sampler names in the shaders. Each diffuse texture should be named
    // as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER. 
    // Same applies to other texture as the following list summarizes:
    // diffuse: texture_diffuseN
    // specular: texture_specularN
    // normal: texture_normalN
    // 1. diffuse maps
    std::vector<Texture> diffuseMaps = loadMaterialTexture(material, aiTextureType_DIFFUSE, "texture_diffuse");
    textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
    // 2. specular maps
    std::vector<Texture> specularMaps = loadMaterialTexture(material, aiTextureType_SPECULAR, "texture_specular");
    textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    // 3. normal maps
    std::vector<Texture> normalMaps = loadMaterialTexture(material, aiTextureType_HEIGHT, "texture_normal");
    textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
    // 4. height maps
    std::vector<Texture> heightMaps = loadMaterialTexture(material, aiTextureType_AMBIENT, "texture_height");
    textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
    
    // return a mesh object created from the extracted mesh data
    Material mat = loadMaterial(material);
    mat.textures = textures;
    return Mesh(vertices, indices, mat);
}

std::vector<Texture> Model::loadMaterialTexture(aiMaterial *mat, aiTextureType type, std::string typeName)
{
    std::vector<Texture> textures;

    for(unsigned int i = 0; i < mat->GetTextureCount(type); i++)
    {
        aiString str;
        mat->GetTexture(type, i, &str);
        // check if texture was loaded before and if so, continue to next iteration: skip loading a new texture
        bool skip = false;
        for(unsigned int j = 0; j < textures_loaded.size(); j++)
        {
            if(std::strcmp(textures_loaded[j].path.data(), str.C_Str()) == 0)
            {
                textures.push_back(textures_loaded[j]);
                skip = true; // a texture with the same filepath has already been loaded, continue to next one. (optimization)
                break;
            }
        }
        if(!skip)
        {   // if texture hasn't been loaded already, load it
            Texture texture;
            texture.id = TextureFromFile(str.C_Str(), this->directory);
            texture.type = typeName;
            texture.path = str.C_Str();
            //std::cout << "Texture Path: " << str.C_Str() << std::endl;
            textures.push_back(texture);
            textures_loaded.push_back(texture);  // store it as texture loaded for entire model, to ensure we won't unnecessary load duplicate textures.
        }
    }
    return textures;
}

Material Model::loadMaterial(aiMaterial *mat)
{
    Material material;
    aiColor3D color(0.f, 0.f, 0.f);

    if (mat->Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS) {
        material.diffuseColor = glm::vec3(color.r, color.g, color.b);
    } else {
        material.diffuseColor = glm::vec3(0.f, 0.f, 0.f);
    }

    if (mat->Get(AI_MATKEY_COLOR_SPECULAR, color) == AI_SUCCESS) {
        material.specularColor = glm::vec3(color.r, color.g, color.b);
    } else {
        material.specularColor = glm::vec3(0.f, 0.f, 0.f);
    }

    if (mat->Get(AI_MATKEY_COLOR_AMBIENT, color) == AI_SUCCESS) {
        material.ambientColor = glm::vec3(color.r, color.g, color.b);
    } else {
        material.ambientColor = glm::vec3(0.f, 0.f, 0.f);
    }

    float shininess;
    if (mat->Get(AI_MATKEY_SHININESS, shininess) == AI_SUCCESS) {
        material.shininess = shininess;
    } else {
        material.shininess = 0.0f;
    }

    return material;
}

void Model::updateAABB(const glm::mat4& newModelMatrix)
{
    glm::vec3 corners[8] = {
        glm::vec3(minBounds.x, minBounds.y, minBounds.z),
        glm::vec3(minBounds.x, minBounds.y, maxBounds.z),
        glm::vec3(minBounds.x, maxBounds.y, minBounds.z),
        glm::vec3(minBounds.x, maxBounds.y, maxBounds.z),
        glm::vec3(maxBounds.x, minBounds.y, minBounds.z),
        glm::vec3(maxBounds.x, minBounds.y, maxBounds.z),
        glm::vec3(maxBounds.x, maxBounds.y, minBounds.z),
        glm::vec3(maxBounds.x, maxBounds.y, maxBounds.z)
    };

    glm::vec3 newMinBounds = glm::vec3(INFINITY);
    glm::vec3 newMaxBounds = glm::vec3(-INFINITY);

    for (int i = 0; i < 8; i++) {
        glm::vec4 transformedCorner = newModelMatrix * glm::vec4(corners[i], 1.0f);
        newMinBounds = glm::min(newMinBounds, glm::vec3(transformedCorner));
        newMaxBounds = glm::max(newMaxBounds, glm::vec3(transformedCorner));
    }
    minBounds = newMinBounds;
    maxBounds = newMaxBounds;
}