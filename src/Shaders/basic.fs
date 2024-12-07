#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

uniform vec3 ambient;
uniform vec3 diffuse;
uniform vec3 specular;
uniform bool hasTexture;

uniform vec3 viewPos;
uniform vec3 lightPos;

uniform sampler2D texture_diffuse1;

void main()
{    
    // 环境光照
    float ambientStrength = 0.1;
    vec3 ambientColor = ambientStrength * ambient;
  
    // 漫反射光照
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuseColor = diff * diffuse;
    
    // 镜面光照
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    vec3 halfwayDir = normalize(lightDir + viewDir);  // Blinn-Phong 用到的半向量
    float spec = pow(max(dot(norm, halfwayDir), 0.0), 32.0);
    vec3 specularColor = specularStrength * spec * specular;

    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
    if (hasTexture)
        FragColor = texture(texture_diffuse1, TexCoords) * FragColor;
}
