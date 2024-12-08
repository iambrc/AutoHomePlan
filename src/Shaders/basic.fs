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
uniform vec3 lightColor;
uniform vec3 highlightColor;

uniform sampler2D texture_diffuse1;

void main()
{    
    vec3 norm = normalize(Normal);
    
    vec3 lightDir = normalize(lightPos - FragPos);
    vec3 viewDir = normalize(viewPos - FragPos);

    vec3 halfwayDir = normalize(lightDir + viewDir);

    vec3 ambientLight = ambient * lightColor;
    
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuseLight = diff * diffuse * lightColor;

    float spec = pow(max(dot(norm, halfwayDir), 0.0), 32.0);
    vec3 specularLight = spec * specular * lightColor;

    vec3 texColor = hasTexture ? texture(texture_diffuse1, TexCoords).xyz : vec3(1.0);

    vec3 result = ambientLight + diffuseLight + specularLight;

    FragColor = vec4(result * texColor * highlightColor, 1.0);
}
