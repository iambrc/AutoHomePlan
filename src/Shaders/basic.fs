#version 330 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D texture1;

void main()
{
    FragColor = texture(texture1, TexCoord);
    FragColor = vec4(1.0f, 0.3f, 0.2f, 1.0f);
}
