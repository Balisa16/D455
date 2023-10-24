#version 330 core

in vec2 fragTexCoord; // Input texture coordinates from vertex shader

out vec4 FragColor;

uniform sampler2D texture; // Texture sampler

void main()
{
    FragColor = texture(texture, fragTexCoord); // Use texture coordinates to sample the texture
}