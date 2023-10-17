#version 330 core
out vec4 FragColor;

in vec2 TexCoord;

uniform sampler2D texture1;
uniform sampler2D texture2;

void main()
{
   // set 80% container and 20% awesomeface
   FragColor = mix(texture(texture1, TexCoord), texture(texture2, TexCoord), 0.2);
};