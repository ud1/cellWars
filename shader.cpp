#include "shader.hpp"
#include <vector>
#include <iostream>

const char *SIMPLE_VS = R"TEXT(#version 330

layout(location = 0) in vec2 fposition;
layout(location = 1) in vec4 fcolor;
uniform mat4 MVP;
out vec4 color;

void main()
{
	gl_Position = MVP * vec4(fposition.xy, 0, 1);
    color = fcolor;
}
)TEXT";

const char *SIMPLE_FS = R"TEXT(#version 330
in vec4 color;
layout(location = 0) out vec4 outputColor;

void main()
{
    outputColor = color;
}
)TEXT";


const char *PARTICLE_VS = R"TEXT(#version 330

layout(location = 0) in vec2 fposition;
layout(location = 1) in float fessence;
layout(location = 2) in int fowner;
layout(location = 3) in int fparticleType;
out float gessence;
out int gowner;
out int gparticleType;

void main()
{
	gl_Position = vec4(fposition.xy, 0, 1);
    gessence = fessence;
    gowner = fowner;
    gparticleType = fparticleType;
}
)TEXT";

const char *PARTICLE_GS = R"TEXT(#version 330
layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform mat4 MVP;
in float gessence[];
in int gowner[];
in int gparticleType[];

out float essence;
flat out int owner;
flat out int particleType;
out vec2 texCoord;

void main()
{
    essence = gessence[0];
    owner = gowner[0];
    particleType = gparticleType[0];

    vec4 p = vec4(gl_in[0].gl_Position.xy, 0, 1);

	gl_Position = MVP * (p + vec4(-0.5, 0.5, 0.0, 0.0));
    texCoord = vec2(-1, 1);
	EmitVertex();

    gl_Position = MVP * (p + vec4(0.5, 0.5, 0.0, 0.0));
    texCoord = vec2(1, 1);
	EmitVertex();

    gl_Position = MVP * (p + vec4(-0.5, -0.5, 0.0, 0.0));
    texCoord = vec2(-1, -1);
	EmitVertex();

    gl_Position = MVP * (p + vec4(0.5, -0.5, 0.0, 0.0));
    texCoord = vec2(1, -1);
	EmitVertex();

    EndPrimitive();
}
)TEXT";

const char *PARTICLE_FS = R"TEXT(#version 330
in float essence;
flat in int owner;
flat in int particleType;
in vec2 texCoord;
layout(location = 0) out vec4 outputColor;

#define NEUTRAL 0
#define SOUL 1
#define MOVER 2
#define ACCUMULATOR 3
#define SHELL 4
#define EXPLOSIVE 5
#define REGENERATOR 6
#define DEVOUR 7

const float PI = 3.1415927;
const float RAD = 0.8;

void main()
{
    float rad = dot(texCoord, texCoord);
    if (rad > 1)
    {
        outputColor = vec4(1, 1, 1, 0);
    }
    else
    {
        outputColor = particleType == NEUTRAL ? vec4(0, 0, 0, 1) : owner == 1 ? vec4(0.5, 0, 0, 1) : vec4(0, 0, 0.5, 1);

        if (rad < RAD*RAD)
        {
            float y = texCoord.y*0.5/RAD + 0.5;
            if (y > essence)
                outputColor = vec4(0.5, 0.5, 0.5, 1);

            if (particleType == NEUTRAL)
            {
                outputColor = vec4(0.85, 0.85, 0.85, 1.0);
            }
            else if (particleType == SOUL)
            {
                if (abs(texCoord.x) + abs(texCoord.y) > 0.7)
                {
                    outputColor = vec4(1, 1, 1, 0);
                }
            }
            else if (particleType == MOVER)
            {
                if (mod(abs(texCoord.x) + texCoord.y*0.7 + 0.1, 0.5) > 0.25)
                {
                    outputColor = vec4(1, 1, 1, 0);
                }
            }
            else if (particleType == ACCUMULATOR)
            {
                //if (abs(texCoord.y*5.0 - asin(sin(texCoord.x*5))) > 1.0 - abs(texCoord.x)*0.5)
                if (abs(clamp(texCoord.x, -0.2, 0.2)*3.0 -texCoord.x*0.9 + texCoord.y) > 0.2)
                {
                    outputColor = vec4(1, 1, 1, 0);
                }
            }
            else if (particleType == SHELL)
            {
                if (mod(texCoord.x + texCoord.y, 0.5) > 0.15 && mod(texCoord.x - texCoord.y, 0.5) > 0.15)
                {
                    outputColor = vec4(1, 1, 1, 0);
                }
            }
            else if (particleType == EXPLOSIVE)
            {
                float an = PI/10.0;
                float en = PI/5.0;
                vec2 racs = vec2(cos(an),sin(an));
                vec2 ecs =   vec2(cos(en),sin(en));

                float bn = mod(atan(texCoord.x, texCoord.y), 2.0*an) - an;
                vec2 p = length(texCoord)*vec2(cos(bn),sin(bn));

                // line sdf
                p -= racs;
                p += -ecs*dot(p,ecs);
                if (p.x > 0.0)
                    outputColor = vec4(1, 1, 1, 0);
            }
            else if (particleType == REGENERATOR)
            {
                if (rad > 0.6 || abs(texCoord.x) > 0.2 && abs(texCoord.y) > 0.2)
                {
                    outputColor = vec4(1, 1, 1, 0);
                }
            }
            else if (particleType == DEVOUR)
            {
                float angle = rad * 10;
                mat2 m = mat2(cos(angle),-sin(angle),
                            sin(angle),cos(angle));
                vec2 rotated = m * texCoord;
                if (abs(rotated.x) > 0.2)
                {
                    outputColor = vec4(1, 1, 1, 0);
                }
            }
        }
    }
}
)TEXT";

static GLuint createShader(GLenum eShaderType, const char *dataStr, const std::string &shaderName)
{
    GLuint shader = glCreateShader(eShaderType);
    glShaderSource(shader, 1, &dataStr, NULL);

    glCompileShader(shader);

    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

    if (status == GL_FALSE)
    {
        GLint infoLogLength;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLength);

        GLchar strInfoLog[4096];
        glGetShaderInfoLog(shader, infoLogLength, NULL, strInfoLog);

        char strShaderType[16];

        switch (eShaderType)
        {
            case GL_VERTEX_SHADER:
                sprintf(strShaderType, "vertex");
                break;

            case GL_GEOMETRY_SHADER:
                sprintf(strShaderType, "geometry");
                break;

            case GL_FRAGMENT_SHADER:
                sprintf(strShaderType, "fragment");
                break;
        }

        std::cerr << "Compile failure in " <<  strShaderType << " shader(" << shaderName.c_str() << "):\n" << strInfoLog;
        return -1;
    }
    else
    {
        //std::cerr << "Shader compiled sucessfully! " << shaderName.c_str();
    }

    return shader;
}

bool Shader::buildShaderProgram(const char *vsText, const char *gsText, const char *fsText)
{
    GLuint vertexShader;
    GLuint geometryShader;
    GLuint fragmentShader;

    vertexShader = createShader(GL_VERTEX_SHADER, vsText, shaderName);
    if (gsText)
        geometryShader = createShader(GL_GEOMETRY_SHADER, gsText, shaderName);
    fragmentShader = createShader(GL_FRAGMENT_SHADER, fsText, shaderName);

    program = glCreateProgram();

    glAttachShader(program, vertexShader);
    if (gsText)
        glAttachShader(program, geometryShader);
    glAttachShader(program, fragmentShader);

    glLinkProgram(program); //linking!

    //error checking
    GLint status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);

    if (status == GL_FALSE)
    {
        GLint infoLogLength;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLength);

        GLchar strInfoLog[4096];
        glGetProgramInfoLog(program, infoLogLength, NULL, strInfoLog);
        std::cerr << "Shader linker failure " << shaderName.c_str() << ": " << strInfoLog;
        return false;
    }
    else
    {
        //std::cerr << "Shader linked sucessfully! " << shaderName.c_str();
    }

    /*glDetachShader(program, vertexShader);
    glDetachShader(program, fragmentShader);*/

    GLint nuni;
    glGetProgramiv(program, GL_ACTIVE_UNIFORMS, &nuni);
    char name[256];

    for (GLint i = 0; i < nuni; ++i)
    {
        GLint size;
        GLenum type;

        glGetActiveUniform(program, i, sizeof(name), NULL, &size, &type, name);
        GLint location = glGetUniformLocation(program, name);
        uniforms[name] = location;
        //std::cerr << "Shader " << shaderName.c_str() << ": " << name << " " << location;
    }

    return true;
}

void initSimpleShader(Shader &shader)
{
    if (!shader.buildShaderProgram(SIMPLE_VS, nullptr, SIMPLE_FS))
        throw std::runtime_error("Build simple shader error");
}

void initParticleShader(Shader &shader)
{
    if (!shader.buildShaderProgram(PARTICLE_VS, PARTICLE_GS, PARTICLE_FS))
        throw std::runtime_error("Build particle shader error");
}

