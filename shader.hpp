#ifndef CELLS_SHADER_HPP
#define CELLS_SHADER_HPP

#include <map>
#include <string>
#include <GL/gl3w.h>

struct Shader
{
    Shader(const std::string &name) : shaderName(name) {}
    GLuint program = 0;
    std::map<std::string, GLint> uniforms;
    std::string shaderName;

    bool buildShaderProgram(const char *vsText, const char *gsText, const char *fsText);
};

void initSimpleShader(Shader &shader);
void initParticleShader(Shader &shader);

#endif //CELLS_SHADER_HPP
