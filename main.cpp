#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "world.hpp"

#include <GL/gl3w.h>    // Initialize with gl3wInit()
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>
#include <random>
#include "shader.hpp"
#include "MyStrategy.hpp"

static void glfw_error_callback(int error, const char* description)
{
    std::cerr << "Glfw Error " << error << ": " << description << std::endl;
}

struct Color
{
    unsigned char r, g, b, a;
};

Color BLACK = {.r = 0, .g = 0, .b = 0, .a = 255};
Color RED = {.r = 255, .g = 0, .b = 0, .a = 255};
Color COL1 = {.r = 0, .g = 200, .b = 200, .a = 255};
Color COL2 = {.r = 200, .g = 200, .b = 0, .a = 255};

struct Vertex
{
    float x;
    float y;
    Color color;
};

struct ParticleRenderData
{
    float x;
    float y;
    float essence;
    uint8_t owner;
    uint8_t particleType;
};

void checkGLError(const std::string &errP)
{
    GLenum err = glGetError();
    if (err != GL_NO_ERROR)
    {
        std::cerr << "GL error " << errP << ": " << err << std::endl;
    }
}

struct RenderBuffer
{
    GLuint vao = 0;
    GLuint bufferObject = 0;
    std::vector<Vertex> data;

    void clear()
    {
        data.clear();
    }

    void buildVao()
    {
        if (vao == 0)
            glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        checkGLError("glBindVertexArray");

        if (bufferObject == 0)
            glGenBuffers(1, &bufferObject);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObject);
        checkGLError("glBindBuffer");

        glEnableVertexAttribArray(0);
        checkGLError("glEnableVertexAttribArray");
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
        checkGLError("glVertexAttribPointer");
        glEnableVertexAttribArray(1);
        checkGLError("glEnableVertexAttribArray");
        glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vertex), reinterpret_cast<const void *>(sizeof(float) * 2));
        checkGLError("glVertexAttribPointer");

        glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(Vertex), data.data(), GL_STREAM_DRAW);
        checkGLError("glBufferData");

        glBindVertexArray(0);
        checkGLError("glBindVertexArray");
    }

    void doRender(const glm::mat4 &MVP, Shader &shader, int type)
    {
        if (data.empty())
            return;

        buildVao();

        glBindVertexArray(vao);
        checkGLError("bind vao");

        glUseProgram(shader.program);

        checkGLError("use program");

        if (shader.uniforms.count("MVP"))
        {
            glUniformMatrix4fv(shader.uniforms["MVP"], 1, GL_FALSE, &MVP[0][0]);

            checkGLError("uniform MVP");
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glDrawArrays(type, 0, data.size());

        checkGLError("draw");

        glBindVertexArray(0);
    }
};

struct LineRenderer : RenderBuffer
{
    void addLine(P from, P to, Color color)
    {
        float x1 = from.x;
        float x2 = to.x;
        float y1 = from.y;
        float y2 = to.y;
        data.push_back({.x = x1, .y = y1, .color = color});
        data.push_back({.x = x2, .y = y2, .color = color});
    }

    void render(const glm::mat4 &MVP, Shader &shader)
    {
        doRender(MVP, shader, GL_LINES);
    }
};

struct ParticleRenderBuffer
{
    GLuint vao = 0;
    GLuint bufferObject = 0;
    std::vector<ParticleRenderData> data;

    void clear()
    {
        data.clear();
    }

    void buildVao()
    {
        if (vao == 0)
            glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        checkGLError("glBindVertexArray");

        if (bufferObject == 0)
            glGenBuffers(1, &bufferObject);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObject);
        checkGLError("glBindBuffer");

        glEnableVertexAttribArray(0);
        checkGLError("glEnableVertexAttribArray");
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleRenderData), 0);
        checkGLError("glVertexAttribPointer");

        glEnableVertexAttribArray(1);
        checkGLError("glEnableVertexAttribArray");
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleRenderData), reinterpret_cast<const void *>(sizeof(float) * 2));
        checkGLError("glVertexAttribPointer");

        glEnableVertexAttribArray(2);
        checkGLError("glEnableVertexAttribArray");
        glVertexAttribIPointer(2, 1, GL_UNSIGNED_BYTE, sizeof(ParticleRenderData), reinterpret_cast<const void *>(sizeof(float) * 3));
        checkGLError("glVertexAttribIPointer");

        glEnableVertexAttribArray(3);
        checkGLError("glEnableVertexAttribArray");
        glVertexAttribIPointer(3, 1, GL_UNSIGNED_BYTE, sizeof(ParticleRenderData), reinterpret_cast<const void *>(sizeof(float) * 3 + 1));
        checkGLError("glVertexAttribIPointer");

        glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(ParticleRenderData), data.data(), GL_STREAM_DRAW);
        checkGLError("glBufferData");

        glBindVertexArray(0);
        checkGLError("glBindVertexArray");
    }

    void doRender(const glm::mat4 &MVP, Shader &shader, int type)
    {
        if (data.empty())
            return;

        buildVao();

        glBindVertexArray(vao);
        checkGLError("bind vao");

        glUseProgram(shader.program);

        checkGLError("use program");

        if (shader.uniforms.count("MVP"))
        {
            glUniformMatrix4fv(shader.uniforms["MVP"], 1, GL_FALSE, &MVP[0][0]);

            checkGLError("uniform MVP");
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glDrawArrays(type, 0, data.size());

        checkGLError("draw");

        glBindVertexArray(0);
    }
};

struct ParticleRenderer : ParticleRenderBuffer
{
    void addParticle(Particle &p)
    {
        double maxEssence = gameParameters.particleParameters[p.particleType].maxEssenceCapacity;

        data.push_back({.x = (float) p.pos.x, .y = (float) p.pos.y, .essence = (float) (p.essence / maxEssence),
                        .owner = p.owner, .particleType = (uint8_t) p.particleType});
    }

    void render(const glm::mat4 &MVP, Shader &shader)
    {
        doRender(MVP, shader, GL_POINTS);
    }
};

struct Zoom {
    double zoom = 1.0;
    P zoomCenter = P(0.0, 0.0);
    P mousePos = P(0.0, 0.0);

    void imguiInputProcessing(double wid, double heig) {
        ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureMouse)
        {
            if (io.MouseWheel != 0.0) {
                float w = io.MouseWheel / 8.0;
                P zoomPoint = zoomCenter + (mousePos - P(0.5, 0.5)) / zoom;

                if (w > 0) {
                    for (int i = 0; i < w; ++i)
                        zoom *= 1.2;
                } else {
                    for (int i = 0; i < -w; ++i)
                        zoom /= 1.2;
                }

                if (zoom < 0.01)
                    zoom = 0.01;

                zoomCenter = zoomPoint - (mousePos - P(0.5, 0.5)) / zoom;

            }
        }

        P oldPos = mousePos;
        mousePos = P(ImGui::GetMousePos().x, ImGui::GetMousePos().y) / P(wid, heig);
//        std::cout << mousePos.x << " " << mousePos.y << std::endl;

        if (!io.WantCaptureMouse && ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            P delta = mousePos - oldPos;
            zoomCenter -= delta / zoom;
        }

//        {
//            if (zoomCenter.x < -0.5)
//                zoomCenter.x = -0.5;
//
//            if (zoomCenter.x > 0.5)
//                zoomCenter.x = 0.5;
//
//            if (zoomCenter.y < -0.5)
//                zoomCenter.y = -0.5;
//
//            if (zoomCenter.y > 0.5)
//                zoomCenter.y = 0.5;
//        }
    }
};

void generateWorld(World &w)
{
    const double maxSpeed = 5.0;
    uint32_t particlesNumber = w.width * w.width / 50;

    {
        Particle p;
        p.pos = P(w.width / 4.0, w.width / 2.0);
        p.vel = P(0.0, 0.0);
        p.owner = 1;
        p.particleType = ParticleType::SOUL;
        p.essence = gameParameters.particleParameters[p.particleType].maxEssenceCapacity;
        w.particles.push_back(p);
    }

    {
        Particle p;
        p.pos = P(3.0 * w.width / 4.0, w.width / 2.0);
        p.vel = P(0.0, 0.0);
        p.owner = 2;
        p.particleType = ParticleType::SOUL;
        p.essence = gameParameters.particleParameters[p.particleType].maxEssenceCapacity;
        w.particles.push_back(p);
    }

    std::seed_seq seed{1, 2, 3, 4, 5, 6, 7, 8};
    std::mt19937 mt(seed);
    std::uniform_real_distribution<> disPos(0.0, (double) w.width);
    std::uniform_real_distribution<> disVel(-maxSpeed, maxSpeed);

    std::vector<bool> grid;
    grid.resize(w.width*w.width);
    if (w.width % 2 == 0)
    {
        int w05 = w.width / 2;
        grid[w05 * w.width + w05] = true;
    }
    else
    {
        int w05 = w.width / 2;
        grid[w05 * w.width + w05] = true;
        grid[w05 * w.width + w05 + 1] = true;
        grid[w05 * w.width + w05] = true;
        grid[(w05 + 1) * w.width + w05 + 1] = true;
    }
    auto testEmpty = [&](IP p){
        for (int x = -1; x <= 1; ++x)
        {
            for (int y = -1; y <= 1; ++y)
            {
                int index = ((p.y + y + w.width) % w.width ) * w.width + (p.x + x + w.width) % w.width;
                if (grid[index])
                    return false;
            }
        }

        return true;
    };

    for (size_t i = 0; i < particlesNumber / 2; ++i)
    {
        Particle p1;
        do {
            p1.pos = P(disPos(mt), disPos(mt));
        } while (!testEmpty(IP(p1.pos)));
        p1.vel = P(disVel(mt), disVel(mt));
        w.particles.push_back(p1);

        IP ip = IP(p1.pos);
        grid[ip.y * w.width + ip.x] = true;

        Particle p2;
        p2.pos = P(w.width, w.width) - p1.pos;
        p2.vel = p1.vel * (-1.0);
        w.particles.push_back(p2);

        ip = IP(p2.pos);
        grid[ip.y * w.width + ip.x] = true;
    }

    std::cout << "COUNT " << w.particles.size() << std::endl;
}

int main()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

#if __APPLE__
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
    glfwWindowHint(GLFW_SAMPLES, 1);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
#endif

    GLFWwindow* window = glfwCreateWindow(1024, 1024, "Cell wars", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    bool err = gl3wInit() != 0;

    if (err)
    {
        std::cerr << "Failed to initialize OpenGL loader!" << std::endl;
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    std::string home_dir = getenv("HOME");
    if (!home_dir.empty())
    {
        home_dir += "/.cell_wars.ini";
        io.IniFilename = home_dir.c_str();
    }

    ImGui::StyleColorsLight();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);


    World w;
    generateWorld(w);
    Grid g{w.width};

    MyStrategy strategy1{1};
    EmptyStrategy strategy2{2};

    Shader simpleShader = {"simple"};
    initSimpleShader(simpleShader);

    Shader particleShader = {"particle"};
    initParticleShader(particleShader);

    LineRenderer lineRenderer;
    ParticleRenderer particleRenderer;

    Zoom zoom;
    float wid = g.w;

    bool started = false;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int display_w, display_h;
        glfwMakeContextCurrent(window);
        glfwGetFramebufferSize(window, &display_w, &display_h);

        glViewport(0, 0, display_w, display_h);
        glClearColor(1.0, 1.0, 1.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        glClear(GL_DEPTH_BUFFER_BIT);
        glClear(GL_STENCIL_BUFFER_BIT);

        zoom.imguiInputProcessing(display_w, display_h);

        glm::mat4 mvp;
        float windowCoordsW;
        float windowCoordsH;
        if (display_w >= display_h)
        {
            float ratio = (float) display_w / (float) display_h;
            windowCoordsW = wid * ratio;
            windowCoordsH = wid;
            mvp = glm::ortho(-0.5f*ratio*wid, 0.5f*ratio*wid, -0.5f*wid, 0.5f*wid, 100.0f, -100.0f);
        }
        else
        {
            float ratio = (float) display_h / (float) display_w;
            windowCoordsW = wid;
            windowCoordsH = wid * ratio;
            mvp = glm::ortho(-0.5f*wid, 0.5f*wid, -0.5f*ratio*wid, 0.5f*ratio*wid, 100.0f, -100.0f);
        }

        mvp = glm::scale(mvp, glm::vec3(zoom.zoom, zoom.zoom, 1.0));
        mvp = glm::translate(mvp, glm::vec3(-(zoom.zoomCenter.x) * windowCoordsW, (zoom.zoomCenter.y) * windowCoordsH, 1.0));
        mvp = glm::translate(mvp, glm::vec3(-0.5 * wid, -0.5 * wid, 1.0));

        lineRenderer.clear();
        lineRenderer.addLine(P{0.0, 0}, P{0.0, wid}, BLACK);
        lineRenderer.addLine(P{0.0, wid}, P{wid, wid}, BLACK);
        lineRenderer.addLine(P{wid, wid}, P{wid, 0.0}, BLACK);
        lineRenderer.addLine(P{wid, 0.0}, P{0.0, 0.0}, BLACK);

        lineRenderer.render(mvp, simpleShader);

        if (ImGui::IsKeyPressed(ImGuiKey_Space))
        {
            started = !started;
        }

        if (started || ImGui::IsKeyPressed(ImGuiKey_RightArrow))
        {
            std::unordered_map<uint16_t, Actions> actions;
            strategy1.generateActions(w, actions[strategy1.playerId]);
            strategy2.generateActions(w, actions[strategy2.playerId]);
            simulate(g, w, gameParameters.tick, actions);
        }

        particleRenderer.clear();
        int i = 0;
        for (Particle &p : w.particles)
        {
            particleRenderer.addParticle(p);
            ++i;
        }

        particleRenderer.render(mvp, particleShader);
        lineRenderer.render(mvp, simpleShader);

        ImGui::EndFrame();

        glfwMakeContextCurrent(window);

        glfwSwapBuffers(window);
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
