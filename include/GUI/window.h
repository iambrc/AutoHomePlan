#pragma once

#include <string>
#include <iostream>
#include "view/SceneViewer.h"
#include "view/Camera.h"
#include "Components/Solver.h"

class Window {
public:
    // Constructor that sets the window's title.
    explicit Window(const std::string& window_name);

    virtual ~Window();

    // Initializes the window and its dependencies (GLFW, GLAD, ImGui, etc.).
    bool init();

    // Enters the main rendering loop.
    void run();

protected:
    void BuildUI();
    void Render();
    // Initializes GLFW library.
    bool init_glfw();

    // Initializes the GUI library (e.g., Dear ImGui).
    bool init_gui();

    // Handles the rendering of each frame.
    void render();

    std::string name_;             // Name (title) of the window.
    GLFWwindow* window_ = nullptr; // Pointer to the GLFW window.
    int width_ = 1920;             // Width of the window.
    int height_ = 1080;            // Height of the window.

private:
    void ProcessInput(float deltaTime);
    static void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);


    bool flag_open_file_dialog_ = false; // Flag to open file dialog.
    bool show_context_menu_ = false;      // Flag to show the context menu.
    bool is_context_menu_open_ = false;   // Flag to indicate if the context menu is open.
    bool model_selected_ = false;         // Flag to indicate if a model is selected.
    float context_menu_x_ = 0.0f;         // X-coordinate of the context menu.
    float context_menu_y_ = 0.0f;         // Y-coordinate of the context menu.

    SceneViewer scene_viewer_;           // Scene viewer object.
    Solver solver_;                     // Solver object.

    Camera camera;
    float lastX;
    float lastY;
    bool firstMouse;

    float lastFrame;
    float currentFrame;

    Shader shader, skyboxshader;
};
