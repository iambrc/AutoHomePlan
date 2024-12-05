#pragma once

#include <GLFW/glfw3.h>
#include <string>
#include <iostream>

class Window {
public:
    // Constructor that sets the window's title.
    explicit Window(const std::string& window_name);

    virtual ~Window();

    // Initializes the window and its dependencies (GLFW, GLAD, ImGui, etc.).
    bool init();

    // Enters the main rendering loop.
    void run();

    bool flag_open_file_dialog = false; // Flag to open file dialog.

protected:
    // Virtual draw function to be implemented by derived classes for custom
    // rendering.
    virtual void BuildUI();

    virtual void Render()
    {
        // Placeholder for custom rendering code.
    }


protected:
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
};
