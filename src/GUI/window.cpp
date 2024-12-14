#include "GUI/window.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <format>
#include <stdexcept>
#include "imgui_internal.h"
#include <ImGuiFileDialog.h>

Window::Window(const std::string& window_name) : name_(window_name)
{
    if (!init_glfw()) {
        // Initialize GLFW and check for failure
        throw std::runtime_error("Failed to initialize GLFW!");
    }

    window_ = glfwCreateWindow(width_, height_, name_.c_str(), nullptr, nullptr);
    if (window_ == nullptr) {
        glfwTerminate();  // Ensure GLFW is cleaned up before throwing
        throw std::runtime_error("Failed to create GLFW window!");
    }

    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);
    glfwSetCursorPosCallback(window_, MouseMoveCallback);
    glfwSetMouseButtonCallback(window_, MouseButtonCallback);
    glfwSetScrollCallback(window_, ScrollCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD!");
    }

    scene_viewer_.initshader(std::string(SHADER_DIR) + "/" + "basic.vs", std::string(SHADER_DIR) + "/" + "basic.fs");

    if (!init_gui()) {
        // Initialize the GUI and check for failure
        glfwDestroyWindow(window_);
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GUI!");
    }

    glEnable(GL_DEPTH_TEST);
}

Window::~Window()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window_);
    glfwTerminate();
}

bool Window::init()
{
    // Placeholder for additional initialization if needed.
    return true;
}

void Window::run()
{
    glfwShowWindow(window_);

    while (!glfwWindowShouldClose(window_)) {
        currentFrame = glfwGetTime();
        if (!glfwGetWindowAttrib(window_, GLFW_VISIBLE) ||
            glfwGetWindowAttrib(window_, GLFW_ICONIFIED))
            glfwWaitEvents();
        else {
            glfwPollEvents();
            ProcessInput(currentFrame - lastFrame);
            render();
        }
        lastFrame = currentFrame;
    }
}

void Window::BuildUI()
{
    if (show_context_menu_) {
        ImGui::OpenPopup("Context Menu");
        show_context_menu_ = false;
    }

    if (ImGui::BeginPopup("Context Menu")) {
        if (ImGui::MenuItem("Delete Model")) {
            scene_viewer_.DeleteSelectedModel();
            model_selected_ = false;
        }
        ImGui::EndPopup();
    }

    if (ImGui::IsPopupOpen("Context Menu"))
        is_context_menu_open_ = true;
    else
        is_context_menu_open_ = false;
    
    if (ImGui::Begin("Setting Solver"))
    {
        ImGui::SliderInt("Scaling Factor for process obstacles(10^n)", &solver_.scalingFactor, 1, 10);

        double min_value = 0.0;
        double max_value = 1.0;
        ImGui::SliderScalar("Area Error", ImGuiDataType_Double, &solver_.hyperparameters[0], &min_value, &max_value);
        ImGui::SliderScalar("Size Error", ImGuiDataType_Double, &solver_.hyperparameters[1], &min_value, &max_value);
        ImGui::SliderScalar("Position Error", ImGuiDataType_Double, &solver_.hyperparameters[2], &min_value, &max_value);
        ImGui::SliderScalar("Adjacency Error", ImGuiDataType_Double, &solver_.hyperparameters[3], &min_value, &max_value);

        ImGui::Spacing();
        ImGui::SliderFloat("Wall Width(x percentage of boundary size)", &scene_viewer_.wallWidth, 0.0f, 0.1f);

        if (ImGui::Button("Solve"))
        {
            solver_.solve();
            if (solver_.floorplan)
                scene_viewer_.setupRooms(solver_.getsolution(), solver_.getboundaryMaxSize());
            else
                scene_viewer_.setupOneRoom(solver_.getsolution(), solver_.getboundary());
        }
    }
    ImGui::End();

    if (ImGui::Begin("Setting Renderer"))
    {
        ImGui::SliderFloat("light position x", &scene_viewer_.lightPos.x, -10.0f, 10.0f);
        ImGui::SliderFloat("light position y", &scene_viewer_.lightPos.y, -10.0f, 10.0f);
        ImGui::SliderFloat("light position z", &scene_viewer_.lightPos.z, -10.0f, 10.0f);

        ImGui::ColorEdit3("Color", glm::value_ptr(scene_viewer_.lightColor), ImGuiColorEditFlags_Float);
    }
    ImGui::End();
    
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Import Object"))
            {
                flag_open_file_dialog_ = true;
            }
            if (ImGui::MenuItem("Import SceneGraph"))
            {
                flag_open_graph_dialog_ = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
    if (flag_open_file_dialog_)
    {
        IGFD::FileDialogConfig config; config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose File", ".obj,.fbx", config);
        if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey"))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
                scene_viewer_.LoadModel(filePathName);
            }
            ImGuiFileDialog::Instance()->Close();
            flag_open_file_dialog_ = false;
        }
    }
    if (flag_open_graph_dialog_)
    {
        IGFD::FileDialogConfig config; config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose File", ".json", config);
        if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey"))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
                solver_.readSceneGraph(filePathName, scene_viewer_.wallWidth);
            }
            ImGuiFileDialog::Instance()->Close();
            flag_open_graph_dialog_ = false;
        }
    }
}

void Window::Render()
{
    ImGuiIO& io = ImGui::GetIO();
    scene_viewer_.renderOtherMesh(camera.GetViewMatrix(), camera.GetProjectionMatrix(io.DisplaySize.x, io.DisplaySize.y), camera.getPosition());
    scene_viewer_.renderModel(camera.GetViewMatrix(), camera.GetProjectionMatrix(io.DisplaySize.x, io.DisplaySize.y), camera.getPosition());
    scene_viewer_.rendersky(glm::mat4(glm::mat3(camera.GetViewMatrix())), camera.GetProjectionMatrix(io.DisplaySize.x, io.DisplaySize.y));
}

bool Window::init_glfw()
{
    glfwSetErrorCallback(
        [](int error, const char* desc) { fprintf(stderr, "GLFW Error %d: %s\n", error, desc); });

    if (!glfwInit()) {
        return false;
    }

#ifdef __APPLE__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    return true;
}

bool Window::init_gui()
{
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // Enable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    float xscale, yscale;
    glfwGetWindowContentScale(window_, &xscale, &yscale);
    // - style
    ImGui::StyleColorsDark();

    io.DisplayFramebufferScale.x = xscale;
    io.DisplayFramebufferScale.x = yscale;

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
#ifdef __APPLE__
    ImGui_ImplOpenGL3_Init("#version 410");
#else
    io.FontGlobalScale = xscale;
    ImGui_ImplOpenGL3_Init("#version 330");
#endif
    return true;
}

void Window::render()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    glfwGetFramebufferSize(window_, &width_, &height_);
    glViewport(0, 0, width_, height_);
    glClearColor(0.35f, 0.45f, 0.50f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Render();
    BuildUI();

    ImGui::Render();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window_);
}

void Window::ProcessInput(float deltaTime) {
    if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboardInput(GLFW_KEY_W, deltaTime);
    if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboardInput(GLFW_KEY_S, deltaTime);
    if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboardInput(GLFW_KEY_A, deltaTime);
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboardInput(GLFW_KEY_D, deltaTime);
    if (glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS)
        camera.ProcessKeyboardInput(GLFW_KEY_Q, deltaTime);
    if (glfwGetKey(window_, GLFW_KEY_E) == GLFW_PRESS)
        camera.ProcessKeyboardInput(GLFW_KEY_E, deltaTime);
}

void Window::MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    Window* instance = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (instance->firstMouse) {
        instance->lastX = xpos;
        instance->lastY = ypos;
        instance->firstMouse = false;
    }

    float xoffset = xpos - instance->lastX;
    float yoffset = instance->lastY - ypos;
    instance->lastX = xpos;
    instance->lastY = ypos;

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        instance->camera.ProcessMouseMovement(xoffset, yoffset);
    }
}

void Window::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    Window* instance = static_cast<Window*>(glfwGetWindowUserPointer(window));
    instance->camera.ProcessMouseScroll(yoffset);
}

void Window::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    Window* instance = static_cast<Window*>(glfwGetWindowUserPointer(window));

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    if (action == GLFW_PRESS)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT && !instance->is_context_menu_open_)
        {
            ImGuiIO& io = ImGui::GetIO();
            instance->model_selected_ = instance->scene_viewer_.SelectModelAt(xpos, ypos,
                instance->camera.GetViewMatrix(),
                instance->camera.GetProjectionMatrix(io.DisplaySize.x, io.DisplaySize.y),
                instance->camera.getPosition());
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT && instance->model_selected_)
        {
            instance->context_menu_x_ = xpos;
            instance->context_menu_y_ = ypos;
            instance->show_context_menu_ = true;
        }
    }
}