#include "view/Camera.h"
#include <iostream>

Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(2.5f), MouseSensitivity(0.1f), Zoom(45.0f), Yaw(yaw), Pitch(pitch), WorldUp(up) {
    Position = position;
    updateCameraVectors();
}

Camera::Camera() {
    *this = Camera(glm::vec3(-1.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 0.0f, 0.0f);
}

glm::mat4 Camera::GetViewMatrix() {
    return glm::lookAt(Position, Position + Front, Up);
}

glm::mat4 Camera::GetProjectionMatrix(float width, float height) {
    return glm::perspective(glm::radians(Zoom), width / height, 0.1f, 100.0f);
}

void Camera::ProcessKeyboardInput(int key, float deltaTime) {
    float velocity = MovementSpeed * deltaTime;
    if (key == GLFW_KEY_W)
        Position += Front * velocity;
    if (key == GLFW_KEY_S)
        Position -= Front * velocity;
    if (key == GLFW_KEY_A)
        Position -= Right * velocity;
    if (key == GLFW_KEY_D)
        Position += Right * velocity;
    if (key == GLFW_KEY_Q)
        Position -= Up * velocity;
    if (key == GLFW_KEY_E)
        Position += Up * velocity;
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset) {
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    Yaw += xoffset;
    Pitch += yoffset;

    if (Pitch > 89.0f)
        Pitch = 89.0f;
    if (Pitch < -89.0f)
        Pitch = -89.0f;

    updateCameraVectors();
}

void Camera::ProcessMouseScroll(float yoffset) {
    Zoom -= (float)yoffset;
    if (Zoom < 1.0f)
        Zoom = 1.0f;
    if (Zoom > 45.0f)
        Zoom = 45.0f;
}

void Camera::updateCameraVectors() {
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front);
    Right = glm::normalize(glm::cross(Front, WorldUp));
    Up = glm::normalize(glm::cross(Right, Front));
}