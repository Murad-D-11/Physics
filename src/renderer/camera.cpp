#include "camera.h"
#include <algorithm>
#include <cmath>

Camera::Camera(float distance, float yawDegrees, float pitchDegrees):  
    target(0.0f, 0.0f, 0.0f),
    distance(distance),
    yaw(yawDegrees),
    pitch(pitchDegrees),
    fovDegrees(45.0f),
    nearPlane(0.1f), // object closer than 0.1 units to the camera won't be drawn
    farPlane(100.0f) // objects farther than 100 units from camera won't be drawn 
    {}

void Camera::processMouseDrag(float deltaX, float deltaY) {
    yaw += deltaX * mouseSensitivity; // horizontal mouse movement rotates the camera around the vertical axis
    
    // additionally: scaling with mouseSensitivity converts the pixels moved into degrees

    pitch += deltaY * mouseSensitivity; // vertical mouse movement tilts the camera up/down
    pitch = std::clamp(pitch, minPitch, maxPitch); // clamp prevents pitch exceeding from its range, preventing the camera to flip upside-down
}

void Camera::processScroll(float yOffset) {
    distance -= yOffset * scrollSensitivity; // same effect here, but with scrolling triggering zoom
    distance = std::clamp(distance, minDistance, maxDistance); // limits zoom intensity
}

glm::vec3 Camera::getPosition() const {
    const float yawRad = glm::radians(yaw);
    const float pitchRad = glm::radians(pitch);

    glm::vec3 offset;

    offset.x = distance * std::cos(pitchRad) * std::cos(yawRad); // spherical to cartesian formula: here, x depends on both angles (shrinks towards 0 as pitch approaches 90 degrees)
    offset.y = distance * std::sin(pitchRad); // height depends only on pitch: looking straight up/down moves y the most
    offset.z = distance * std::cos(pitchRad) * std::sin(yawRad); // z is the depth component; using sin(yaw) instead of cos(yaw) so that as yaw increases, the camera sweeps around in a circle

    return target + offset;
}

/**
 * Builds a view matrix that transforms world coordinates as if the camera were sitting at 'eye',
 * looking toward 'center', with (0, 1, 0) defining which way is "up" on screen.
 */
glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(getPosition(), target, glm::vec3(0.0f, 1.0f, 0.0f));
}

/**
 * Builds a perspective projection matrix
 * Aspect is width/height of the viewport, so the image is not stretched,
 * and near/far define the visible depth range.
 */
glm::mat4 Camera::getProjectionMatrix(float aspectRatio) const {
    return glm::perspective(glm::radians(fovDegrees), aspectRatio, nearPlane, farPlane);
}