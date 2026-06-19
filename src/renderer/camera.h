#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

/**
 * Orbit-style camera that always looks at a fixed target (the origin).
 * It is positioned using spherical coordinates (yaw, pitch, distance) around that target,
 * so moving the camera really means changing those three values.
 * 
 * Usage:
 *  - call processMouseDrag() while the mouse button is held and moving
 *  - call processScroll() on scroll wheel input to zoom in/out
 *  - call getViewMatrix() / getProjectionMatrix() each frame
 */
class Camera {
    public:
        explicit Camera(float distance = 4.0f, float yawDegrees = -90.0f, float pitchDegrees = 20.0f);
        
        void processMouseDrag(float deltaX, float deltaY);
        void processScroll(float yOffset);

        glm::mat4 getViewMatrix() const;
        glm::mat4 getProjectionMatrix(float aspectRatio) const;
        glm::vec3 getPosition() const;

    private:
        glm::vec3 target; // point the camera orbits around (the round)
        float distance; // distance from target
        float yaw; // horizontal angle around the target, degrees
        float pitch; // vertical angle around the target, degrees

        float fovDegrees;
        float nearPlane;
        float farPlane;

        static constexpr float minDistance = 1.5f;
        static constexpr float maxDistance = 20.0f;
        static constexpr float minPitch = -89.0f;
        static constexpr float maxPitch = 89.0f;
        static constexpr float mouseSensitivity = 0.3f;
        static constexpr float scrollSensitivity = 0.5f;
};