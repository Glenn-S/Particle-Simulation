#pragma once
#include "io.h"

enum SCENE { SCENE_1, SCENE_2, SCENE_3, SCENE_4, BONUS_1, BONUS_2 };
SCENE scene = SCENE_1;
bool RESET = false;

template <typename CameraT>
class TurnTableControls {
    public:
        TurnTableControls(
            io::Window &w,
            CameraT &c
        ) : m_window(w)
          , m_camera(c)
        {
          m_window.keyboardCommands()
              | io::Key(GLFW_KEY_ESCAPE,
                  [&](auto const &/*event*/) { m_window.shouldClose(); })
              | io::Key(GLFW_KEY_X,
                  [&](auto const &event) {
                    if (event.action == GLFW_PRESS)
                        xLock = !xLock;
                  })
              | io::Key(GLFW_KEY_Y,
                  [&](auto const &event) {
                    if (event.action == GLFW_PRESS)
                        yLock = !yLock;
                  })
              // maximize/minimize window
              | io::Key(GLFW_KEY_F,
                  [&](auto const &event) {
                  if (event.action ==GLFW_PRESS) {
                      GLFWwindow *window = m_window.m_handle.get();
                      if (!glfwGetWindowAttrib(window, GLFW_MAXIMIZED)) glfwMaximizeWindow(window);
                      else glfwRestoreWindow(window);
                  }
              })
              // key callback scene 1
              | io::Key(GLFW_KEY_1,
                  [&](auto const &event) {
                  if (event.action == GLFW_PRESS) {
                      cout << "1 pressed" << endl;
                      scene = SCENE_1;
                      RESET = true;
                  }
              })
              // key callback scene 2
              | io::Key(GLFW_KEY_2,
                  [&](auto const &event) {
                  if (event.action == GLFW_PRESS) {
                      cout << "2 pressed" << endl;
                      scene = SCENE_2;
                      RESET = true;
                  }
              })
              // key callback scene 3
              | io::Key(GLFW_KEY_3,
                  [&](auto const &event) {
                  if (event.action == GLFW_PRESS) {
                      cout << "3 pressed" << endl;
                      scene = SCENE_3;
                      RESET = true;
                  }
              })
              // key callback scene 4
              | io::Key(GLFW_KEY_4,
                  [&](auto const &event) {
                  if (event.action == GLFW_PRESS) {
                      cout << "4 pressed" << endl;
                      scene = SCENE_4;
                      RESET = true;
                  }
              })
              // key callback bonus 1
              | io::Key(GLFW_KEY_5,
                  [&](auto const &event) {
                  if (event.action == GLFW_PRESS) {
                      cout << "5 pressed" << endl;
                      scene = BONUS_1;
                      RESET = true;
                  }
              })
              // key callback bonus 2
              | io::Key(GLFW_KEY_6,
                  [&](auto const &event) {
                  if (event.action == GLFW_PRESS) {
                      cout << "6 pressed" << endl;
                      scene = BONUS_2;
                      RESET = true;
                  }
              });

          m_window.mouseCommands() | //
              io::MouseButton(     //
                  GLFW_MOUSE_BUTTON_LEFT, [&](auto const &event) {
                    if (event.action == GLFW_PRESS)
                        cursorLocked = true;
                    else
                        cursorLocked = false;

                    if (event.mods == GLFW_MOD_SHIFT)
                        cursorZoom = true;
                    else
                        cursorZoom = false;
                  });

          m_window.cursorCommand() = [&](auto const &event) {
              if (cursorLocked) {
                  if (cursorZoom) {
                      float deltaY = (cursorPosition.y - event.y);
                      float percY = deltaY / m_window.height();
                      m_camera.zoom(-percY * 50.f);
                  } else {
                      float deltaX = cursorPosition.x - event.x;
                      float deltaY = cursorPosition.y - event.y;
                      float percX = deltaX / m_window.width();
                      float percY = deltaY / m_window.height();
                      if (!yLock) m_camera.rotateAroundXPercent(percX);
                      if (!xLock) m_camera.rotateAroundYPercent(percY);
                  }
              }
              cursorPosition = event;
          };

        }

    private:
        io::Window &m_window;
        CameraT &m_camera;
        io::CursorPosition cursorPosition;
        bool cursorLocked = false;
        bool cursorZoom = false;
        bool xLock = false;
        bool yLock = false;
}; // end turntable camera controls
