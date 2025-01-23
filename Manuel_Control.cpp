#include <SDL2/SDL.h>
#include "Drone.h"

int main(int argc, char* argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0) {
        return -1;
    }

    // Open the first joystick
    SDL_Joystick* joystick = SDL_JoystickOpen(0);
    if (joystick == nullptr) {
        SDL_Quit();
        return -1;
    }

    // Initialize the drone
    Drone drone;

    bool running = true;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_JOYAXISMOTION) {
                // Get joystick axes
                int x_axis = SDL_JoystickGetAxis(joystick, 0);
                int y_axis = SDL_JoystickGetAxis(joystick, 1);
                int z_axis = SDL_JoystickGetAxis(joystick, 2);
                int yaw_axis = SDL_JoystickGetAxis(joystick, 3);

                // Map joystick input to drone control
                drone.setPitch(y_axis);
                drone.setRoll(x_axis);
                drone.setThrottle(z_axis);
                drone.setYaw(yaw_axis);
            } else if (event.type == SDL_JOYBUTTONDOWN) {
                if (SDL_JoystickGetButton(joystick, 0)) {  // Assuming button 0 is for takeoff
                    drone.takeoff();
                } else if (SDL_JoystickGetButton(joystick, 1)) {  // Assuming button 1 is for landing
                    drone.land();
                }
            }
        }
    }

    // Clean up
    SDL_JoystickClose(joystick);
    SDL_Quit();

    return 0;
}
