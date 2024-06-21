#include "planar_quadrotor_visualizer.h"
#include <ctime>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta, propeller_speed = 100.0f, scale = 100.0f;
    int screen_width, screen_height;
    Uint32 ticks = SDL_GetTicks()/10.0f;

    SDL_GetRendererOutputSize(gRenderer.get(), &screen_width, &screen_height);

    /* x, y, theta coordinates */
    q_x = state[0] * scale + screen_width/2;
    q_y = screen_height/2 - state[1] * scale;
    q_theta = state[2];

    /*
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFF0000FF);
    */

    int line_length = 100;
    int propeller_mount_height = 25, propeller_length = 15;
    int propeller_mount_offset = 5;

    float propeller_sin = sin(q_theta + ticks * propeller_speed), propeller_cos = cos(q_theta + ticks * propeller_speed);

    float cos_theta = cos(q_theta), sin_theta = sin(q_theta);

    float q_coords[10][4];
    // Quadrotor coordinates
    q_coords[0][0] = q_x + (line_length / 2) * cos_theta;
    q_coords[0][1] = q_y - (line_length / 2) * sin_theta;
    q_coords[0][2] = q_x - (line_length / 2) * cos_theta;
    q_coords[0][3] = q_y + (line_length / 2) * sin_theta;

    // Propeller mount left coordinates
    q_coords[1][0] = q_x - ((line_length / 4) + propeller_mount_offset) * cos_theta;
    q_coords[1][1] = q_y + ((line_length / 4) + propeller_mount_offset) * sin_theta;
    q_coords[1][2] = q_coords[1][0] - propeller_mount_height * sin_theta;
    q_coords[1][3] = q_coords[1][1] - propeller_mount_height * cos_theta;

    // Propeller mount right coordinates
    q_coords[2][0] = q_x + ((line_length / 4) + propeller_mount_offset) * cos_theta;
    q_coords[2][1] = q_y - ((line_length / 4) + propeller_mount_offset) * sin_theta;
    q_coords[2][2] = q_coords[2][0] - propeller_mount_height * sin_theta;
    q_coords[2][3] = q_coords[2][1] - propeller_mount_height * cos_theta;

    // Left propeller coordinates
    q_coords[3][0] = q_coords[1][2] - (propeller_length / 2) * propeller_cos;
    q_coords[3][1] = q_coords[1][3] + (propeller_length / 2) * propeller_sin;
    q_coords[3][2] = q_coords[1][2] + (propeller_length / 2) * propeller_cos;
    q_coords[3][3] = q_coords[1][3] - (propeller_length / 2) * propeller_sin;

    // Right propeller coordinates
    q_coords[4][0] = q_coords[2][2] - (propeller_length / 2) * propeller_cos;
    q_coords[4][1] = q_coords[2][3] + (propeller_length / 2) * propeller_sin;
    q_coords[4][2] = q_coords[2][2] + (propeller_length / 2) * propeller_cos;
    q_coords[4][3] = q_coords[2][3] - (propeller_length / 2) * propeller_sin;

    // Draw the lines
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    for (int i = 0; i < 5; i++) {
        SDL_RenderDrawLine(gRenderer.get(), q_coords[i][0], q_coords[i][1], q_coords[i][2], q_coords[i][3]);
    }

    /*
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    filledCircleColor(gRenderer.get(), q_x - 15*cos((M_PI/2)-q_theta), q_y - 15*sin((M_PI/2)-q_theta), 5, 0xFF0000FF);
    */
}