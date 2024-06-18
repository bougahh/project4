#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta, scale = 100.0f;
    int screen_width, screen_height;

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
    float x1,x2,y1,y2,dx,dy;

    dx = line_length/2 * cos(q_theta);
    dy = line_length/2 * sin(q_theta);

    x1 = q_x + dx;
    y1 = q_y - dy;
    x2 = q_x - dx;
    y2 = q_y + dy;

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(),x1,y1,x2,y2);

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    filledCircleColor(gRenderer.get(), q_x - 15*cos((M_PI/2)-q_theta), q_y - 15*sin((M_PI/2)-q_theta), 5, 0xFF0000FF);
}