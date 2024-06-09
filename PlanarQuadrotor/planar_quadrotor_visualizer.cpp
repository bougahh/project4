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
    float q_x, q_y, q_theta;
    int width, height;
    if (SDL_GetRendererOutputSize(gRenderer.get(), &width, &height) != 0) {
        std::cout << "Wystąpił błąd, sprawdź SDL_GetError()\n";
    }
    /* x, y, theta coordinates */
    q_x = state[0] + static_cast<float>(width)/2;
    q_y = static_cast<float>(height)/2 - state[1];
    q_theta = state[2];

    filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFF0000FF);
    /*
    SDL_SetRenderDrawColor(gRenderer.get(), 0x22, 0x22, 0x22, 0xFF);
    SDL_Rect rect;
    rect.x = q_x;
    rect.y = q_y;
    rect.w = 90; // szerokość prostokąta
    rect.h = 30; // wysokość prostokąta 
    SDL_RenderFillRect(gRenderer.get(), &rect);
    */

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF); // Ustaw kolor na niebieski
    int line_length = 90; // Długość linii
    int line_x1 = q_x - line_length / 2;
    int line_y1 = q_y;
    int line_x2 = q_x + line_length / 2;
    int line_y2 = q_y;

    int rotated_x1 = q_x + (line_x1 - q_x) * cos(q_theta) - (line_y1 - q_y) * sin(q_theta);
    int rotated_y1 = q_y + (line_x1 - q_x) * sin(q_theta) + (line_y1 - q_y) * cos(q_theta);
    int rotated_x2 = q_x + (line_x2 - q_x) * cos(q_theta) - (line_y2 - q_y) * sin(q_theta);
    int rotated_y2 = q_y + (line_x2 - q_x) * sin(q_theta) + (line_y2 - q_y) * cos(q_theta);

    SDL_RenderDrawLine(gRenderer.get(), rotated_x1, rotated_y1, rotated_x2, rotated_y2);
}
