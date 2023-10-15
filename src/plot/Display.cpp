/**
 * @file Display.cpp
 *
 * @brief This file contains the implementation of the Display class
 *
 * @details The Display class is used to plot the robot's trajectory and
 * the robot's position and orientation. It also plots the dead reckoning
 * estimates and the error between the ground truth and the dead reckoning.
 * The Display class uses SDL to plot the elements on the screen.
 *
 * @see Display.hpp
 *
 * @date 2023-10-15
 *
 * @author Allan Souza Almeida
 *
 * @note This file is part of the `allan_husky` package.
 */

#include "Display.hpp"

Display::Display()
{
    theta = 0;
    xpos = 0;
    ypos = 0;
    theta_est = 0;
    xpos_est = 0;
    ypos_est = 0;
    gridSize = 130;
    fontPath = ros::package::getPath("allan_husky") + "/src/plot/";
    isBeingDragged = false;
}
Display::~Display() {}

// --------------------------------------------------------
// --------------------------------------------------------
// -------------------- PUBLIC METHODS --------------------
// --------------------------------------------------------
// --------------------------------------------------------

/**
 * @brief Initialize the display class
 *
 * @param title The title of the window
 * @param xpos The x position of the window
 * @param ypos The y position of the window
 * @param width The width of the window
 * @param height The height of the window
 * @param fullScreen If the window should be fullscreen or not
 */
void Display::init(const char *title, int xpos, int ypos, int width, int height, bool fullScreen)
{
    int flags = 0;
    if (fullScreen)
    {
        flags = SDL_WINDOW_FULLSCREEN;
    }
    if (SDL_Init(SDL_INIT_EVERYTHING) == 0)
    {
        std::cout << "Subsystems Initialized!\n";

        displayWidth = width;
        displayHeight = height;
        x0 = displayWidth / 2;
        y0 = displayHeight / 2;

        window = SDL_CreateWindow(title, xpos, ypos, width, height, flags);
        if (window)
        {
            std::cout << "Window created!\n";
        }

        renderer = SDL_CreateRenderer(window, -1, 0);
        if (renderer)
        {
            std::cout << "Renderer created!\n";
        }

        isRunning = true;
    }
    else
    {
        isRunning = false;
    }
}

/**
 * @brief Handle the events of the display, such as mouse and keyboard events
 *
 */
void Display::handleEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        // Check if the window is closed
        case SDL_QUIT:
            isRunning = false;
            break;

        // Check if E, C, or ESC key is pressed
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym)
            {
            case SDLK_ESCAPE:
                isRunning = false;
                break;
            case SDLK_e:
                x_vec.clear();
                y_vec.clear();
                x_est_vec.clear();
                y_est_vec.clear();
                xpos = 0;
                ypos = 0;
                theta = 0;
                break;
            case SDLK_c:
                x0 = displayWidth / 2;
                y0 = displayHeight / 2;
                break;
            default:
                break;
            }

        // Check if the left mouse button is pressed
        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_LEFT)
                isBeingDragged = true;
            break;

        // Check if the left mouse button is released
        case SDL_MOUSEBUTTONUP:
            if (event.button.button == SDL_BUTTON_LEFT)
                isBeingDragged = false;
            break;

        // Implement dragging
        case SDL_MOUSEMOTION:
            if (isBeingDragged)
            {
                x0 += event.motion.xrel;
                y0 += event.motion.yrel;
            }
            break;

        default:
            break;
        }
    }
}

/**
 * @brief Set the states of the robot (ground truth)
 *
 * @param x The x position of the robot
 * @param y The y position of the robot
 * @param theta The orientation of the robot
 */
void Display::setStates(double x, double y, double theta)
{
    this->xpos = x * gridSize;
    this->ypos = y * gridSize;
    this->theta = theta;
    x_vec.push_back(xpos);
    y_vec.push_back(ypos);
}

/**
 * @brief Set the estimates of the robot (dead reckoning)
 *
 * @param x The x position of the robot
 * @param y The y position of the robot
 * @param theta The orientation of the robot
 */
void Display::setEstimates(double x, double y, double theta)
{
    this->xpos_est = x * gridSize;
    this->ypos_est = y * gridSize;
    this->theta_est = theta;
    x_est_vec.push_back(xpos_est);
    y_est_vec.push_back(ypos_est);
}

/**
 * @brief Calculate the error between the ground truth and the dead reckoning
 */
void Display::calculateError()
{
    double error_x = xpos - xpos_est;
    double error_y = ypos - ypos_est;
    double error_theta = theta - theta_est;
    if (x_est_vec.size() > 1)
    {
        this->x_err_sum += error_x * error_x;
        this->y_err_sum += error_y * error_y;
        this->theta_err_sum += error_theta * error_theta;
        this->x_rmse = sqrt(x_err_sum / x_est_vec.size());
        this->y_rmse = sqrt(y_err_sum / y_est_vec.size());
        this->theta_rmse = sqrt(theta_err_sum / y_est_vec.size());
    }
}

/**
 * @brief Render the elements on the display
 */
void Display::render()
{
    // Clear the renderer with a background color (e.g., black)
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black color
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_RenderClear(renderer);

    // ADD STUFF TO RENDER HERE
    drawGrid();
    drawLegend();
    drawPositionInfo();
    drawRobot();
    drawTrajectory();
    drawEstimate();

    // Present the renderer
    SDL_RenderPresent(renderer);
}

// ---------------------------------------------------------
// ---------------------------------------------------------
// -------------------- PRIVATE METHODS --------------------
// ---------------------------------------------------------
// ---------------------------------------------------------

/**
 * @brief Draw a grid on the display. The grid is centered at (`x0`, `y0`) and has a size of `gridSize`
 *
 */
void Display::drawGrid()
{
    // draw a grid
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 40); // White color
    for (int i = x0; i < displayWidth; i += gridSize)
    {
        SDL_RenderDrawLine(renderer, i, 0, i, displayHeight);
    }
    for (int i = x0; i > 0; i -= gridSize)
    {
        SDL_RenderDrawLine(renderer, i, 0, i, displayHeight);
    }
    for (int i = y0; i < displayHeight; i += gridSize)
    {
        SDL_RenderDrawLine(renderer, 0, i, displayWidth, i);
    }
    for (int i = y0; i > 0; i -= gridSize)
    {
        SDL_RenderDrawLine(renderer, 0, i, displayWidth, i);
    }
}

/**
 * @brief Draw a text on the display. If `align` is `ALIGN_LEFT`, the x offset is
 * relative to the left side of the display. If `align` is `ALIGN_RIGHT`, the x offset
 * is relative to the right side of the display.
 *
 * @param text The text to be drawn
 * @param x_offset The x offset of the text (relative to the left or right side of the display)
 * @param y_offset The y offset of the text (relative to the top of the display)
 * @param size The size of the text
 * @param align The alignment of the text (ALIGN_LEFT or ALIGN_RIGHT)
 * @param greek If the text is greek or not (used for the theta symbol) - default: false
 * @param color The color of the text - default: {255, 255, 255} (white)
 */
void Display::drawText(const char *text, int x_offset, int y_offset,
                       int size, int align = ALIGN_RIGHT, bool greek = false,
                       SDL_Color color = {255, 255, 255})
{
    TTF_Init();
    TTF_Font *font;
    if (greek)
    {
        font = TTF_OpenFont((fontPath + "greek.ttf").c_str(), size);
    }
    else
    {
        font = TTF_OpenFont((fontPath + "Roboto-Regular.ttf").c_str(), size);
    }
    SDL_Surface *surface = TTF_RenderText_Solid(font, text, color);
    SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_Rect textRect;
    if (align == ALIGN_LEFT)
    {
        textRect.x = x_offset;
        textRect.y = y_offset;
    }
    else
    {
        textRect.x = displayWidth - surface->w - x_offset;
        textRect.y = displayHeight - surface->h - y_offset;
    }
    textRect.w = surface->w;
    textRect.h = surface->h;
    SDL_RenderCopy(renderer, texture, NULL, &textRect);
    SDL_FreeSurface(surface);
    SDL_DestroyTexture(texture);
    TTF_CloseFont(font);
}

/**
 * @brief Draw a legend on the display. The legend is a rectangle with two squares
 * inside it. The first square represents the ground truth color and the second square
 * represents the dead reckoning color.
 */
void Display::drawLegend()
{
    // draw a rectangle
    SDL_Rect rect;
    rect.x = displayWidth - 200 - 30;
    rect.y = displayHeight - 100 - 30;
    rect.w = 200;
    rect.h = 100;
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // White color
    SDL_RenderDrawRect(renderer, &rect);

    // draw squares to represent the legend color
    SDL_Rect rect1;
    rect1.x = displayWidth - 200 - 10;
    rect1.y = displayHeight - 100 - 10;
    rect1.w = 20;
    rect1.h = 20;
    SDL_SetRenderDrawColor(renderer, 0, 100, 255, 255); // Blue color
    SDL_RenderFillRect(renderer, &rect1);

    SDL_Rect rect2;
    rect2.x = displayWidth - 200 - 10;
    rect2.y = displayHeight - 100 - 10 + 40;
    rect2.w = 20;
    rect2.h = 20;
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red color
    SDL_RenderFillRect(renderer, &rect2);

    // draw text
    this->drawText("Ground Truth", 70, 88, 18);
    this->drawText("Dead Reckoning", 45, 48, 18);
}

/**
 * @brief Draw the position information on the display. The position information
 * is the ground truth and dead reckoning positions and orientations and the
 * error between them. It also shows the instructions to clear the trajectory,
 * center the map, and exit the simulation.
 */
void Display::drawPositionInfo()
{
    // draw text
    this->drawText("Ground Truth", 20, 20, 20, ALIGN_LEFT);
    this->drawText("x :", 20, 50, 20, ALIGN_LEFT);
    this->drawText("y :", 20, 70, 20, ALIGN_LEFT);
    this->drawText("q", 20, 96, 23, ALIGN_LEFT, true);
    this->drawText(" :", 30, 90, 20, ALIGN_LEFT);
    std::stringstream x_fixed, y_fixed, theta_fixed;
    x_fixed << std::fixed << std::setprecision(3) << xpos / gridSize;
    y_fixed << std::fixed << std::setprecision(3) << ypos / gridSize;
    theta_fixed << std::fixed << std::setprecision(3) << theta;
    std::string x_str = x_fixed.str();
    std::string y_str = y_fixed.str();
    std::string theta_str = theta_fixed.str();
    this->drawText(x_str.c_str(), 60, 50, 20, ALIGN_LEFT);
    this->drawText(y_str.c_str(), 60, 70, 20, ALIGN_LEFT);
    this->drawText(theta_str.c_str(), 60, 90, 20, ALIGN_LEFT);
    this->drawText("m", 130, 50, 20, ALIGN_LEFT);
    this->drawText("m", 130, 70, 20, ALIGN_LEFT);
    this->drawText("rad", 130, 90, 20, ALIGN_LEFT);

    this->drawText("Dead Reckoning", 20, 130, 20, ALIGN_LEFT);
    this->drawText("x :", 20, 160, 20, ALIGN_LEFT);
    this->drawText("y :", 20, 180, 20, ALIGN_LEFT);
    this->drawText("q", 20, 206, 23, ALIGN_LEFT, true);
    std::stringstream x_est_fixed, y_est_fixed, theta_est_fixed;
    x_est_fixed << std::fixed << std::setprecision(3) << xpos_est / gridSize;
    y_est_fixed << std::fixed << std::setprecision(3) << ypos_est / gridSize;
    theta_est_fixed << std::fixed << std::setprecision(3) << theta_est;
    std::string x_est_str = x_est_fixed.str();
    std::string y_est_str = y_est_fixed.str();
    std::string theta_est_str = theta_est_fixed.str();
    this->drawText(x_est_str.c_str(), 60, 160, 20, ALIGN_LEFT);
    this->drawText(y_est_str.c_str(), 60, 180, 20, ALIGN_LEFT);
    this->drawText(theta_est_str.c_str(), 60, 200, 20, ALIGN_LEFT);
    this->drawText("m", 130, 160, 20, ALIGN_LEFT);
    this->drawText("m", 130, 180, 20, ALIGN_LEFT);
    this->drawText("rad", 130, 200, 20, ALIGN_LEFT);

    this->drawText("Erro RMSE", 20, 240, 20, ALIGN_LEFT);
    this->drawText("x (RMSE) :", 20, 270, 20, ALIGN_LEFT);
    this->drawText("y (RMSE) :", 20, 290, 20, ALIGN_LEFT);
    this->drawText("q", 20, 316, 23, ALIGN_LEFT, true);
    this->drawText(" (RMSE) :", 30, 310, 20, ALIGN_LEFT);
    std::stringstream x_rmse_fixed, y_rmse_fixed, theta_rmse_fixed;
    x_rmse_fixed << std::fixed << std::setprecision(3) << x_rmse / gridSize;
    y_rmse_fixed << std::fixed << std::setprecision(3) << y_rmse / gridSize;
    theta_rmse_fixed << std::fixed << std::setprecision(3) << theta_rmse;
    std::string x_rmse_str = x_rmse_fixed.str();
    std::string y_rmse_str = y_rmse_fixed.str();
    std::string theta_rmse_str = theta_rmse_fixed.str();
    this->drawText(x_rmse_str.c_str(), 120, 270, 20, ALIGN_LEFT);
    this->drawText(y_rmse_str.c_str(), 120, 290, 20, ALIGN_LEFT);
    this->drawText(theta_rmse_str.c_str(), 120, 310, 20, ALIGN_LEFT);
    this->drawText("m", 190, 270, 20, ALIGN_LEFT);
    this->drawText("m", 190, 290, 20, ALIGN_LEFT);
    this->drawText("rad", 190, 310, 20, ALIGN_LEFT);

    this->drawText("Pressione [E] para limpar o trajeto", 20, displayHeight - 90, 18, ALIGN_LEFT, false, {0, 255, 0});
    this->drawText("Pressione [C] para centralizar o mapa", 20, displayHeight - 65, 18, ALIGN_LEFT, false, {0, 255, 0});
    this->drawText("Pressione [ESC] para sair", 20, displayHeight - 40, 18, ALIGN_LEFT, false, {0, 255, 0});
}

/**
 * @brief Draw the robot on the display. The robot is a rectangle with a circle
 * representing the front of the robot.
 */
void Display::drawRobot()
{
    // draw a rectangle as 4 lines
    double x = x0 + xpos;
    double y = y0 - ypos;
    double length = 14 * gridSize / 40;
    double width = 10 * gridSize / 40;
    double x1 = x + length * cos(-theta) - width * sin(-theta);
    double y1 = y + length * sin(-theta) + width * cos(-theta);
    double x2 = x + length * cos(-theta) + width * sin(-theta);
    double y2 = y + length * sin(-theta) - width * cos(-theta);
    double x3 = x - length * cos(-theta) + width * sin(-theta);
    double y3 = y - length * sin(-theta) - width * cos(-theta);
    double x4 = x - length * cos(-theta) - width * sin(-theta);
    double y4 = y - length * sin(-theta) + width * cos(-theta);

    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255); // White color
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    SDL_RenderDrawLine(renderer, x2, y2, x3, y3);
    SDL_RenderDrawLine(renderer, x3, y3, x4, y4);
    SDL_RenderDrawLine(renderer, x4, y4, x1, y1);

    // draw a circle
    double radius = 10 * gridSize / 130;
    double x0 = x + length * cos(-theta);
    double y0 = y + length * sin(-theta);
    for (double i = 0; i < 2 * M_PI; i += 0.01)
    {
        double x = x0 + radius * cos(i);
        double y = y0 + radius * sin(i);
        SDL_RenderDrawPoint(renderer, x, y);
    }
}

/**
 * @brief Draw the trajectory of the robot on the display. The trajectory is a
 * sequence of lines connecting the previous positions of the robot.
 */
void Display::drawTrajectory()
{
    // draw a trajectory
    if (x_vec.size() >= 1)
    {
        SDL_SetRenderDrawColor(renderer, 0, 100, 255, 255); // Blue color
        for (int i = 0; i < x_vec.size() - 1; i++)
        {
            SDL_RenderDrawLine(renderer, x_vec[i] + x0, y0 - y_vec[i], x_vec[i + 1] + x0, y0 - y_vec[i + 1]);
        }
    }
}

/**
 * @brief Draw the estimate of the robot on the display. The estimate is a
 * sequence of lines connecting the previous estimates of the robot.
 */
void Display::drawEstimate()
{
    // draw a trajectory
    if (x_est_vec.size() >= 1)
    {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red color
        for (int i = 0; i < x_est_vec.size() - 1; i++)
        {
            SDL_RenderDrawLine(renderer, x_est_vec[i] + x0, y0 - y_est_vec[i], x_est_vec[i + 1] + x0, y0 - y_est_vec[i + 1]);
        }
    }
}

/**
 * @brief Clean the display by destroying the window and the renderer and quitting SDL
 */
void Display::clean()
{
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(renderer);
    SDL_Quit();
    std::cout << "Simulation exited!\n";
}

/**
 * @brief Check if the simulation is running
 */
bool Display::running()
{
    return isRunning;
}