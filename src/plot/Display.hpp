/**
 * @file Display.hpp
 *
 * @brief This file contains the Display class.
 *
 * @details This file contains the Display class, which is used to display the robot's position and trajectory.
 *
 * @version 1.0.0
 *
 * @date 2023-10-15
 *
 * @author Allan Souza Almeida
 *
 * @note This file is part of the `allan_husky` package.
 */

#ifndef Display_hpp
#define Display_hpp
#include "SDL2/SDL.h"
#include "SDL2/SDL_ttf.h"
#include "ros/package.h"
#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <sstream>
#include <algorithm>

#define ALIGN_LEFT 0  // Offset from the left
#define ALIGN_RIGHT 1 // Offset from the right

class Display
{
public:
    Display();
    ~Display();

    void init(const char *title, int xpos, int ypos, int width, int height, bool fullScreen);
    void handleEvents();
    void setStates(double x, double y, double theta);
    void setEstimates(double x, double y, double theta);
    void setEKFEstimates(double x, double y, double theta);
    void calculateError();
    void render();
    void clean();

    bool running();

private:
    // Private variables
    bool isRunning;                // Is the display running?
    SDL_Window *window;            // The window
    SDL_Renderer *renderer;        // The renderer
    int displayWidth;              // The display width
    int displayHeight;             // The display height
    double theta;                  // The robot's orientation
    double xpos;                   // The robot's x position
    double ypos;                   // The robot's y position
    double theta_est;              // The estimated orientation
    double xpos_est;               // The estimated x position
    double ypos_est;               // The estimated y position
    double x_ekf;                  // The EKF estimated x position
    double y_ekf;                  // The EKF estimated y position
    double theta_ekf;              // The EKF estimated orientation
    double x_err_sum;              // The x error sum
    double y_err_sum;              // The y error sum
    double theta_err_sum;          // The theta error sum
    double x_rmse;                 // The x root mean square error
    double y_rmse;                 // The y root mean square error
    double theta_rmse;             // The theta root mean square error
    double x0;                     // The origin x position (used for dragging)
    double y0;                     // The origin y position (used for dragging)
    bool isBeingDragged;           // Is the display being dragged?
    std::vector<double> x_vec;     // The x position vector
    std::vector<double> y_vec;     // The y position vector
    std::vector<double> x_est_vec; // The estimated x position vector
    std::vector<double> y_est_vec; // The estimated y position vector
    std::vector<double> x_ekf_vec; // The EKF estimated x position vector
    std::vector<double> y_ekf_vec; // The EKF estimated y position vector
    int gridSize;                  // The grid size
    std::string fontPath;          // The font path
    // Private methods
    void drawRobot();
    void drawGrid();
    void drawLegend();
    void drawPositionInfo();
    void drawTrajectory();
    void drawEstimate();
    void drawEKFEstimate();
    void drawText(const char *text, int x_offset, int y_offset, int size, int align, bool greek, SDL_Color color);
};

#endif /* Display_hpp */