//
// Created by aadc on 03.08.18.
//

#ifndef PLANNER_PRINTUTILS_H
#define PLANNER_PRINTUTILS_H

#include "vector"
#include "Point.h"
#include "string"
#include <iostream>
#include "math.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iomanip>

using namespace std;
using namespace fraisers::models;


class PrintUtils {
private:

public:

    enum Color {
        RED = 31,
        BLUE = 34,
        GREEN = 32,
        YELLOW = 33,
        MAGENTA = 35,
        CYAN = 36
    };


    static void drawCircle(Point center, int radius, cv::Scalar color_rgb, cv::Mat *img){
        if(center.getY()>=img->rows || center.getX()>= img->cols){
            return;
        }
        cv::Point2f pt(center.getY(), center.getX());
        cv::circle(*img, pt, radius, color_rgb);
    }

    static void printWithColor(std::string text, Color color) {

        std::string col = "\033[1;" + std::to_string(color) + "m";

        std::cout << col << text << "\033[0m";

    }

    static void printWithColor(std::string text, char color) {

        int color_int = 30;

        if (color == 'r') {
            color_int = 31;
        } else if (color == 'g') {
            color_int = 32;
        } else if (color == 'y') {
            color_int = 33;
        } else if (color == 'b') {
            color_int = 34;
        } else if (color == 'm') {
            color_int = 35;
        } else if (color == 'c') {
            color_int = 36;
        }
        std::string col = "\033[1;" + std::to_string(color_int) + "m";

        std::cout << col << text << "\033[0m";
    };

    static void
    printGridAndWaypoints(cv::Mat *grid_ptr, vector<Point> wps, vector<int> start,
                          vector<int> goal) {
        cv::Mat grid = (*grid_ptr);

        for (int i = 0; i < grid.rows; i++) {
            for (int j = 0; j < grid.cols; j++) {
                if (i == goal[0] && j == goal[1]) {
                    printWithColor("|G| ", 'r');
                } else if (i == start[0] && j == start[1]) {
                    printWithColor("|P| ", 'g');
                } else {
                    bool wp_found_ = false;
                    for (Point &wp : wps) {
                        if (i == round(wp.getX()) && j == round(wp.getY())) {
                            printWithColor("|W| ", 'b');
                            wp_found_ = true;
                        }
                    }
                    if (!wp_found_) {
                        cout << grid.at<float>(i, j) << " ";
                    }

                }
            }
            cout << std::endl;
        }
    }

    static void printGrid(vector<vector<int>> *grid_ptr) {
        vector<vector<int>> grid = (*grid_ptr);

        for (uint i = 0; i < grid.size(); i++) {
            for (uint j = 0; j < grid[i].size(); j++) {

                cout << grid[i][j] << " ";
            }
            cout << std::endl;
        }
    };

    static void printUcharGrid(cv::Mat *grid) {
        for (int i = 0; i < grid->rows; i++) {
            for (int j = 0; j < grid->cols; j++) {
                auto test = grid->at<uchar>(i, j);

                cout << int(test) << " ";
            }
            cout << std::endl;
        }
        cout << std::endl;
    };

    static void printFloatGrid(cv::Mat *grid) {
        for (int i = 0; i < grid->rows; i++) {
            for (int j = 0; j < grid->cols; j++) {
                auto test = grid->at<float>(i, j);

                cout << std::fixed << std::setprecision(1) << test << " ";
            }
            cout << std::endl;
        }
        cout << std::endl;
    };

    static void printVec3fGrid(cv::Mat *grid) {
        for (int i = 0; i < grid->rows; i++) {
            for (int j = 0; j < grid->cols; j++) {
                auto test = grid->at<cv::Vec3f>(i, j);

                cout << std::fixed << std::setprecision(1) << test << " ";
            }
            cout << std::endl;
        }
        cout << std::endl;
    };

    static void GridToImage(cv::Mat *grid) {
        double min, max;
        cv::minMaxLoc(*grid, &min, &max);
        int max_intensity = 255;

        //*grid = max_intensity - (*grid);
        *grid = *grid * (max_intensity / max);
    };


    static bool writeMatToPng(const cv::Mat &mat, const std::string &path,
                              const int &img_padding = 1,
                              const int &map_padding = 1) {
        std::cout << "\t\t[Writing map to .png]" << std::endl << std::flush;
        try {
            cv::imwrite(path, mat);
            std::cout << "\t\tMap saved as: " << path << std::endl << std::flush;
            return true;
        }
        catch (cv::Exception &ex) {
            fprintf(stderr, "Exception exporting map as .png file: %s\n", ex.what());
            return false;
        }
    }
};


#endif //PLANNER_PRINTUTILS_H