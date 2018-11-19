//
// Created by aadc on 13.09.18.
//
#pragma once
#ifndef AADC_USER_MATUTILS_H
#define AADC_USER_MATUTILS_H


#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "Point.h"

using namespace fraisers::models;


class MatUtils {
public:

    static bool inBoundaries(cv::Mat &mat, const Point &point) {
        return MatUtils::inBoundaries(mat,
                                      static_cast<int>(ceil(point.getX())),
                                      static_cast<int>(ceil(point.getY())));
    }

    static bool inBoundaries(const cv::Mat &mat, int x, int y) {
        return x >= 0 && y >= 0 && x < mat.rows && y < mat.cols;

    }

    static void interpolatePoints(cv::Mat &mat, std::vector<Point> *center_points) {
        uint pts_size = static_cast<uint>(center_points->size() - 1);


        for (uint idx = 0; idx < pts_size; idx += 1) {
            Point start = (*center_points)[idx];
            Point end_pnt = (*center_points)[idx + 1];
            cv::Point2f curr_center(start.getX(), start.getY());
            cv::Point2f next_center(end_pnt.getX(), end_pnt.getY());

            cv::LineIterator it(mat, curr_center, next_center, 8);
            for (int i = 0; i < it.count; i++, ++it) {
                cv::Point interpolated_pt = it.pos();
                center_points->emplace_back(Point::Global(interpolated_pt.x, interpolated_pt.y));
            }
        }
    }


    static void rotateImg(cv::Mat *img, float angleDegree, Point center = Point::Local(0, 0), float scale = 1.0) {
    cv::Point2f rotate_center;
    if (!center.isInitPoint()) {
        rotate_center = cv::Point2f(center.getX(), center.getY());
    } else {
        rotate_center = cv::Point2f(img->rows / 2, img->cols / 2);
    }

    cv::Mat rot_mat = cv::getRotationMatrix2D(rotate_center, angleDegree, scale);
    cv::warpAffine(*img, *img, rot_mat, img->size());
}

};

#endif //AADC_USER_MATUTILS_H
