/*
 * Utils.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */
#pragma once
#include <iostream>
#include "od/detectors/local2D/simple_ransac_detection/ODPnPProblem.h"
#include "od/detectors/local2D/simple_ransac_detection/ODModelRegistration.h"
#include "od/detectors/local2D/simple_ransac_detection/ODModel.h"
#include "od/detectors/local2D/simple_ransac_detection/ODMesh.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class Mesh;
class PnPProblem;
class Model;

namespace od {
  
  namespace l2d {

		void viewImage(const cv::Mat & image, const std::vector<cv::KeyPoint> & keypoints = std::vector<cv::KeyPoint>(0));

		// Draw a text with the question point
		void drawQuestion(cv::Mat & image, const cv::Point3f & point, const cv::Scalar & color);

		// Draw a text with the number of entered points
		void drawText(cv::Mat & image, const std::string & text, const cv::Scalar & color);

		// Draw a text with the number of entered points
		void drawText2(cv::Mat & image, const std::string & text, const cv::Scalar & color);

		// Draw a text with the frame ratio
		void drawFPS(cv::Mat & image, double fps, const cv::Scalar & color);

		// Draw a text with the frame ratio
		void drawConfidence(cv::Mat & image, double confidence, const cv::Scalar & color);

		// Draw a text with the number of entered points
		void drawCounter(cv::Mat & image, int n, int n_max, const cv::Scalar & color);

		// Draw the points and the coordinates
		void drawPoints(cv::Mat & image, const std::vector<cv::Point2f> & list_points_2d, const std::vector<cv::Point3f> & list_points_3d, 
			            const cv::Scalar & color);

		// Draw only the 2D points
		void draw2DPoints(cv::Mat & image, const std::vector<cv::Point2f> & list_points, const cv::Scalar & color);

		// Draw an arrow into the image
		void drawArrow(cv::Mat & image, cv::Point2i & p, const cv::Point2i & q, const cv::Scalar & color, int arrowMagnitude = 9, 
			           int thickness = 1, int line_type = 8, int shift = 0);

		// Draw the 3D coordinate axes
		void draw3DCoordinateAxes(cv::Mat & image, const std::vector<cv::Point2f> & list_points2d);

		// Draw the object mesh
		void drawObjectMesh(cv::Mat & image, const Mesh & mesh, PnPProblem & pnpProblem, const cv::Scalar & color);
		void drawModel(cv::Mat & image, const Model & model,  PnPProblem & pnpProblem, const cv::Scalar color);
		void drawModel(cv::Mat & image, const Model & model, cv::Mat r_vect,  cv::Mat t_mat, cv::Mat a_mat, 
			           cv::Mat dist, const cv::Scalar & color);

		// Computes the norm of the translation error
		double getTranslationError(const cv::Mat & t_true, const cv::Mat & t);

		// Computes the norm of the rotation error
		double getRotationError(const cv::Mat & r_true, const cv::Mat & r);

		// Converts a given Rotation Matrix to Euler angles
		cv::Mat rot2euler(const cv::Mat & rotationMatrix);

		// Converts a given Euler angles to Rotation Matrix
		cv::Mat euler2rot(const cv::Mat & euler);

	}

}

