/*
 * Utils.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#include "od/detectors/local2D/simple_ransac_detection/Utils.h"

namespace od {
  
  namespace l2d {
    // For text
    int fontFace = cv::FONT_ITALIC;
    double fontScale = 0.75;
    int thickness_font = 2;

    // For circles
    int lineType = 16;
    int radius = 1;
    double thickness_circ = -1;

    void viewImage(const cv::Mat & image, const std::vector<cv::KeyPoint> & keypoints)
    {
      cv::Mat outimage = image;
      cv::drawKeypoints(image, keypoints, outimage, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      cv::namedWindow( "Keypoints", cv::WINDOW_NORMAL); // Create a window for display.
      cv::resizeWindow("Keypoints", 800, 600);

      cv::imshow("Keypoints", outimage);
      cv::waitKey(0);
    }

    // Draw a text with the question point
    void drawQuestion(cv::Mat & image, const cv::Point3f & point, const cv::Scalar & color)
    {
      std::string x = std::to_string((int)point.x);
      std::string y = std::to_string((int)point.y);
      std::string z = std::to_string((int)point.z);

      std::string text = "Where is point (" + x + ","  + y + "," + z + ") ?";
      cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 8);
    }

    // Draw a text with the number of entered points
    void drawText(cv::Mat & image, const std::string & text, const cv::Scalar & color)
    {
      cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 8);
    }

    // Draw a text with the number of entered points
    void drawText2(cv::Mat & image, const std::string & text, const cv::Scalar & color)
    {
      cv::putText(image, text, cv::Point(25,75), fontFace, fontScale, color, thickness_font, 8);
    }

    // Draw a text with the frame ratio
    void drawFPS(cv::Mat & image, double fps, const cv::Scalar & color)
    {
      std::string fps_str = std::to_string((int)fps);
      std::string text = fps_str + " FPS";
      cv::putText(image, text, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
    }

    // Draw a text with the frame ratio
    void drawConfidence(cv::Mat & image, double confidence, const cv::Scalar & color)
    {
      std::string conf_str = std::to_string((int)confidence);
      std::string text = conf_str + std::string(" %");
      cv::putText(image, text, cv::Point(500,75), fontFace, fontScale, color, thickness_font, 8);
    }

    // Draw a text with the number of entered points
    void drawCounter(cv::Mat & image, int n, int n_max, const cv::Scalar & color)
    {
      std::string n_str = std::to_string(n);
      std::string n_max_str = std::to_string(n_max);
      std::string text = n_str + " of " + n_max_str + " points";
      cv::putText(image, text, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
    }

    // Draw the points and the coordinates
    void drawPoints(cv::Mat & image, const std::vector<cv::Point2f> & list_points_2d, const std::vector<cv::Point3f> & list_points_3d, 
                    const cv::Scalar & color)
    {
      for(size_t i = 0; i < list_points_2d.size(); ++i)
      {
        cv::Point2f point_2d = list_points_2d[i];
        cv::Point3f point_3d = list_points_3d[i];

        // Draw Selected points
        cv::circle(image, point_2d, radius, color, -1, lineType );

        std::string idx = std::to_string(i+1);
        std::string x = std::to_string((int)point_3d.x);
        std::string y = std::to_string((int)point_3d.y);
        std::string z = std::to_string((int)point_3d.z);
        std::string text = "P" + idx + " (" + x + "," + y + "," + z +")";

        point_2d.x = point_2d.x + 10;
        point_2d.y = point_2d.y - 10;
        cv::putText(image, text, point_2d, fontFace, fontScale*0.5, color, thickness_font, 8);
      }
    }

    // Draw only the 2D points
    void draw2DPoints(cv::Mat & image, const std::vector<cv::Point2f> & list_points, const cv::Scalar & color)
    {
      for(size_t i = 0; i < list_points.size(); i++)
      {
        cv::Point2f point_2d = list_points[i];

        // Draw Selected points
        cv::circle(image, point_2d, radius, color, -1, lineType );
      }
    }

    // Draw an arrow into the image
    void drawArrow(cv::Mat & image, cv::Point2i & p, const cv::Point2i & q, const cv::Scalar & color, 
                   int arrowMagnitude, int thickness, int line_type, int shift)
    {
      //Draw the principle line
      cv::line(image, p, q, color, thickness, line_type, shift);
      const double PI = CV_PI;
      //compute the angle alpha
      const double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
      //compute the coordinates of the first segment
      p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
      p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
      //Draw the first segment
      cv::line(image, p, q, color, thickness, line_type, shift);
      //compute the coordinates of the second segment
      p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
      p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
      //Draw the second segment
      cv::line(image, p, q, color, thickness, line_type, shift);
    }

    // Draw the 3D coordinate axes
    void draw3DCoordinateAxes(cv::Mat & image, const std::vector<cv::Point2f> & list_points2d)
    {
      cv::Scalar red(0, 0, 255);
      cv::Scalar green(0,255,0);
      cv::Scalar blue(255,0,0);
      cv::Scalar black(0,0,0);

      cv::Point2i origin = list_points2d[0];
      cv::Point2i pointX = list_points2d[1];
      cv::Point2i pointY = list_points2d[2];
      cv::Point2i pointZ = list_points2d[3];

      drawArrow(image, origin, pointX, red, 9, 2);
      drawArrow(image, origin, pointY, blue, 9, 2);
      drawArrow(image, origin, pointZ, green, 9, 2);
      cv::circle(image, origin, radius/2, black, -1, lineType );

    }

    // Draw the object mesh
    void drawObjectMesh(cv::Mat image, const Mesh & mesh, PnPProblem & pnpProblem, const cv::Scalar & color)
    {
      std::vector<std::vector<int> > list_triangles = mesh.getTrianglesList();
    //  if (pnpProblem->get_A_matrix().at<float>(0,2) < 300 && list_triangles.size() > 1000)
    //    skip = 10;

      for(size_t i = 0; i < list_triangles.size(); ++i)
      {
        std::vector<int> tmp_triangle = list_triangles.at(i);

        const cv::Point3f point_3d_0 = mesh.getVertex(tmp_triangle[0]);
        const cv::Point3f point_3d_1 = mesh.getVertex(tmp_triangle[1]);
         cv::Point3f point_3d_2 = mesh.getVertex(tmp_triangle[2]);

        //std::cout << point_3d_0 << std::endl << point_3d_1 << std::endl << point_3d_2 << std::endl << std::endl;

        const cv::Point2f point_2d_0 = pnpProblem.backproject3DPoint(point_3d_0);
        const cv::Point2f point_2d_1 = pnpProblem.backproject3DPoint(point_3d_1);
        const cv::Point2f point_2d_2 = pnpProblem.backproject3DPoint(point_3d_2);

        cv::line(image, point_2d_0, point_2d_1, color, 1);
        cv::line(image, point_2d_1, point_2d_2, color, 1);
        cv::line(image, point_2d_2, point_2d_0, color, 1);
      }
    }

    void drawModel(cv::Mat & image, const Model & model, const PnPProblem & pnpProblem, const cv::Scalar & color)
    {
      std::vector<cv::Point2f> img_points;
      cv::projectPoints(model.getPoints3d(), pnpProblem.getRVect(), pnpProblem.getTMatrix(), pnpProblem.getAMatrix(), 
                        pnpProblem.getDistCoef(), img_points);
      draw2DPoints(image, img_points, color);

    }

    void drawModel(cv::Mat & image, const Model & model,  cv::Mat r_vect, 
                    cv::Mat t_mat,  cv::Mat a_mat,  cv::Mat dist, const cv::Scalar & color)
    {
      std::vector<cv::Point2f> img_points;
      cv::projectPoints(model.getPoints3d(), r_vect, t_mat, a_mat, dist, img_points);
      draw2DPoints(image, img_points, color);

    }

    // Computes the norm of the translation error
    double getTranslationError(const cv::Mat & t_true, const cv::Mat & t)
    {
      return cv::norm( t_true - t );
    }

    // Computes the norm of the rotation error
    double getRotationError(const cv::Mat & r_true, const cv::Mat & r)
    {
      cv::Mat error_vec, error_mat;
      error_mat = r_true * cv::Mat(r.inv()).mul(-1);
      cv::Rodrigues(error_mat, error_vec);

      return cv::norm(error_vec);
    }

    // Converts a given Rotation Matrix to Euler angles
    cv::Mat rot2euler(const cv::Mat & rotation_matrix)
    {
      cv::Mat euler(3,1,CV_64F);

      const double m00 = rotation_matrix.at<double>(0,0);
      const double m02 = rotation_matrix.at<double>(0,2);
      const double m10 = rotation_matrix.at<double>(1,0);
      const double m11 = rotation_matrix.at<double>(1,1);
      const double m12 = rotation_matrix.at<double>(1,2);
      const double m20 = rotation_matrix.at<double>(2,0);
      const double m22 = rotation_matrix.at<double>(2,2);

      double x, y, z;

      // Assuming the angles are in radians.
      if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI/2;
        z = atan2(m02,m22);
      }
      else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI/2;
        z = atan2(m02,m22);
      }
      else
      {
        x = atan2(-m12,m11);
        y = asin(m10);
        z = atan2(-m20,m00);
      }

      euler.at<double>(0) = x;
      euler.at<double>(1) = y;
      euler.at<double>(2) = z;

      return euler;
    }

    // Converts a given Euler angles to Rotation Matrix
    cv::Mat euler2rot(const cv::Mat & euler)
    {
      cv::Mat rotation_matrix(3, 3, CV_64F);

      const double x = euler.at<double>(0);
      const double y = euler.at<double>(1);
      const double z = euler.at<double>(2);

      // Assuming the angles are in radians.
      const double ch = cos(z);
      const double sh = sin(z);
      const double ca = cos(y);
      const double sa = sin(y);
      const double cb = cos(x);
      const double sb = sin(x);

      double m00, m01, m02, m10, m11, m12, m20, m21, m22;

      m00 = ch * ca;
      m01 = sh*sb - ch*sa*cb;
      m02 = ch*sa*sb + sh*cb;
      m10 = sa;
      m11 = ca*cb;
      m12 = -ca*sb;
      m20 = -sh*ca;
      m21 = sh*sa*cb + ch*sb;
      m22 = -sh*sa*sb + ch*cb;

      rotation_matrix.at<double>(0,0) = m00;
      rotation_matrix.at<double>(0,1) = m01;
      rotation_matrix.at<double>(0,2) = m02;
      rotation_matrix.at<double>(1,0) = m10;
      rotation_matrix.at<double>(1,1) = m11;
      rotation_matrix.at<double>(1,2) = m12;
      rotation_matrix.at<double>(2,0) = m20;
      rotation_matrix.at<double>(2,1) = m21;
      rotation_matrix.at<double>(2,2) = m22;

      return rotation_matrix;
    }

  }

}