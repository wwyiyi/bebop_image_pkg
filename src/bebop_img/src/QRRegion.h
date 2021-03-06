#include <iostream>
#include <cmath>

class QRRegion{
	public:
		cv::Mat image;
		static const int CV_QR_NORTH = 0;
		static const int CV_QR_EAST = 1;
		static const int CV_QR_SOUTH = 2;
		static const int CV_QR_WEST = 3;
	private:
		float cv_distance(Point2f P, Point2f Q);					// Get Distance between two points
		float cv_lineEquation(Point2f L, Point2f M, Point2f J);		// Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
		float cv_lineSlope(Point2f L, Point2f M, int& alignement);	// Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
		void cv_getVertices(vector<vector<Point> > contours, int c_id,float slope, vector<Point2f>& X);
		void cv_updateCorner(Point2f P, Point2f ref ,float& baseline,  Point2f& corner);
		void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
		bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
		float cross(Point2f v1,Point2f v2);
}
