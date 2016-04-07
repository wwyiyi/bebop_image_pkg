/**
 * @function hull_demo.cpp
 * @brief Demo code to find contours in an image
 * @author OpenCV team
 */

//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 130;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );


double dist(Point x,Point y)
{
  return (x.x-y.x)*(x.x-y.x)+(x.y-y.y)*(x.y-y.y);
}

pair<Point,double> circleFromPoints(Point p1, Point p2, Point p3)
{
  double offset = pow(p2.x,2) +pow(p2.y,2);
  double bc =   ( pow(p1.x,2) + pow(p1.y,2) - offset )/2.0;
  double cd =   (offset - pow(p3.x, 2) - pow(p3.y, 2))/2.0;
  double det =  (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y); 
  double TOL = 0.0000001;
  if (abs(det) < TOL) { cout<<"POINTS TOO CLOSE"<<endl;return make_pair(Point(0,0),0); }

  double idet = 1/det;
  double centerx =  (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
  double centery =  (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
  double radius = sqrt( pow(p2.x - centerx,2) + pow(p2.y-centery,2));

  return make_pair(Point(centerx,centery),radius);
}


/**
 * @function main
 */
Point findPalm(Mat src)
{
  /// Load source image and convert it to gray
  //src = imread( argv[1], 1 );

  /// Convert image to gray and blur it
  cvtColor( src, src_gray, COLOR_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  //const char* source_window = "Source";
  //namedWindow( source_window, WINDOW_AUTOSIZE );
  //imshow( source_window, src );

  //createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
  //thresh_callback( 0, 0 );

  //waitKey(0);
  //return(0);
//}

/**
 * @function thresh_callback
 */
//void thresh_callback(int, void* )
//{
  Mat src_copy = src.clone();
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );

  /// Find contours
  findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );


  //find large contours
   vector<vector<Point> > large_contours;
  for( size_t i = 0; i < contours.size(); i++ )
  {   
      if (contourArea(contours[i]) >= 5000)
      {
        large_contours.push_back(contours[i]);
      }
  }





  /// Find the convex hull object for each contour
  Point p_center;
  Point final_center;
  double final_radius;
  double p_radius = -1.0;
  vector<vector<Point> >hull( large_contours.size() );
  vector<vector<int> > hullsI(large_contours.size());
  for( size_t i = 0; i < large_contours.size(); i++ )
  {   
      convexHull( Mat(large_contours[i]), hull[i], false ); 
      convexHull(Mat(large_contours[i]),hullsI[i],false);
      vector<Vec4i> defects;
      RotatedRect rect=minAreaRect(Mat(large_contours[i]));

      if(hullsI[i].size()>0)
      {
          Point2f rect_points[4]; 
          rect.points( rect_points );

          Point rough_palm_center;
          convexityDefects(large_contours[i], hullsI[i], defects);
          if(defects.size()>=3)
          {
            vector<Point> palm_points;
            for(int j=0;j<defects.size();j++)
            {
              int startidx=defects[j][0]; Point ptStart( large_contours[i][startidx] );
              int endidx=defects[j][1]; Point ptEnd( large_contours[i][endidx] );
              int faridx=defects[j][2]; Point ptFar( large_contours[i][faridx] );
              //Sum up all the hull and defect points to compute average
              rough_palm_center+=ptFar+ptStart+ptEnd;
              palm_points.push_back(ptFar);
              palm_points.push_back(ptStart);
              palm_points.push_back(ptEnd);
            }

            cout << "defects size: " << defects.size() << endl;

            //Get palm center by 1st getting the average of all defect points, this is the rough palm center,
            //Then U chose the closest 3 points ang get the circle radius and center formed from them which is the palm center.
            rough_palm_center.x/=defects.size()*3;
            rough_palm_center.y/=defects.size()*3;
            Point closest_pt=palm_points[0];
            vector<pair<double,int> > distvec;
            for(int ii=0;ii<palm_points.size();ii++)
              distvec.push_back(make_pair(dist(rough_palm_center,palm_points[ii]),ii));
            sort(distvec.begin(),distvec.end());

            //Keep choosing 3 points till you find a circle with a valid radius
            //As there is a high chance that the closes points might be in a linear line or too close that it forms a very large circle
            pair<Point,double> soln_circle;
            for(int ii=0;ii+2<distvec.size();ii++)
            {
              Point p1=palm_points[distvec[ii+0].second];
              Point p2=palm_points[distvec[ii+1].second];
              Point p3=palm_points[distvec[ii+2].second];
              soln_circle=circleFromPoints(p1,p2,p3);//Final palm center,radius
              if(soln_circle.second!=0)
                break;
            }

            //Find avg palm centers for the last few frames to stabilize its centers, also find the avg radius
            //palm_centers.push_back(soln_circle);
            /*if(palm_centers.size()>10)
              palm_centers.erase(palm_centers.begin());
            
            Point palm_center;
            double radius=0;
            for(int i=0;i<palm_centers.size();i++)
            {
              palm_center+=palm_centers[i].first;
              radius+=palm_centers[i].second;
            }*/
            
              p_center=soln_circle.first;
              p_radius=soln_circle.second;

              cout << "p_center: " << p_center.x << ", " << p_center.y <<endl;
              cout << "p_radius: " << p_radius << endl; 
            //Draw the palm center and the palm circle
            //The size of the palm gives the depth of the hand
            //circle(frame,palm_center,5,Scalar(144,144,255),3);
            //circle(frame,palm_center,radius,Scalar(144,144,255),2);

            //Detect fingers by finding points that form an almost isosceles triangle with certain thesholds
            int no_of_fingers=0;
            for(int j=0;j<defects.size();j++)
            {
              int startidx=defects[j][0]; Point ptStart( large_contours[i][startidx] );
              int endidx=defects[j][1]; Point ptEnd( large_contours[i][endidx] );
              int faridx=defects[j][2]; Point ptFar( large_contours[i][faridx] );
              //X o--------------------------o Y
              double Xdist=sqrt(dist(p_center,ptFar));
              double Ydist=sqrt(dist(p_center,ptStart));
              double length=sqrt(dist(ptFar,ptStart));

              double retLength=sqrt(dist(ptEnd,ptFar));
              //Play with these thresholds to improve performance
              if(length<=3*p_radius&&Ydist>=0.4*p_radius&&length>=10&&retLength>=10&&max(length,retLength)/min(length,retLength)>=0.8)
                if(min(Xdist,Ydist)/max(Xdist,Ydist)<=0.8)
                {
                  if((Xdist>=0.1*p_radius&&Xdist<=1.3*p_radius&&Xdist<Ydist)||(Ydist>=0.1*p_radius&&Ydist<=1.3*p_radius&&Xdist>Ydist))
                    //line( frame, ptEnd, ptFar, Scalar(0,255,0), 1 ),
                    no_of_fingers++;
                }


            }
            
            no_of_fingers=min(5,no_of_fingers);
            cout<<"NO OF FINGERS: "<<no_of_fingers<<endl;
            if (no_of_fingers >= 3)
            {
                final_radius = p_radius;
                final_center = p_center;
                //p_center = palm_center;
            }   
          }
        }
  }




  /// Draw contours + hull results
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( size_t i = 0; i< large_contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, large_contours, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       drawContours( drawing, hull, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       
       if (p_radius != -1.0)
       {
          circle(drawing,final_center,5,Scalar(144,144,255),3);
          circle(drawing,final_center,final_radius,Scalar(144,144,255),2);
        }
        else
        {
        	final_center.x = -1.0;
        	final_center.y = -1.0;
        }
     }

  /// Show in a window
  namedWindow( "Hull demo", WINDOW_AUTOSIZE );
  imshow( "Hull demo", drawing );
  //waitKey(0);
  return final_center;
  
}