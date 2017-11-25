
#include "geometry2d.h"

Angle normalize_angle (Angle angle)
   {
   // wrap the angle at 2pi
   Angle fmod_angle = fmod(angle, 2.0 * pi);

   // normalize the angle to +/- pi
   if (fmod_angle > pi) fmod_angle = fmod_angle - (2.0 * pi);
   else if (fmod_angle < -pi) fmod_angle = (2.0 * pi) + fmod_angle;

   return fmod_angle;
   }

//
// Point2D member definitions
//

//static const int buffer_size;

void Point2D::write_to_buffer (
   char* &buf_ptr) const
   {
//   BufferIO::write_double_to_buffer(coords[0], buf_ptr);
//   BufferIO::write_double_to_buffer(coords[1], buf_ptr);
   }

void Point2D::read_from_buffer (
   char* &buf_ptr)
   {
//   BufferIO::read_double_from_buffer(buf_ptr, coords[0]);
//   BufferIO::read_double_from_buffer(buf_ptr, coords[1]);
   }

//
// end of Point2D member definitions
//

//
// LineSegment2D member definitions
//

Angle LineSegment2D::angle () const
   {
   const Coordinate dx = pt2.x() - pt1.x();
   const Coordinate dy = pt2.y() - pt1.y();
   Angle a = (equal_Coordinates(dx, 0.0))
      ? (((dy < 0.0) ? -pi : pi) / 2.0)  // vertical line
      : atan2(dy, dx);
      
   return a;
   }

bool LineSegment2D::point_is_on (
   const Point2D &pt) const
   {
   bool pt_is_on_seg = (pt == pt1) || (pt == pt2);

   if (!pt_is_on_seg) {
      // do simple fast gross checks to see if the point is in the
      // segment's bounding box
      BoundingCoords x_bounds(pt1.x(), pt2.x());
      if (x_bounds.coord_is_in(pt.x())) {
         BoundingCoords y_bounds(pt1.y(), pt2.y());
         if (y_bounds.coord_is_in(pt.y())) {
            // the point is in the bounding box
            // see if the point is on the line
            pt_is_on_seg = point_is_on_line(pt);
         }
      }
   }

   return pt_is_on_seg;
   }

bool LineSegment2D::point_is_on_line (
   const Point2D &pt) const
   {
   bool pt_on_line = false;
   
   if (equal_Coordinates(pt1.x(), pt2.x())) {
      // vertical line
      pt_on_line = equal_Coordinates(pt.x(), pt1.x());
   } else {
      // segment is not vertical
      // get the equation of the line
      const Coordinate m = (pt2.y() - pt1.y()) / (pt2.x() - pt1.x());
      const Coordinate b = pt1.y() - m * pt1.x();
      pt_on_line = equal_Coordinates(pt.y() - m * pt.x(), b);
   }
   
   return pt_on_line;
   }

bool LineSegment2D::point_is_to_the_right_of_line (
   const Point2D &pt) const
   {
   bool pt_is_to_the_right = false;
   
   const Coordinate dx = pt2.x() - pt1.x();
   const Coordinate dy = pt2.y() - pt1.y();
   if (equal_Coordinates(dx, 0.0))
      {  // vertical line
      pt_is_to_the_right = (dy > 0.0)
         ? pt.x() > pt1.x()   // straight up
         : pt.x() < pt1.x();  // straight down
      }
   else
      {  // nonvertical line
      Coordinate A, B, C;
      get_line_parameters(A, B, C);
      // solve for Y at pt's x
      const Coordinate pt_y = -((A * pt.x()) + C) / B;
      pt_is_to_the_right = (dx > 0.0)
         ? pt.y() < pt_y
         : pt.y() > pt_y;
      }
      
   return pt_is_to_the_right;
   }

Coordinate LineSegment2D::length () const
   {
   const Coordinate dx = pt2.x() - pt1.x();
   const Coordinate dy = pt2.y() - pt1.y();
   return sqrt((dx * dx) + (dy * dy));
   }

Coordinate LineSegment2D::length_sqrd () const
   {
   const Coordinate dx = pt2.x() - pt1.x();
   const Coordinate dy = pt2.y() - pt1.y();
   return (dx * dx) + (dy * dy);
   }

Point2D LineSegment2D::midpoint () const
   {
   return Point2D((pt1.x() + pt2.x()) / 2.0, (pt1.y() + pt2.y()) / 2.0);
   }

Point2D LineSegment2D::closest_point (
   const Point2D &pt) const
   {
   const Coordinate len_sqrd = length_sqrd();
   const Coordinate dx = pt2.x() - pt1.x();
   const Coordinate dy = pt2.y() - pt1.y();
   const Coordinate u =
      (((pt.x() - pt1.x()) * dx) +
       ((pt.y() - pt1.y()) * dy)) /
       len_sqrd;
   
   Point2D closest_pt_on_seg;
   if (u <= 0.0)
      {  // point is on or before pt1
      closest_pt_on_seg = pt1;
      }
   else if (u >= 1.0)
      {  // point is on or after pt2
      closest_pt_on_seg = pt2;
      }
   else
      {  // point is on a perpendicular to the segment
      closest_pt_on_seg.x() = pt1.x() + u * dx;
      closest_pt_on_seg.y() = pt1.y() + u * dy;
      }
   
   return closest_pt_on_seg;
   }
      
void LineSegment2D::get_line_parameters (
   Coordinate &A,
   Coordinate &B,
   Coordinate &C) const
   {
   A = pt1.y() - pt2.y();
   B = pt2.x() - pt1.x();
   C = pt1.x() * pt2.y() - pt2.x() * pt1.y();
   }

void LineSegment2D::get_slope_and_intercept (
   bool &is_vertical,
   Angle &slope,
   Coordinate &intercept) const
   {
   is_vertical = equal_Coordinates(pt1.x(), pt2.x());
   if (is_vertical)
      {
      slope = 0.0;
      intercept = 0.0;
      }
   else
      {  // nonvertical line
      const Coordinate dy = pt2.y() - pt1.y();
      const Coordinate dx = pt2.x() - pt1.x();
      slope = dy / dx;
      intercept = pt1.y() - (slope * pt1.x());
      }
   }
   
void LineSegment2D::extend (
   const Coordinate pt1_len,
   const Coordinate pt2_len,
   LineSegment2D &extended_seg) const
   {
   // compute the length of the segment
   const Coordinate dy = pt2.y() - pt1.y();
   const Coordinate dx = pt2.x() - pt1.x();
   const Coordinate seg_len = sqrt((dx * dx) + (dy * dy));

   const Angle pt1_extrapolation_ratio = pt1_len / seg_len;
   extended_seg.pt1.x() = pt1.x() - (dx * pt1_extrapolation_ratio);
   extended_seg.pt1.y() = pt1.y() - (dy * pt1_extrapolation_ratio);

   const Angle pt2_extrapolation_ratio = pt2_len / seg_len;
   extended_seg.pt2.x() = pt2.x() + (dx * pt2_extrapolation_ratio);
   extended_seg.pt2.y() = pt2.y() + (dy * pt2_extrapolation_ratio);
   }

void LineSegment2D::get_perpendicular_seg (
   const Coordinate seg_len,
   const Point2D &seg_midpoint,
   LineSegment2D &perpendicular_seg) const
   {
   const Angle this_angle = angle();
   transform_point2D(seg_midpoint, seg_len / 2.0, this_angle + pi / 2.0,
      perpendicular_seg.pt1);
   perpendicular_seg.pt2.x() = seg_midpoint.x() -
      (perpendicular_seg.pt1.x() - seg_midpoint.x());
   perpendicular_seg.pt2.y() = seg_midpoint.y() -
      (perpendicular_seg.pt1.y() - seg_midpoint.y());
   }

void LineSegment2D::write_to_buffer (
   char* &buf_ptr) const
   {
   pt1.write_to_buffer(buf_ptr);
   pt2.write_to_buffer(buf_ptr);
   }
   
void LineSegment2D::read_from_buffer (
   char* &buf_ptr)
   {
   pt1.read_from_buffer(buf_ptr);
   pt2.read_from_buffer(buf_ptr);
   }
   
//
// end of LineSegment2D member definitions
//

void find_line_intersection_2D (
   const Coordinate A1,
   const Coordinate B1,
   const Coordinate C1,
   const Coordinate A2,
   const Coordinate B2,
   const Coordinate C2,
   bool &lines_intersect,
   Point2D &intersection)
   {
   lines_intersect = false;
   
   // first check to see if the lines are parallel
   const Coordinate A1B2minusA2B1 = (A1 * B2) - (A2 * B1);
   if (!equal_Coordinates(A1B2minusA2B1, 0.0))
      {  // the lines are not parallel
      lines_intersect = true;
      intersection.x() = ((B1 * C2) - (B2 * C1)) / A1B2minusA2B1;
      intersection.y() = ((C1 * A2) - (C2 * A1)) / A1B2minusA2B1;
      }
   }

void find_segment_intersection_2D (
   const LineSegment2D &seg1,
   const LineSegment2D &seg2,
   bool &segs_intersect,
   Point2D &intersection)
   {
   segs_intersect = false;

   // segment 1 line parameters
   float A1, B1, C1;
   seg1.get_line_parameters(A1, B1, C1);

   // segment 2 line parameters
   float A2, B2, C2;
   seg2.get_line_parameters(A2, B2, C2);

   float det = A1*B2 - A2*B1;
   if (equal_Coordinates(det, 0.0))
      {  // lines are parallel
      // are they colinear?
      if (equal_Coordinates(C1, C2))
         {
         // lines are colinear
         if (equal_Coordinates(seg1.pt1.x(), seg1.pt2.x()))
            {  // vertical line - compare y coordinates
            BoundingCoords seg1_y_bounds(seg1.pt1.y(), seg1.pt2.y());
            BoundingCoords seg2_y_bounds(seg2.pt1.y(), seg2.pt2.y());
            if (seg1_y_bounds.coord_is_in(seg2.pt1.y())) {
               segs_intersect = true;
               intersection = seg2.pt1;
               }
            else if (seg1_y_bounds.coord_is_in(seg2.pt2.y())) {
               segs_intersect = true;
               intersection = seg2.pt2;
               }
            else if (seg2_y_bounds.coord_is_in(seg1.pt1.y())) {
               segs_intersect = true;
               intersection = seg1.pt1;
               }
            else if (seg2_y_bounds.coord_is_in(seg1.pt2.y())) {
               segs_intersect = true;
               intersection = seg1.pt2;
               }
            }
         else
            {  // nonvertical line - compare x coordinates
            BoundingCoords seg1_x_bounds(seg1.pt1.x(), seg1.pt2.x());
            BoundingCoords seg2_x_bounds(seg2.pt1.x(), seg2.pt2.x());
            if (seg1_x_bounds.coord_is_in(seg2.pt1.x())) {
               segs_intersect = true;
               intersection = seg2.pt1;
               }
            else if (seg1_x_bounds.coord_is_in(seg2.pt2.x())) {
               segs_intersect = true;
               intersection = seg2.pt2;
               }
            else if (seg2_x_bounds.coord_is_in(seg1.pt1.x())) {
               segs_intersect = true;
               intersection = seg1.pt1;
               }
            else if (seg2_x_bounds.coord_is_in(seg1.pt2.x())) {
               segs_intersect = true;
               intersection = seg1.pt2;
               }
            }
         }
      }
   else
      { // Lines are not parallel
      // compute intersect point
//      intersection.x() = (B2*C1 - B1*C2)/det;
//      intersection.y() = (A1*C2 - A2*C1)/det;
      intersection.x() = ((B1 * C2) - (B2 * C1)) / det;
      intersection.y() = ((C1 * A2) - (C2 * A1)) / det;

      // check that intersect point is on a segment
      const BoundingBox2D seg1_bbox(seg1.bounding_box());
      const BoundingBox2D seg2_bbox(seg2.bounding_box());
      segs_intersect = seg1_bbox.point_is_in(intersection) && 
                       seg2_bbox.point_is_in(intersection);
      }
   }

void get_segment_2D_overlap (
   const LineSegment2D &seg1,
   const LineSegment2D &seg2,
   bool &segments_overlap,
   LineSegment2D &overlap)
   {
   int num_overlap_pts = 0;
   Point2D overlap_pts[2];
   
   if (seg1.point_is_on(seg2.pt1)) {
      overlap_pts[num_overlap_pts] = seg2.pt1;
      ++num_overlap_pts;
      }
   if (seg1.point_is_on(seg2.pt2)) {
      overlap_pts[num_overlap_pts] = seg2.pt2;
      ++num_overlap_pts;
      }
   if ((num_overlap_pts < 2) &&
       seg2.point_is_on(seg1.pt1) &&
       !((num_overlap_pts == 1) && (seg1.pt1 == overlap_pts[0]))) {
      overlap_pts[num_overlap_pts] = seg1.pt1;
      ++num_overlap_pts;
      }
   if ((num_overlap_pts == 1) &&
       seg2.point_is_on(seg1.pt2) &&
       !((num_overlap_pts == 1) && (seg1.pt2 == overlap_pts[0]))) {
      overlap_pts[num_overlap_pts] = seg1.pt2;
      ++num_overlap_pts;
      }
      
   segments_overlap = (num_overlap_pts == 2);
   overlap.pt1 = overlap_pts[0];
   overlap.pt2 = overlap_pts[1];
   }

void transform_point2D (
   const Point2D &pt,
   const Coordinate distance,
   const Angle angle,
   Point2D &transformed_pt)
   {
   transformed_pt.x() = pt.x() + distance * cos(angle);
   transformed_pt.y() = pt.y() + distance * sin(angle);
   }

Coordinate taxi_distance2D (
   const Point2D &pt1,
   const Point2D &pt2)
   {
   return fabs(pt2.x() - pt1.x()) + fabs(pt2.y() - pt1.y());
   }

Coordinate euclidean_distance2D (
   const Point2D &pt1,
   const Point2D &pt2)
   {
   const Coordinate dx = pt2.x() - pt1.x();
   const Coordinate dy = pt2.y() - pt1.y();
   return sqrt((dx * dx) + (dy * dy));
   }

//
// ArcSegment2D member definitions
//

ArcSegment2D::ArcSegment2D () :
   arc_radius(0.0),
   arc_start_angle(0.0),
   arc_end_angle(0.0)
   {
   }

ArcSegment2D::ArcSegment2D (
   const Point2D &p1,
   const Point2D &p2,
   const Point2D &ctr,
   const arc_dir direction) :
   pt1((direction == ad_ccw) ? p1 : p2),  // normalize to ccw arc
   pt2((direction == ad_ccw) ? p2 : p1),
   arc_center(ctr)
   {
   set_radius_and_angles();
   }

ArcSegment2D::ArcSegment2D (
   const Point2D &p1,
   const Point2D &ctr,
   const Coordinate arc_length,
   const arc_dir direction)
   {
   LineSegment2D p1_seg(ctr, p1);
   arc_center = ctr;
   arc_radius = p1_seg.length();
   const Coordinate circumference = 2.0 * pi * arc_radius;
   const Angle angle = (arc_length / circumference) * 2.0 * pi;
   switch (direction) {
      case ad_cw :
         pt2 = p1;
         arc_end_angle = p1_seg.angle();
         arc_start_angle = arc_end_angle - angle;
         transform_point2D(arc_center, arc_radius, arc_start_angle, pt1);
         break;
      case ad_ccw :
         pt1 = p1;
         arc_start_angle = p1_seg.angle();
         arc_end_angle = arc_start_angle + angle;
         transform_point2D(arc_center, arc_radius, arc_end_angle, pt2);
         break;
      }
   }
   
void ArcSegment2D::clear ()
   {
   arc_radius = 0.0;
   }

bool ArcSegment2D::angle_is_in_range (
   const Angle angle) const
   {
   bool in_range =
      is_empty()
      ? false
      : (arc_start_angle > arc_end_angle)
         ? ((angle >= arc_start_angle) || (angle <= arc_end_angle))
         : ((angle >= arc_start_angle) && (angle <= arc_end_angle));
   return in_range;
   }

const Coordinate ArcSegment2D::length () const
   {
   Angle arc_angle = arc_end_angle - arc_start_angle;
   if (arc_start_angle > arc_end_angle)
      arc_angle += (2.0 * pi);
   const Angle arc_angle_fraction = arc_angle / (2.0 * pi);
   
   const Coordinate circumference = 2.0 * pi * arc_radius;
   return circumference * arc_angle_fraction;   
   }

void ArcSegment2D::get_polyline (
   PolyLine2D &polyline) const
   {
   const Angle angle_increment = 0.15; // about 10 degrees
   polyline.clear();
   
   if (!is_empty())
      {
      polyline.push_back(pt1);

      Angle angle = arc_start_angle + angle_increment;
      Angle end_angle = arc_end_angle;
      if (arc_start_angle > arc_end_angle)
         end_angle += (2.0 * pi);
      while (angle < end_angle)
         {
         Point2D arc_pt;
         transform_point2D(arc_center, arc_radius, angle, arc_pt);
         polyline.push_back(arc_pt);
         
         angle += angle_increment;
         }
      polyline.push_back(pt2);
      }
   }

void ArcSegment2D::write_to_buffer (
   char* &buf_ptr) const
   {
   pt1.write_to_buffer(buf_ptr);
   pt2.write_to_buffer(buf_ptr);
   arc_center.write_to_buffer(buf_ptr);
   }
   
void ArcSegment2D::read_from_buffer (
   char* &buf_ptr)
   {
   pt1.read_from_buffer(buf_ptr);
   pt2.read_from_buffer(buf_ptr);
   arc_center.read_from_buffer(buf_ptr);
   set_radius_and_angles();
   }

void ArcSegment2D::set_radius_and_angles()
   {
   arc_radius = euclidean_distance2D(pt1, arc_center);
   LineSegment2D pt1_seg(arc_center, pt1);
   arc_start_angle = pt1_seg.angle();
   LineSegment2D pt2_seg(arc_center, pt2);
   arc_end_angle = pt2_seg.angle();   
   }
   
//
// end of ArcSegment2D member definitions
//

namespace {

   Coordinate dy_on_arc_circle (
      const Coordinate radius,
      const Coordinate dx)
      {
      return sqrt((radius * radius) - (dx * dx));
      }
      
   void get_point_on_line (
      const float slope,
      const Coordinate intercept,
      const float A,
      const float B,
      const float sqrt_det,
      Point2D &pt_on_line)
      {
      const Coordinate intersection_x = ((-B) + sqrt_det) / (2.0 * A);
      pt_on_line.x() = intersection_x;
      pt_on_line.y() = (slope * intersection_x) + intercept;
      }
       
   bool intersection_in_range (
      const ArcSegment2D &arc_seg,
      const LineSegment2D &line_seg,
      const Point2D &intersection)
      {
      const LineSegment2D intersection_radial_seg(arc_seg.center(), intersection);
      const Angle intersection_angle = intersection_radial_seg.angle();
      
      return
         line_seg.bounding_box().point_is_in(intersection) &&
         arc_seg.angle_is_in_range(intersection_angle);
      }

int var1 = 0;
     
}  // namespace

void find_arc_segment_intersection_2D (
   const ArcSegment2D &arc_seg,
   const LineSegment2D &line_seg,
   int &num_intersections,    // 0, 1, or 2
   Point2D &intersection1,
   Point2D &intersection2)
   {
   num_intersections = 0;
   
   // first we find any intersection between a full cirle and infinite line
   bool is_vertical;
   float slope;
   Coordinate intercept;
   line_seg.get_slope_and_intercept(is_vertical, slope, intercept);
   if (is_vertical)
      {
      const Coordinate circle_left_x = arc_seg.center().x() - arc_seg.radius();
      const Coordinate circle_right_x = arc_seg.center().x() + arc_seg.radius();
      if (equal_Coordinates(line_seg.pt1.x(), circle_left_x) ||
          equal_Coordinates(line_seg.pt1.x(), circle_right_x))
         {  // vertical line is tangent to left or right side of circle
         num_intersections = 1;
         intersection1 = Point2D(line_seg.pt1.x(), arc_seg.center().y());
         }
      else if ((line_seg.pt1.x() > circle_left_x) &&
               (line_seg.pt1.x() < circle_right_x))
         {  // vertical line cuts through circle
         num_intersections = 2;
         const Coordinate dx = line_seg.pt1.x() - arc_seg.center().x();
         const Coordinate dy = dy_on_arc_circle(arc_seg.radius(), dx);
         intersection1 = Point2D(line_seg.pt1.x(), arc_seg.center().y() + dy);
         intersection2 = Point2D(line_seg.pt1.x(), arc_seg.center().y() - dy);
         }
      }
   else
      {  // nonvertical line
      // compute determinant of intersection quadratic
      const float k = intercept - arc_seg.center().y();
      const float A = 1.0 + (slope * slope);
      const float B = (2.0 * slope * k) - (2.0 * arc_seg.center().x());
      const float C = (arc_seg.center().x() * arc_seg.center().x()) +
                       (k * k) -
                       (arc_seg.radius() * arc_seg.radius());
      const float det = (B * B) - (4.0 * A * C);
      if (equal_Coordinates(det, 0.0))
         {  // tangent to circle
         num_intersections = 1;
         get_point_on_line(slope, intercept, A, B, 0.0, intersection1);
         }
      else if (det > 0.0)
         {
         const float sqrt_det = sqrt(det);
         num_intersections = 2;
         get_point_on_line(slope, intercept, A, B, sqrt_det, intersection1);
         get_point_on_line(slope, intercept, A, B, -sqrt_det, intersection2);
         }
      }

   // now qualify the intersections to make sure they are in the range of
   // the arc sweep angle and line segment endpoints
   if ((num_intersections == 2) &&
       !intersection_in_range(arc_seg, line_seg, intersection2))
      --num_intersections;
      
   if ((num_intersections > 0) &&
       !intersection_in_range(arc_seg, line_seg, intersection1))
      {
      --num_intersections;
      if (num_intersections > 0)
         intersection1 = intersection2;
      }
   }
