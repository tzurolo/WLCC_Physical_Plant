//
// 2D and 3D Geometry support
//

#include <list>
#include <math.h>
#include <cstdio>
//#include "BufferIO.h"

#ifndef GEMOETRY2D_LOADED
#define GEMOETRY2D_LOADED

const float pi = 3.1415926535897932384626433832795;

typedef float Coordinate;
typedef float Angle;

inline bool equal_Coordinates (
    const Coordinate c1,
    const Coordinate c2) {
    return (fabs(c2 - c1) < 0.0001);
    }

// nomalizes an angle to +/- pi
extern Angle normalize_angle (const Angle angle);

// need a struct for the point so we can put it in a list
class Point2D {
   public:
      Point2D ()
         { }
      Point2D (
         const Coordinate initial_x,
         const Coordinate initial_y)
         {
         coords[0] = initial_x;
         coords[1] = initial_y;
         }
      Point2D (
         const Point2D &initial_pt)
         {
         coords[0] = initial_pt.x();
         coords[1] = initial_pt.y();
         }
      void clear ()
         {
         coords[0] = 0.0;
         coords[1] = 0.0;
         }
      const Coordinate* coordinates () const
         { return coords; }
      const Coordinate& coord (
         const int i) const
         { return coords[i]; }
      Coordinate& coord (
         const int i)
         { return coords[i]; }
      const Coordinate& x () const
         { return coords[0]; }
      Coordinate& x ()
         { return coords[0]; }
      const Coordinate& y () const
         { return coords[1]; }
      Coordinate& y ()
         { return coords[1]; }
      bool operator== (
         const Point2D &rhs) const
         { return equal_Coordinates(coords[0], rhs.coords[0]) &&
                  equal_Coordinates(coords[1], rhs.coords[1]); }
      // generates [x, y]. buf is assumed to have enough space
      void print (
         char* buf)
         { sprintf(buf, "[%f, %f]", coords[0], coords[1]); }
//      enum { buffer_size = 2 * BufferIO::double_buffer_size};
      void write_to_buffer (
         char* &buf_ptr) const;
      void read_from_buffer (
         char* &buf_ptr);

   private:
      Coordinate coords[2];
   };

typedef std::list<Point2D> PolyLine2D;

struct BoundingCoords {
   BoundingCoords () :
      has_coords(false)
      { }
   BoundingCoords (
      const Coordinate initial_coord) :
      has_coords(true),
      min_coord(initial_coord),
      max_coord(initial_coord)
      { }
   BoundingCoords (
      const Coordinate initial_coord1,
      const Coordinate initial_coord2) :
      has_coords(true),
      min_coord(initial_coord1),
      max_coord(initial_coord1) {
      update(initial_coord2);
      }
   void clear ()
      { has_coords = false; }
   void update (
      const Coordinate new_coord) {
      if (has_coords)
         {
         if (new_coord < min_coord) min_coord = new_coord;
         if (new_coord > max_coord) max_coord = new_coord;
         }
      else
         {
         has_coords = true;
         min_coord = new_coord;
         max_coord = new_coord;
         }
      }
   bool is_defined () const
      { return has_coords; }
   bool coord_is_in (
      const Coordinate c) const {
      return
         has_coords && 
         (((c > min_coord) && (c < max_coord)) ||
         equal_Coordinates(c, min_coord) ||
         equal_Coordinates(c, max_coord));
      }

   bool has_coords;
   Coordinate min_coord;
   Coordinate max_coord;
   };

struct BoundingBox2D {
   BoundingBox2D ()
      { }
   BoundingBox2D (
      const Point2D &initial_pt) :
      x_bounds(initial_pt.x()),
      y_bounds(initial_pt.y())
      { }
   BoundingBox2D (
      const Point2D &initial_pt1,
      const Point2D &initial_pt2) :
      x_bounds(initial_pt1.x(), initial_pt2.x()),
      y_bounds(initial_pt1.y(), initial_pt2.y())
      { }
   void clear () {
      x_bounds.clear();
      y_bounds.clear();
      }
   void update (
      const Point2D &new_pt) {
      x_bounds.update(new_pt.x());
      y_bounds.update(new_pt.y());
      }
   bool is_defined () const
      { return x_bounds.is_defined() && y_bounds.is_defined(); }
   bool point_is_in (
      const Point2D &pt) const {
      return is_defined() &&
             x_bounds.coord_is_in(pt.x()) && y_bounds.coord_is_in(pt.y());
      }   
   BoundingCoords x_bounds;
   BoundingCoords y_bounds;
   };

// line segment explicitly defined by two points
struct LineSegment2D {
   LineSegment2D ()
      { }
   LineSegment2D (
      const Point2D &p1,
      const Point2D &p2) :
      pt1(p1),
      pt2(p2)
      { }
   LineSegment2D (
      const Coordinate pt1_x,
      const Coordinate pt1_y,
      const Coordinate pt2_x,
      const Coordinate pt2_y) :
      pt1(pt1_x, pt1_y),
      pt2(pt2_x, pt2_y)
      { }
   // returns the angle (-pi..pi) of the segment in radians
   Angle angle () const;
   bool point_is_on (
      const Point2D &pt) const;
   // returns true if the point is on the infinite line defined by
   // this segments endpoint
   bool point_is_on_line (
      const Point2D &pt) const;
   // returns true if the given point is in the half-plane to the
   // right of this segment
   bool point_is_to_the_right_of_line (
      const Point2D &pt) const;
   Coordinate length () const;
   Coordinate length_sqrd () const;
   Point2D midpoint () const;
   Point2D closest_point (
      const Point2D &pt) const;
   // returns the coefficients of Ax + By + C = 0
   void get_line_parameters (
       Coordinate &A,
       Coordinate &B,
       Coordinate &C) const;
   void get_slope_and_intercept (
      bool &is_vertical,
      Angle &slope,
      Coordinate &intercept) const;
   BoundingBox2D bounding_box () const {
      return BoundingBox2D(pt1, pt2);
      }
   // extrapolates the line segment out by pt1_len off of pt1
   // and by pt2_len off of pt2. Note that you can also contract
   // the line by using negative values for pt1_len or pt2_len
   void extend (
      const Coordinate pt1_len,
      const Coordinate pt2_len,
      LineSegment2D &extended_seg) const;
   // returns a segment perpendicular to this one centered on the
   // given point
   void get_perpendicular_seg (
      const Coordinate seg_len,
      const Point2D &seg_midpoint,
      LineSegment2D &perpendicular_seg) const;
//   enum { buffer_size = 2 * Point2D::buffer_size };
   void write_to_buffer (
      char* &buf_ptr) const;
   void read_from_buffer (
      char* &buf_ptr);
   
   Point2D pt1;
   Point2D pt2;
   };

// returns true if the two lines intersect. The lines are
// given in Ax + By + C = 0 format
void find_line_intersection_2D (
   const Coordinate A1,
   const Coordinate B1,
   const Coordinate C1,
   const Coordinate A2,
   const Coordinate B2,
   const Coordinate C2,
   bool &lines_intersect,
   Point2D &intersection);

// returns true if the two segments intersect in any way
void find_segment_intersection_2D (
   const LineSegment2D &seg1,
   const LineSegment2D &seg2,
   bool &segs_intersect,
   Point2D &intersection);

// returns the overlapping portion of two segments, if any.
// For there to be an overlap the segments must
// be colinear (crossing lines are not considered
// an overlap by this function). The overlapping portion,
// if any, is a line segment
void get_segment_2D_overlap (
   const LineSegment2D &seg1,
   const LineSegment2D &seg2,
   bool &segments_overlap,
   LineSegment2D &overlap);

// translates the given point by the given distance in the direction
// of the given angle
void transform_point2D (
   const Point2D &pt,
   const Coordinate distance,
   const Angle angle,
   Point2D &transformed_pt);

// a faster (than Euclidean) but cruder distance metric
Coordinate taxi_distance2D (
   const Point2D &pt1,
   const Point2D &pt2);

Coordinate euclidean_distance2D (
   const Point2D &pt1,
   const Point2D &pt2);

// arc segment defined by two points, a center, and a direction. Internally
// the arc is normalized to a ccw arc
struct ArcSegment2D {
      enum arc_dir {
          ad_cw,
          ad_ccw
      };
      ArcSegment2D ();
      ArcSegment2D (
         const Point2D &p1,
         const Point2D &p2,
         const Point2D &ctr,
         const arc_dir direction);
      ArcSegment2D (
         const Point2D &p1,
         const Point2D &ctr,
         const Coordinate arc_length,
         const arc_dir direction);
      void clear ();
      const Point2D& center () const
         { return arc_center; }
      const Coordinate& radius () const
         { return arc_radius; }
      const Angle start_angle () const
         { return arc_start_angle; }
      const Point2D& start_pt () const
         { return pt1; }
      const Angle end_angle () const
         { return arc_end_angle; }
      const Point2D& end_pt () const
         { return pt2; }
      const Coordinate length () const;
      const bool is_empty () const
         { return arc_radius == 0.0; }
      
      // returns true if the given angle is within the sweep of the arc
      // from start angle to end angle
      bool angle_is_in_range (
         const Angle angle) const;

      void get_polyline (
         PolyLine2D &polyline) const;

//      enum { buffer_size = 3 * Point2D::buffer_size };
      void write_to_buffer (
         char* &buf_ptr) const;
      void read_from_buffer (
         char* &buf_ptr);

   private:
   
      // normalized to ccw arc
      Point2D pt1;
      Point2D pt2;
      Point2D arc_center;
      Coordinate arc_radius;
      Angle arc_start_angle;
      Angle arc_end_angle;
      
      void set_radius_and_angles ();
      
   };

void find_arc_segment_intersection_2D (
   const ArcSegment2D &arc_seg,
   const LineSegment2D &line_seg,
   int &num_intersections,    // 0, 1, or 2
   Point2D &intersection1,
   Point2D &intersection2);
   
#endif  /* GEMOETRY2D_LOADED */