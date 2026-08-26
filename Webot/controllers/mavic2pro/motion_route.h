#ifndef MOTION_ROUTE_H
#define MOTION_ROUTE_H

#include <stdbool.h>

#define MOTION_ROUTE_MIN_DISTANCE_METERS 1.0

typedef struct {
  double x;
  double y;
} MotionRoutePoint;

typedef struct {
  MotionRoutePoint start;
  MotionRoutePoint destination;
  double distance_meters;
  double duration_seconds;
  double nominal_speed_meters_per_sec;
  double heading_radians;
} MotionRoutePlan;

typedef struct {
  MotionRoutePoint nominal_position;
  MotionRoutePoint nominal_velocity;
  double progress;
} MotionRouteSample;

bool motion_route_plan(
  MotionRoutePoint start,
  MotionRoutePoint destination,
  double duration_seconds,
  MotionRoutePlan *plan,
  const char **error);
MotionRouteSample motion_route_sample(const MotionRoutePlan *plan, double elapsed_seconds);
void motion_route_errors(
  const MotionRoutePlan *plan,
  MotionRoutePoint actual,
  double *along_track_error_meters,
  double *cross_track_error_meters);

#endif
