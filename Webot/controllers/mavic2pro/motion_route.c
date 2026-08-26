#include "motion_route.h"

#include <math.h>
#include <stddef.h>

bool motion_route_plan(
  MotionRoutePoint start,
  MotionRoutePoint destination,
  double duration_seconds,
  MotionRoutePlan *plan,
  const char **error) {
  if (error != NULL)
    *error = NULL;
  if (plan == NULL || !isfinite(start.x) || !isfinite(start.y) || !isfinite(destination.x) ||
      !isfinite(destination.y) || !isfinite(duration_seconds) || duration_seconds <= 0.0) {
    if (error != NULL)
      *error = "The route contains invalid coordinates or duration";
    return false;
  }

  const double delta_x = destination.x - start.x;
  const double delta_y = destination.y - start.y;
  const double distance = hypot(delta_x, delta_y);
  if (distance < MOTION_ROUTE_MIN_DISTANCE_METERS) {
    if (error != NULL)
      *error = "The destination must be at least 1.0 m from the captured start point";
    return false;
  }
  const double speed = distance / duration_seconds;

  *plan = (MotionRoutePlan){
    .start = start,
    .destination = destination,
    .distance_meters = distance,
    .duration_seconds = duration_seconds,
    .nominal_speed_meters_per_sec = speed,
    .heading_radians = atan2(delta_y, delta_x),
  };
  return true;
}

MotionRouteSample motion_route_sample(const MotionRoutePlan *plan, double elapsed_seconds) {
  if (plan == NULL || plan->duration_seconds <= 0.0)
    return (MotionRouteSample){0};
  const double progress = fmin(1.0, fmax(0.0, elapsed_seconds / plan->duration_seconds));
  const double delta_x = plan->destination.x - plan->start.x;
  const double delta_y = plan->destination.y - plan->start.y;
  const bool moving = elapsed_seconds >= 0.0 && elapsed_seconds < plan->duration_seconds;
  return (MotionRouteSample){
    .nominal_position = {
      .x = plan->start.x + delta_x * progress,
      .y = plan->start.y + delta_y * progress,
    },
    .nominal_velocity = moving ? (MotionRoutePoint){
      .x = delta_x / plan->duration_seconds,
      .y = delta_y / plan->duration_seconds,
    } : (MotionRoutePoint){0},
    .progress = progress,
  };
}

void motion_route_errors(
  const MotionRoutePlan *plan,
  MotionRoutePoint actual,
  double *along_track_error_meters,
  double *cross_track_error_meters) {
  if (along_track_error_meters != NULL)
    *along_track_error_meters = 0.0;
  if (cross_track_error_meters != NULL)
    *cross_track_error_meters = 0.0;
  if (plan == NULL || plan->distance_meters <= 0.0)
    return;

  const double direction_x = (plan->destination.x - plan->start.x) / plan->distance_meters;
  const double direction_y = (plan->destination.y - plan->start.y) / plan->distance_meters;
  const double offset_x = actual.x - plan->start.x;
  const double offset_y = actual.y - plan->start.y;
  if (along_track_error_meters != NULL)
    *along_track_error_meters = offset_x * direction_x + offset_y * direction_y;
  if (cross_track_error_meters != NULL)
    *cross_track_error_meters = -offset_x * direction_y + offset_y * direction_x;
}
