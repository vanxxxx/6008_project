#include "../motion_route.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_plan_and_samples(void) {
  MotionRoutePlan plan;
  const char *error = NULL;
  assert(motion_route_plan((MotionRoutePoint){0.0, 0.0}, (MotionRoutePoint){10.0, 0.0}, 40.0, &plan, &error));
  assert(fabs(plan.nominal_speed_meters_per_sec - 0.25) < 0.00001);
  MotionRouteSample start = motion_route_sample(&plan, 0.0);
  MotionRouteSample middle = motion_route_sample(&plan, 20.0);
  MotionRouteSample end = motion_route_sample(&plan, 40.0);
  assert(fabs(start.nominal_position.x) < 0.00001);
  assert(fabs(middle.nominal_position.x - 5.0) < 0.00001);
  assert(fabs(end.nominal_position.x - 10.0) < 0.00001);
  assert(fabs(end.nominal_velocity.x) < 0.00001);
}

static void test_distance_guard_and_unlimited_nominal_speed(void) {
  MotionRoutePlan plan;
  const char *error = NULL;
  assert(!motion_route_plan((MotionRoutePoint){0.0, 0.0}, (MotionRoutePoint){0.5, 0.0}, 40.0, &plan, &error));
  assert(error != NULL);
  assert(motion_route_plan((MotionRoutePoint){0.0, 0.0}, (MotionRoutePoint){20.0, 0.0}, 1.0, &plan, &error));
  assert(fabs(plan.nominal_speed_meters_per_sec - 20.0) < 0.00001);
}

static void test_cross_track_error_is_relative_to_the_nominal_line(void) {
  MotionRoutePlan plan;
  assert(motion_route_plan((MotionRoutePoint){0.0, 0.0}, (MotionRoutePoint){10.0, 0.0}, 40.0, &plan, NULL));
  double along = 0.0;
  double cross = 0.0;
  motion_route_errors(&plan, (MotionRoutePoint){5.0, -0.4}, &along, &cross);
  assert(fabs(along - 5.0) < 0.00001);
  assert(fabs(cross + 0.4) < 0.00001);
}

int main(void) {
  test_plan_and_samples();
  test_distance_guard_and_unlimited_nominal_speed();
  test_cross_track_error_is_relative_to_the_nominal_line();
  puts("motion_route_test: all tests passed");
  return 0;
}
