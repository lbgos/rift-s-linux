#include <stdlib.h>
#include <stdio.h>

#include "tracking/t_led_models.h"

/* Set to 0 to use 3D euclidean distance to sort neighbours,
 * 1 to use orthographic projected (undistorted) 2D distance
 * with the anchor LED is forward-facing */
#define PROJECTED_DISTANCE 0
#define DUMP_FULL_DEBUG 0

#if DUMP_FULL_DEBUG
#define DEBUG(s, ...) printf(s, __VA_ARGS__)
#else
#define DEBUG(s, ...)
#endif

void
t_constellation_led_model_init(uint8_t device_id,
                               struct xrt_pose *P_device_model,
                               struct t_constellation_led_model *led_model,
                               uint8_t num_leds,
                               uint8_t num_bounding_points)
{
	led_model->id = device_id;
	if (P_device_model != NULL) {
		led_model->P_device_model = *P_device_model;
		math_pose_invert(&led_model->P_device_model, &led_model->P_model_device);
	} else {
		led_model->P_device_model = led_model->P_model_device = (struct xrt_pose)XRT_POSE_IDENTITY;
	}
	led_model->leds = calloc(num_leds, sizeof(struct t_constellation_led));
	led_model->num_leds = num_leds;
	led_model->min_acquisition_leds = 0;
	led_model->bounding_points = calloc(num_bounding_points, sizeof(struct t_constellation_bounding_point));
	led_model->num_bounding_points = num_bounding_points;
}

void
t_constellation_led_model_dump(struct t_constellation_led_model *led_model, const char *desc)
{
	int i;
	printf("LED model: %s\n", desc);
	printf("x,y,z,tx,ty,tz,d\n");
	for (i = 0; i < led_model->num_leds; i++) {
		struct t_constellation_led *p = led_model->leds + i;

		// printf("{ .pos = {%f,%f,%f}, .dir={%f,%f,%f} },\n", p->pos.x, p->pos.y, p->pos.z, p->dir.x, p->dir.y,
		//        p->dir.z);
		printf("%f,%f,%f,%f,%f,%f,%f\n", p->pos.x, p->pos.y, p->pos.z, p->dir.x, p->dir.y, p->dir.z,
		       p->radius_mm / 1000.0);
	}
}

void
t_constellation_led_model_clear(struct t_constellation_led_model *led_model)
{
	free(led_model->leds);
	free(led_model->bounding_points);
	led_model->leds = NULL;
	led_model->bounding_points = NULL;
	led_model->num_leds = 0;
	led_model->num_bounding_points = 0;
}

struct led_candidate_sort_entry
{
	struct t_constellation_led *led;
	double distance;
};

#define POW2(_x) ((_x) * (_x))

static double
calc_led_dist(const struct t_constellation_search_led_candidate *c,
              const struct t_constellation_led *anchor,
              struct xrt_vec3 *out_led_pos)
{
	struct xrt_vec3 led_pos;

#if PROJECTED_DISTANCE
	struct xrt_vec3 tmp;

	tmp = m_vec3_add(anchor->pos, c->pose.position);
	math_quat_rotate_vec3(&c->pose.orientation, &tmp, &led_pos);

	if (out_led_pos)
		*out_led_pos = led_pos;

	if (led_pos.z > 0.0) {
		return sqrt(POW2(led_pos.x / led_pos.z) + POW2(led_pos.y / led_pos.z));
	}
	return sqrt(POW2(led_pos.x) + POW2(led_pos.y));
#else
	led_pos = m_vec3_add(anchor->pos, c->pose.position);

	if (out_led_pos)
		*out_led_pos = led_pos;

	return m_vec3_len(led_pos);
#endif
}

static int
compare_led_distance(const void *elem1, const void *elem2)
{
	const struct led_candidate_sort_entry *l1 = (const struct led_candidate_sort_entry *)elem1;
	const struct led_candidate_sort_entry *l2 = (const struct led_candidate_sort_entry *)elem2;

	if (l1->distance > l2->distance)
		return 1;
	if (l1->distance < l2->distance)
		return -1;

	return 0;
}

struct t_constellation_search_led_candidate *
t_constellation_search_led_candidate_new(struct t_constellation_led *led, struct t_constellation_led_model *led_model)
{
	int i;

	struct t_constellation_search_led_candidate *c = calloc(1, sizeof(struct t_constellation_search_led_candidate));
	c->led = led;

	/* Calculate the pose that places this LED forward-facing at 0,0,0
	 * and then calculate the distance for all visible LEDs when (orthographic)
	 * projected in that pose */
	const struct xrt_vec3 fwd = (struct xrt_vec3){0.0, 0.0, 1.0};

	c->pose.position = m_vec3_inverse(led->pos);
	math_quat_from_vec_a_to_vec_b(&led->dir, &fwd, &c->pose.orientation);

	struct led_candidate_sort_entry array[256];

	for (i = 0; i < led_model->num_leds; i++) {
		struct t_constellation_led *cur = led_model->leds + i;

		if (cur == led)
			continue; // Don't put the current LED in its own neighbour list

		if (m_vec3_dot(led->dir, cur->dir) <= 0)
			continue; // Normals are more than 90 degrees apart - these are mutually exclusive LEDs

		struct led_candidate_sort_entry *entry = &array[c->num_neighbours];
		entry->led = cur;
		entry->distance = calc_led_dist(c, cur, NULL);

		c->num_neighbours++;
	}

	if (c->num_neighbours > 1) {
		qsort(array, c->num_neighbours, sizeof(struct led_candidate_sort_entry), compare_led_distance);

		c->neighbours = calloc(c->num_neighbours, sizeof(struct t_constellation_led *));
		for (i = 0; i < c->num_neighbours; i++) {
			c->neighbours[i] = array[i].led;
		}
	}

#if DUMP_FULL_DEBUG
	DEBUG("Have %u neighbours for LED %u (%f,%f,%f) dir (%f,%f,%f) ", c->num_neighbours, c->led->id, c->led->pos.x,
	      c->led->pos.y, c->led->pos.z, c->led->dir.x, c->led->dir.y, c->led->dir.z);
	DEBUG(" front-facing rotation (%f,%f,%f,%f):\n", c->pose.orientation.x, c->pose.orientation.y,
	      c->pose.orientation.z, c->pose.orientation.w);

	for (i = 0; i < c->num_neighbours; i++) {
		struct t_constellation_led *cur = c->neighbours[i];
		struct xrt_vec3 led_pos;
		double distance = calc_led_dist(c, cur, &led_pos);

		struct xrt_vec3 tmp;
		math_quat_rotate_vec3(&c->pose.orientation, &cur->dir, &tmp);
		float theta = m_vec3_dot(tmp, fwd);

		DEBUG(
		    "  LED id %2u @ %10.7f %10.7f %10.7f dir %10.7f %10.7f %10.7f -> %10.7f %10.7f dist %10.7f (dir "
		    "%f,%f,%f) angle %f\n",
		    cur->id, cur->pos.x, cur->pos.y, cur->pos.z, cur->dir.x, cur->dir.y, cur->dir.z, led_pos.x,
		    led_pos.y, distance, tmp.x, tmp.y, tmp.z, acosf(theta) / M_PI * 180.0);
	}
#endif

	return c;
}

void
t_constellation_search_led_candidate_free(struct t_constellation_search_led_candidate *candidate)
{
	free(candidate->neighbours);
	free(candidate);
}

struct t_constellation_search_model *
t_constellation_search_model_new(struct t_constellation_led_model *led_model)
{
	struct t_constellation_search_model *m = calloc(1, sizeof(struct t_constellation_search_model));
	int i;

	m->id = led_model->id;
	m->led_model = led_model;

	m->points = calloc(led_model->num_leds, sizeof(struct t_constellation_search_led_candidate *));
	m->num_points = led_model->num_leds;

	for (i = 0; i < led_model->num_leds; i++) {
		m->points[i] = t_constellation_search_led_candidate_new(led_model->leds + i, led_model);
	}

	return m;
}

void
t_constellation_search_model_free(struct t_constellation_search_model *model)
{
	int i;

	for (i = 0; i < model->num_points; i++) {
		t_constellation_search_led_candidate_free(model->points[i]);
	}
	free(model->points);
	free(model);
}
