#include <stdio.h>
#include <stdlib.h>

#include "vec.h"

#include "motion/tracking/tracking_struct.h"

size_t _tracking_get_track_time(const RoI4track_t track_begin, const RoI4track_t track_end) {
    return track_end.frame - track_begin.frame;
}

size_t tracking_get_track_time(const vec_track_t tracks, const size_t t) {
    return _tracking_get_track_time(tracks[t].begin, tracks[t].end);
}

size_t tracking_count_objects(const vec_track_t tracks) {
    size_t n_tracks = vector_size(tracks);
    unsigned real_n_tracks = 0;
    for (size_t i = 0; i < n_tracks; i++)
        if (tracks[i].id)
            real_n_tracks++;
    return real_n_tracks;
}
