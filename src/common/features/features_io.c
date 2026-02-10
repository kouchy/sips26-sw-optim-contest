#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "vec.h"

#include "motion/tracking/tracking_struct.h"
#include "motion/features/features_io.h"

int find_corresponding_track(const int frame, const vec_track_t tracks, const RoI_t* RoIs, const int sel_RoIs_id,
                             const size_t n_RoIs, const unsigned age) {
    assert(age == 0 || age == 1);

    size_t n_tracks = vector_size(tracks);
    for (size_t t = 0; t < n_tracks; t++) {
        if (tracks[t].id) {
            if (tracks[t].end.frame == frame + age) {
                int cur_RoIs_id;
                if (age == 0)
                    cur_RoIs_id = tracks[t].end.r.id;
                else {
                    if (tracks[t].end.r.prev_id == 0)
                        continue;
                    cur_RoIs_id = RoIs[tracks[t].end.r.prev_id - 1].id;
                }
                assert(cur_RoIs_id <= (int)n_RoIs);
                if (cur_RoIs_id <= 0)
                    continue;
                if (sel_RoIs_id == cur_RoIs_id)
                    return t;
            }
        }
    }
    return -1;
}

void features_RoIs_write(FILE* f, const int frame, const RoI_t* RoIs, const size_t n_RoIs, const vec_track_t tracks,
                         const unsigned age) {
    int cpt = 0;
    for (size_t i = 0; i < n_RoIs; i++)
        if (RoIs[i].id != 0)
            cpt++;

    fprintf(f, "Regions of interest (RoI) [%d]: \n", cpt);
    // if (cpt) {  // for compare.py
    if (tracks) {
        fprintf(f, "# ------||-------||---------------------------||---------||-------------------\n");
        fprintf(f, "#   RoI || Track ||        Bounding Box       || Surface ||      Center       \n");
        fprintf(f, "# ------||-------||---------------------------||---------||-------------------\n");
        fprintf(f, "# ------||-------||------|------|------|------||---------||---------|---------\n");
        fprintf(f, "#    ID ||    ID || xmin | xmax | ymin | ymax ||       S ||       x |       y \n");
        fprintf(f, "# ------||-------||------|------|------|------||---------||---------|---------\n");
    } else {
        fprintf(f, "# ------||---------------------------||---------||-------------------\n");
        fprintf(f, "#   RoI ||        Bounding Box       || Surface ||      Center       \n");
        fprintf(f, "# ------||---------------------------||---------||-------------------\n");
        fprintf(f, "# ------||------|------|------|------||---------||---------|---------\n");
        fprintf(f, "#    ID || xmin | xmax | ymin | ymax ||       S ||       x |       y \n");
        fprintf(f, "# ------||------|------|------|------||---------||---------|---------\n");
    }
    // }

    for (size_t i = 0; i < n_RoIs; i++) {
        if (RoIs[i].id != 0) {
            int t = tracks ? find_corresponding_track(frame, tracks, RoIs, RoIs[i].id, n_RoIs, age) : -1;
            char track_id_str[16];
            if (t == -1)
                strcpy(track_id_str, "    -");
            else
                snprintf(track_id_str, sizeof(track_id_str), "%5u", tracks[t].id);

            if (tracks) {
                fprintf(f, "   %4u || %s || %4u | %4u | %4u | %4u || %7u || %7.1f | %7.1f \n",
                        RoIs[i].id, track_id_str, RoIs[i].xmin, RoIs[i].xmax, RoIs[i].ymin, RoIs[i].ymax, RoIs[i].S,
                        RoIs[i].x, RoIs[i].y);
            } else {
                fprintf(f, "   %4u || %4u | %4u | %4u | %4u || %7u || %7.1f | %7.1f \n",
                        RoIs[i].id, RoIs[i].xmin, RoIs[i].xmax, RoIs[i].ymin, RoIs[i].ymax, RoIs[i].S, RoIs[i].x,
                        RoIs[i].y);
            }
        }
    }
}

void features_RoIs0_RoIs1_write(FILE* f, const int prev_frame, const int cur_frame, const RoI_t* RoIs0,
                                const size_t n_RoIs0, const RoI_t* RoIs1, const size_t n_RoIs1,
                                const vec_track_t tracks) {
    if (prev_frame >= 0) {
        fprintf(f, "# Frame n°%05d (t-1) -- ", prev_frame);
        features_RoIs_write(f, prev_frame, RoIs0, n_RoIs0, tracks, 1);
        fprintf(f, "#\n");
    }

    fprintf(f, "# Frame n°%05d (t) -- ", cur_frame);
    features_RoIs_write(f, cur_frame, RoIs1, n_RoIs1, tracks, 0);
}
