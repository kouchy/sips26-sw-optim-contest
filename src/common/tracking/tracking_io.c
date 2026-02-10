#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vec.h"
#include "motion/tracking/tracking_io.h"

void tracking_tracks_write(FILE* f, const vec_track_t tracks) {
    size_t real_n_tracks = 0;
    size_t n_tracks = vector_size(tracks);
    for (size_t i = 0; i < n_tracks; i++)
        if (tracks[i].id)
            real_n_tracks++;

    fprintf(f, "# Tracks [%lu]:\n", (unsigned long)real_n_tracks);
    fprintf(f, "# -------||---------------------------||---------------------------\n");
    fprintf(f, "#  Track ||           Begin           ||            End            \n");
    fprintf(f, "# -------||---------------------------||---------------------------\n");
    fprintf(f, "# -------||---------|--------|--------||---------|--------|--------\n");
    fprintf(f, "#     Id || Frame # |      x |      y || Frame # |      x |      y \n");
    fprintf(f, "# -------||---------|--------|--------||---------|--------|--------\n");

    for (size_t i = 0; i < n_tracks; i++)
        if (tracks[i].id) {
            fprintf(f, "   %5d || %7u | %6.1f | %6.1f || %7u | %6.1f | %6.1f \n", tracks[i].id, tracks[i].begin.frame,
                    tracks[i].begin.r.x, tracks[i].begin.r.y, tracks[i].end.frame, tracks[i].end.r.x,
                    tracks[i].end.r.y);
        }
}

void tracking_tracks_write_full(FILE* f, const vec_track_t tracks) {
    size_t real_n_tracks = 0;
    size_t n_tracks = vector_size(tracks);
    for (size_t i = 0; i < n_tracks; i++)
        if (tracks[i].id)
            real_n_tracks++;

    fprintf(f, "# Tracks [%lu]:\n", (unsigned long)real_n_tracks);
    fprintf(f, "# -------||---------------------------||---------------------------||-------\n");
    fprintf(f, "#  Track ||           Begin           ||            End            || State \n");
    fprintf(f, "# -------||---------------------------||---------------------------||-------\n");
    fprintf(f, "# -------||---------|--------|--------||---------|--------|--------||-------\n");
    fprintf(f, "#     Id || Frame # |      x |      y || Frame # |      x |      y ||       \n");
    fprintf(f, "# -------||---------|--------|--------||---------|--------|--------||-------\n");

    for (size_t i = 0; i < n_tracks; i++)
        if (tracks[i].id) {
            char str_state[16];
            switch(tracks[i].state) {
                case STATE_UNKNOWN:
                    snprintf(str_state, sizeof(str_state), "  UKN");
                    break;
                case STATE_UPDATED:
                    snprintf(str_state, sizeof(str_state), "  UPD");
                    break;
                case STATE_LOST:
                    snprintf(str_state, sizeof(str_state), "  LST");
                    break;
                case STATE_FINISHED:
                    snprintf(str_state, sizeof(str_state), "  FNS");
                    break;
                default:
                    snprintf(str_state, sizeof(str_state), "  ???");
                    break;
            }
            fprintf(f, "   %5d || %7u | %6.1f | %6.1f || %7u | %6.1f | %6.1f || %s \n", tracks[i].id, tracks[i].begin.frame,
                    tracks[i].begin.r.x, tracks[i].begin.r.y, tracks[i].end.frame, tracks[i].end.r.x,
                    tracks[i].end.r.y, str_state);
        }
}

void tracking_tracks_RoIs_id_write(FILE* f, const vec_track_t tracks) {
    size_t n_tracks = vector_size(tracks);
    for (size_t i = 0; i < n_tracks; i++)
        if (tracks[i].id) {
            fprintf(f, " %5d %s ", tracks[i].id, "object");
            if (tracks[i].RoIs_id != NULL) {
                size_t vs = vector_size(tracks[i].RoIs_id);
                for (size_t j = 0; j < vs; j++)
                    fprintf(f, " %5u ", tracks[i].RoIs_id[j]);
                fprintf(f, "\n");
            }
        }
}
