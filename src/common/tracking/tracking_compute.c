#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "motion/tools.h"
#include "motion/macros.h"
#include "vec.h"

#include "motion/tracking/tracking_compute.h"

History_t* alloc_history(const size_t max_history_size, const size_t max_RoIs_size) {
    History_t* history = (History_t*)malloc(sizeof(History_t));
    history->_max_size = max_history_size;
    history->RoIs = (RoI4track_t**)malloc(history->_max_size * sizeof(RoI4track_t*));
    history->n_RoIs = (uint32_t*)malloc(history->_max_size * sizeof(uint32_t));
    history->_max_n_RoIs = max_RoIs_size;
    history->_size = 0;
    for (size_t i = 0; i < history->_max_size; i++) {
        history->RoIs[i] = (RoI4track_t*)malloc(max_RoIs_size * sizeof(RoI4track_t));
        history->n_RoIs[i] = 0;
        for (size_t j = 0; j < max_RoIs_size; j++) {
            memset(&history->RoIs[i][j], 0, sizeof(RoI4track_t));
            history->RoIs[i][j].r.x = NAN;
            history->RoIs[i][j].r.y = NAN;
        }
    }
    return history;
}

void free_history(History_t* history) {
    for (size_t i = 0; i < history->_max_size; i++)
        free(history->RoIs[i]);
    free(history->RoIs);
    free(history->n_RoIs);
    free(history);
}

void rotate_history(History_t* history) {
    RoI4track_t* last_RoIs_tmp = history->RoIs[history->_max_size -1];
    uint32_t last_n_RoIs_tmp = history->n_RoIs[history->_max_size -1];
    for (int i = (int)(history->_max_size -2); i >= 0; i--) {
        history->RoIs[i + 1] = history->RoIs[i];
        history->n_RoIs[i + 1] = history->n_RoIs[i];
    }
    history->RoIs[0] = last_RoIs_tmp;
    history->n_RoIs[0] = last_n_RoIs_tmp;
}

tracking_data_t* tracking_alloc_data(const size_t max_history_size, const size_t max_RoIs_size) {
    tracking_data_t* tracking_data = (tracking_data_t*)malloc(sizeof(tracking_data_t));
    tracking_data->tracks = (vec_track_t)vector_create();
    tracking_data->history = alloc_history(max_history_size, max_RoIs_size);
    tracking_data->RoIs_list = (RoI4track_t*)malloc(max_history_size * sizeof(RoI4track_t));
    return tracking_data;
}

void tracking_init_data(tracking_data_t* tracking_data) {
    memset(tracking_data->RoIs_list, 0, tracking_data->history->_max_size * sizeof(RoI4track_t));
    for (size_t i = 0; i < tracking_data->history->_max_size; i++) {
        memset(tracking_data->history->RoIs[i], 0, tracking_data->history->_max_n_RoIs *
               sizeof(RoI4track_t));
        tracking_data->history->n_RoIs[i] = 0;
    }
    tracking_data->history->_size = 0;
}

void tracking_free_data(tracking_data_t* tracking_data) {
    int vs = vector_size(tracking_data->tracks);
    for (int t = 0; t < vs; t++)
        if (tracking_data->tracks[t].RoIs_id != NULL)
            vector_free(tracking_data->tracks[t].RoIs_id);
    vector_free(tracking_data->tracks);
    free_history(tracking_data->history);
    free(tracking_data->RoIs_list);
    free(tracking_data);
}

// Returns 0 if no RoI matches or returns the RoI id found (RoI id >= 1)
size_t _find_matching_RoI(const History_t* history, const track_t* cur_track, const size_t r_extrapol,
                          const float min_extrapol_ratio_S) {
    for (size_t j = 0; j < history->n_RoIs[0]; j++) {
        if (!history->RoIs[0][j].r.prev_id && !history->RoIs[0][j].is_extrapolated) {
            float x0_0 = history->RoIs[0][j].r.x;
            float y0_0 = history->RoIs[0][j].r.y;

            // motion compensation from t - 1 to t
            float x1_0 = cur_track->extrapol_x1;
            float y1_0 = cur_track->extrapol_y1;

            float x_diff = x0_0 - (x1_0 + cur_track->extrapol_dx);
            float y_diff = y0_0 - (y1_0 + cur_track->extrapol_dy);
            float dist = sqrtf(x_diff * x_diff + y_diff * y_diff);

            float ratio_S_ij = cur_track->end.r.S < history->RoIs[0][j].r.S ?
                               (float)cur_track->end.r.S / (float)history->RoIs[0][j].r.S :
                               (float)history->RoIs[0][j].r.S / (float)cur_track->end.r.S;

            if (dist < r_extrapol && ratio_S_ij >= min_extrapol_ratio_S) {
                // in the current implementation, the first RoI that matches is used for extrapolation
                // TODO: this behavior is dangerous, we should associate the closest RoI
                return j + 1;
            }
        }
    }
    return 0;
}

void _track_extrapolate(const History_t* history, track_t* cur_track) {
    float x1_1 = cur_track->extrapol_x1;
    float y1_1 = cur_track->extrapol_y1;

    // motion compensation from t - 1 to t
    float x1_0 = x1_1;
    float y1_0 = y1_1;

    cur_track->extrapol_x2 = x1_1;
    cur_track->extrapol_y2 = y1_1;

    // extrapolate x0 and y0 @ t
    cur_track->extrapol_x1 = x1_0 + cur_track->extrapol_dx;
    cur_track->extrapol_y1 = y1_0 + cur_track->extrapol_dy;
}

void _update_extrapol_vars(const History_t* history, track_t* cur_track) {
    float x2_1 = cur_track->extrapol_x1;
    float y2_1 = cur_track->extrapol_y1;

    // motion compensation from t - 1 to t
    float x2_0 = x2_1;
    float y2_0 = y2_1;

    float x1_0 = cur_track->end.r.x;
    float y1_0 = cur_track->end.r.y;

    cur_track->extrapol_dx = x1_0 - x2_0;
    cur_track->extrapol_dy = y1_0 - y2_0;

    // for tracking @ t + 1
    cur_track->extrapol_x2 = cur_track->extrapol_x1;
    cur_track->extrapol_y2 = cur_track->extrapol_y1;
    cur_track->extrapol_x1 = cur_track->end.r.x;
    cur_track->extrapol_y1 = cur_track->end.r.y;
}

void _update_existing_tracks(History_t* history, vec_track_t track_array, const size_t frame, const size_t r_extrapol,
                             const uint8_t extrapol_order_max, const float min_extrapol_ratio_S) {
    size_t n_tracks = vector_size(track_array);
    for (size_t i = 0; i < n_tracks; i++) {
        track_t* cur_track = &track_array[i];
        if (cur_track->id && cur_track->state != STATE_FINISHED) {
            if (cur_track->state == STATE_LOST) {
                size_t RoI_id = _find_matching_RoI(history, cur_track, r_extrapol, min_extrapol_ratio_S);
                if (RoI_id) {
                    cur_track->state = STATE_UPDATED;
                    history->RoIs[0][RoI_id - 1].is_extrapolated = 1;
                    memcpy(&cur_track->end, &history->RoIs[0][RoI_id - 1], sizeof(RoI4track_t));
                    _update_extrapol_vars(history, cur_track);

                    if (cur_track->RoIs_id != NULL) {
                        // no RoI id when the RoI has been extrapolated
                        for (uint8_t e = cur_track->extrapol_order; e >= 1; e--)
                            vector_add(&cur_track->RoIs_id, (uint32_t)0);
                        vector_add(&cur_track->RoIs_id, history->RoIs[0][RoI_id - 1].r.id);
                    }
                    cur_track->extrapol_order = 0;
                }
            }
            else if (cur_track->state == STATE_UPDATED) {
                int next_id = history->RoIs[1][cur_track->end.r.id - 1].r.next_id;
                if (next_id) {
                    memcpy(&cur_track->end, &history->RoIs[0][next_id - 1], sizeof(RoI4track_t));
                    _update_extrapol_vars(history, cur_track);
                    if (cur_track->RoIs_id != NULL)
                        vector_add(&cur_track->RoIs_id, history->RoIs[0][next_id - 1].r.id);
                } else {
                    size_t RoI_id = _find_matching_RoI(history, cur_track, r_extrapol, min_extrapol_ratio_S);
                    if (RoI_id) {
                        history->RoIs[0][RoI_id - 1].is_extrapolated = 1;
                        memcpy(&cur_track->end, &history->RoIs[0][RoI_id - 1], sizeof(RoI4track_t));
                        _update_extrapol_vars(history, cur_track);

                        if (cur_track->RoIs_id != NULL)
                            vector_add(&cur_track->RoIs_id, history->RoIs[0][RoI_id - 1].r.id);
                    } else {
                        cur_track->state = STATE_LOST;
                    }
                }
            }
            if (cur_track->state == STATE_LOST) {
                cur_track->extrapol_order++;
                if (cur_track->extrapol_order > extrapol_order_max) {
                    cur_track->state = STATE_FINISHED;
                } else {
                    // extrapolate if the state is not finished
                    _track_extrapolate(history, cur_track);
                }
            }
        }
    }
}

void _insert_new_track(const RoI4track_t* RoIs_list, const unsigned n_RoIs, vec_track_t* track_array, const int frame,
                       const uint8_t save_RoIs_id) {
    assert(n_RoIs >= 1);

    size_t track_id = vector_size(*track_array) + 1;
    track_t* tmp_track = vector_add_asg(track_array);
    tmp_track->id = track_id;
    memcpy(&tmp_track->begin, &RoIs_list[n_RoIs - 1], sizeof(RoI4track_t));
    memcpy(&tmp_track->end, &RoIs_list[0], sizeof(RoI4track_t));
    tmp_track->state = STATE_UPDATED;
    tmp_track->RoIs_id = NULL;
    tmp_track->extrapol_x2 = RoIs_list[1].r.x;
    tmp_track->extrapol_y2 = RoIs_list[1].r.y;
    tmp_track->extrapol_x1 = RoIs_list[0].r.x;
    tmp_track->extrapol_y1 = RoIs_list[0].r.y;
    tmp_track->extrapol_dx = NAN; // this will be properly initialized later in "_update_existing_tracks"
    tmp_track->extrapol_dy = NAN; // this will be properly initialized later in "_update_existing_tracks"
    tmp_track->extrapol_order = 0;
    if (save_RoIs_id) {
        tmp_track->RoIs_id = (vec_uint32_t)vector_create();
        for (unsigned n = 0; n < n_RoIs; n++)
            vector_add(&tmp_track->RoIs_id, RoIs_list[(n_RoIs - 1) - n].r.id);
    }
    tmp_track = NULL; // stop using temp now that the element is initialized
}

void _create_new_tracks(History_t* history, RoI4track_t* RoIs_list, vec_track_t* track_array, const size_t frame,
                        const size_t fra_obj_min, const uint8_t save_RoIs_id) {
    for (size_t i = 0; i < history->n_RoIs[1]; i++) {
        int asso = history->RoIs[1][i].r.next_id;
        if (asso) {
            if (history->RoIs[1][i].is_extrapolated)
                continue; // Extrapolated
            int time = history->RoIs[1][i].time_motion + 1;
            history->RoIs[0][asso - 1].time_motion = time;
            int fra_min = fra_obj_min;
            if (time == fra_min - 1) {
                // this loop prevent adding duplicated tracks
                size_t n_tracks = vector_size(*track_array);
                size_t j = 0;
                while (j < n_tracks && ((*track_array)[j].end.r.id != history->RoIs[1][i].r.id ||
                       (*track_array)[j].end.r.x != history->RoIs[1][i].r.x ||
                       (*track_array)[j].end.r.y != history->RoIs[1][i].r.y))
                    j++;

                if (j == n_tracks || n_tracks == 0) {
                    memcpy(&RoIs_list[0], &history->RoIs[1][i], sizeof(RoI4track_t));

                    const size_t n_RoIs = fra_min - 1;
                    for (size_t ii = 1; ii < n_RoIs; ii++)
                        memcpy(&RoIs_list[ii], &history->RoIs[ii + 1][RoIs_list[ii - 1].r.prev_id - 1],
                               sizeof(RoI4track_t));

                     _insert_new_track(RoIs_list, fra_min - 1, track_array, frame, save_RoIs_id);
                }
            }
        }
    }
}

void _light_copy_RoIs(const RoI_t* RoIs_src, const size_t n_RoIs_src, RoI4track_t* RoIs_dst, const uint32_t frame) {
    for (size_t i = 0; i < n_RoIs_src; i++) {
        RoIs_dst[i].r = RoIs_src[i];
        RoIs_dst[i].r.next_id = 0;
        RoIs_dst[i].frame = frame;
        RoIs_dst[i].time_motion = 0;
        RoIs_dst[i].is_extrapolated = 0;
    }
}

void _update_RoIs_next_id(const RoI_t* RoIs, RoI4track_t* RoIs_dst, const size_t n_RoIs) {
    for (size_t i = 0; i < n_RoIs; i++)
        if (RoIs[i].prev_id)
            RoIs_dst[RoIs[i].prev_id - 1].r.next_id = i + 1;
}

void tracking_perform(tracking_data_t* tracking_data, const RoI_t* RoIs, const size_t n_RoIs, const size_t frame,
                      const size_t r_extrapol, const size_t fra_obj_min, const uint8_t save_RoIs_id,
                      const uint8_t extrapol_order_max, const float min_extrapol_ratio_S) {
    assert(extrapol_order_max < tracking_data->history->_max_size);
    assert(min_extrapol_ratio_S >= 0.f && min_extrapol_ratio_S <= 1.f);

    tracking_data->history->n_RoIs[0] = n_RoIs;
    _light_copy_RoIs(RoIs, n_RoIs, tracking_data->history->RoIs[0], frame);

    if (tracking_data->history->_size > 0)
        _update_RoIs_next_id(RoIs, tracking_data->history->RoIs[1], n_RoIs);
    if (tracking_data->history->_size < tracking_data->history->_max_size)
        tracking_data->history->_size++;

    if (tracking_data->history->_size >= 2) {
        _create_new_tracks(tracking_data->history, tracking_data->RoIs_list, &tracking_data->tracks, frame,
                           fra_obj_min, save_RoIs_id);
        _update_existing_tracks(tracking_data->history, tracking_data->tracks, frame, r_extrapol,
                                extrapol_order_max, min_extrapol_ratio_S);
    }

    rotate_history(tracking_data->history);
    memset(tracking_data->history->RoIs[0], 0, tracking_data->history->n_RoIs[0] * sizeof(RoI4track_t));
    tracking_data->history->n_RoIs[0] = 0;
}
