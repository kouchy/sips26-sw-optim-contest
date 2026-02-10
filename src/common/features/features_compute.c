#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "motion/macros.h"
#include "vec.h"

#include "motion/features/features_compute.h"

RoI_t* features_alloc_RoIs(const size_t max_size) {
    RoI_t* RoIs = (RoI_t*)malloc(max_size * sizeof(RoI_t));
    return RoIs;
}

void features_init_RoIs(RoI_t* RoIs, const size_t max_size) {
    memset(RoIs, 0, max_size * sizeof(RoI_t));
}

void features_free_RoIs(RoI_t* RoIs) {
    free(RoIs);
}

void features_extract(const uint32_t** labels, const int i0, const int i1, const int j0, const int j1, RoI_t* RoIs,
                      const size_t n_RoIs) {
    for (size_t i = 0; i < n_RoIs; i++) {
        RoIs[i].xmin = j1;
        RoIs[i].xmax = j0;
        RoIs[i].ymin = i1;
        RoIs[i].ymax = i0;
        RoIs[i].S = 0;
        uint32_t *RoIs_Sx = (uint32_t*)&RoIs[i].x;
        uint32_t *RoIs_Sy = (uint32_t*)&RoIs[i].y;
        *RoIs_Sx = 0;
        *RoIs_Sy = 0;
    }

    uint32_t maxlbl = 0;
    for (int i = i0; i <= i1; i++) {
        for (int j = j0; j <= j1; j++) {
            const uint32_t e = (uint32_t)labels[i][j];
            if (e > 0) {
                maxlbl = MAX(maxlbl, e);
                uint32_t r = e - 1;
                RoIs[r].S += 1;
                RoIs[r].id = e;
                uint32_t *RoIs_Sx = (uint32_t*)&RoIs[r].x;
                uint32_t *RoIs_Sy = (uint32_t*)&RoIs[r].y;
                *RoIs_Sx += j;
                *RoIs_Sy += i;
                if (j < (int)RoIs[r].xmin)
                    RoIs[r].xmin = j;
                if (j > (int)RoIs[r].xmax)
                    RoIs[r].xmax = j;
                if (i < (int)RoIs[r].ymin)
                    RoIs[r].ymin = i;
                if (i > (int)RoIs[r].ymax)
                    RoIs[r].ymax = i;
            }
        }
    }
    assert(maxlbl == n_RoIs);

    for (size_t i = 0; i < n_RoIs; i++) {
        uint32_t *RoIs_Sx = (uint32_t*)&RoIs[i].x;
        uint32_t *RoIs_Sy = (uint32_t*)&RoIs[i].y;
        RoIs[i].x = (float)*RoIs_Sx / (float)RoIs[i].S;
        RoIs[i].y = (float)*RoIs_Sy / (float)RoIs[i].S;
    }
}

uint32_t features_filter_surface(const uint32_t** in_labels, uint32_t** out_labels, const int i0, const int i1,
                                 const int j0, const int j1, RoI_t* RoIs, const size_t n_RoIs, const uint32_t S_min,
                                 const uint32_t S_max) {
    if (out_labels != NULL && (void*)in_labels != (void*)out_labels)
        for (int i = i0; i <= i1; i++)
            memset(out_labels[i], 0, (j1 - j0 + 1) * sizeof(uint32_t));

    uint32_t x0, x1, y0, y1, id;
    uint32_t cur_label = 1;
    for (size_t i = 0; i < n_RoIs; i++) {
        if (RoIs[i].id) {
            id = RoIs[i].id;
            x0 = RoIs[i].ymin;
            x1 = RoIs[i].ymax;
            y0 = RoIs[i].xmin;
            y1 = RoIs[i].xmax;
            if (S_min > RoIs[i].S || RoIs[i].S > S_max) {
                RoIs[i].id = 0;
                if (out_labels != NULL && ((void*)in_labels == (void*)out_labels)) {
                    for (uint32_t k = x0; k <= x1; k++) {
                        for (uint32_t l = y0; l <= y1; l++) {
                            if (in_labels[k][l] == id)
                                out_labels[k][l] = 0;
                        }
                    }
                }
                continue;
            }
            if (out_labels != NULL) {
                for (uint32_t k = x0; k <= x1; k++) {
                    for (uint32_t l = y0; l <= y1; l++) {
                        if (in_labels[k][l] == id) {
                            out_labels[k][l] = cur_label;
                        }
                    }
                }
            }
            cur_label++;
        }
    }

    return cur_label - 1;
}

void features_shrink_basic(const RoI_t* RoIs_src, const size_t n_RoIs_src, RoI_t* RoIs_dst) {
    size_t cpt = 0;
    for (size_t i = 0; i < n_RoIs_src; i++) {
        if (RoIs_src[i].id) {
            RoIs_dst[cpt].id = cpt + 1;
            RoIs_dst[cpt].xmin = RoIs_src[i].xmin;
            RoIs_dst[cpt].xmax = RoIs_src[i].xmax;
            RoIs_dst[cpt].ymin = RoIs_src[i].ymin;
            RoIs_dst[cpt].ymax = RoIs_src[i].ymax;
            RoIs_dst[cpt].S = RoIs_src[i].S;
            RoIs_dst[cpt].x = RoIs_src[i].x;
            RoIs_dst[cpt].y = RoIs_src[i].y;
            cpt++;
        }
    }
}

void features_labels_zero_init(const RoI_t* RoIs, const size_t n_RoIs, uint32_t** labels) {
        for (size_t i = 0; i < n_RoIs; i++) {
        uint32_t y0 = RoIs[i].ymin;
        uint32_t y1 = RoIs[i].ymax;
        uint32_t x0 = RoIs[i].xmin;
        uint32_t x1 = RoIs[i].xmax;
        for (uint32_t k = y0; k <= y1; k++)
            for (uint32_t l = x0; l <= x1; l++)
                labels[k][l] = 0;
    }
}
