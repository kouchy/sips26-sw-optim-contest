#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "motion/kNN/kNN_io.h"

void _kNN_conflicts_write(FILE* f, const uint32_t* kNN_data_conflicts, const float** kNN_data_distances,
                          const uint32_t** kNN_data_nearest, int n_asso, int n_conflicts) {
    // Conflicts
    if (kNN_data_conflicts != NULL) {
        size_t cpt = 0;
        for (int i = 0; i < n_conflicts; i++) {
            if (kNN_data_conflicts[i] > 1)
                cpt++;
        }

        fprintf(f, "#\n");
        if (cpt) {
            fprintf(f, "# Association conflicts [%d]:\n", (int)cpt);
            for (int j = 0; j < n_conflicts; j++) {
                if (kNN_data_conflicts[j] > 1) {
                    fprintf(f, "RoI ID (t) = %d, list of possible RoI IDs (t-1): { ", j + 1);
                    int first = 1;
                    for (int i = 0 ; i < n_asso; i++) {
                        if (kNN_data_nearest[i][j] == 1) {
                            if (!first)
                                fprintf(f, ", ");
                            fprintf(f, "%d [dist = %2.2f]", i + 1, sqrtf(kNN_data_distances[i][j]));
                            first = 0;
                        }
                    }
                    fprintf(f, " }\n");
                }
            }
        } else {
            fprintf(f, "# No conflict found\n");
        }
    }
}

void kNN_asso_conflicts_write(FILE* f, const kNN_data_t* kNN_data, const RoI_t* RoIs0, const size_t n_RoIs0,
                              const RoI_t* RoIs1, const size_t n_RoIs1) {
    // Asso
    int cpt = 0;
    for (size_t i = 0; i < n_RoIs0; i++) {
        if (RoIs0[i].next_id != 0)
            cpt++;
    }
    fprintf(f, "# Associations [%d]:\n", cpt);

    if (cpt) {
        fprintf(f, "# ------------||---------------\n");
        fprintf(f, "#    RoI ID   ||    Distance   \n");
        fprintf(f, "# ------------||---------------\n");
        fprintf(f, "# -----|------||--------|------\n");
        fprintf(f, "#  t-1 |    t || pixels | rank \n");
        fprintf(f, "# -----|------||--------|------\n");
    }

    for (size_t i = 0; i < n_RoIs0; i++) {
        if (RoIs0[i].id == 0)
            continue;
        if (RoIs0[i].next_id) {
            size_t j = (size_t)(RoIs0[i].next_id - 1);
            float dist_ij = sqrtf(kNN_data->distances[i][j]);
            fprintf(f, "  %4u | %4u || %6.3f | %4d \n", RoIs0[i].id, RoIs0[i].next_id, dist_ij,
                                                        kNN_data->nearest[i][j]);
        }
    }

    _kNN_conflicts_write(f, kNN_data->conflicts, (const float**)kNN_data->distances,
                         (const uint32_t**)kNN_data->nearest, n_RoIs0, n_RoIs1);
}
