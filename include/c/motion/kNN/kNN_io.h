/*!
 * \file
 * \brief IOs for k-Nearest Neighbors (kNN) matching algorithm.
 */

#pragma once

#include <stdio.h>
#include <stdint.h>

#include "motion/features/features_struct.h"
#include "motion/kNN/kNN_struct.h"

/**
 * Print a table of RoIs association features plus the corresponding RoIs motion features.
 * @param f File descriptor (in write mode).
 * @param kNN_data Inner kNN data.
 * @param RoIs0 Features at \f$t - 1\f$.
 * @param n_RoIs0 Number of connected-components (= number of RoIs) (at \f$t - 1\f$).
 * @param RoIs1 Features at \f$t\f$.
 * @param n_RoIs1 Number of connected-components (= number of RoIs) (at \f$t\f$)..
 */
void kNN_asso_conflicts_write(FILE* f, const kNN_data_t* kNN_data, const RoI_t* RoIs0, const size_t n_RoIs0,
                              const RoI_t* RoIs1, const size_t n_RoIs1);
