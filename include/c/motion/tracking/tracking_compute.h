/*!
 * \file
 * \brief Functions to compute the tracks.
 */

#pragma once

#include "motion/features/features_struct.h"
#include "motion/tracking/tracking_struct.h"

/**
 * Allocation of inner data required to perform the tracking.
 * @param max_history_size The maximum size of the history window (number of frames memorized in the history of RoIs).
 * @param max_RoIs_size The maximum number of RoIs per frame.
 * @return The allocated data.
 */
tracking_data_t* tracking_alloc_data(const size_t max_history_size, const size_t max_RoIs_size);

/**
 * Zero initialization of inner data required to perform the tracking.
 * @param tracking_data Pointer of tracking inner data.
 */
void tracking_init_data(tracking_data_t* tracking_data);

/**
 * Free the tracking inner data.
 * @param tracking_data Pointer of tracking inner data.
 */
void tracking_free_data(tracking_data_t* tracking_data);

/**
 * Create, update and finalize tracks. This function also performs the classification of the tracks.
 * @param tracking_data Inner data.
 * @param RoIs Features (at \f$t\f$).
 * @param n_RoIs Number of connected-components (= number of RoIs) (at \f$t\f$).
 * @param frame Current frame number.
 * @param r_extrapol Accepted range for extrapolation.
 * @param fra_obj_min Minimum number of CC/RoI associations before creating a obj track.
 * @param save_RoIs_id Boolean to save the list of the RoI ids for each tracks.
 * @param extrapol_order_max Maximum number of frames where a lost track is extrapolated (0 means no extrapolation).
 * @param min_extrapol_ratio_S Minimum ratio between two RoIs. \f$ r_S = RoI_{S}^j / RoI_{S}^i\f$, if
 *                             \f$r_S < r_S^{min}\f$ then the association for the extrapolation is not made.
 */
void tracking_perform(tracking_data_t* tracking_data, const RoI_t* RoIs, const size_t n_RoIs, size_t frame,
                      const size_t r_extrapol, const size_t fra_obj_min, const uint8_t save_RoIs_id,
                      const uint8_t extrapol_order_max, const float min_extrapol_ratio_S);
