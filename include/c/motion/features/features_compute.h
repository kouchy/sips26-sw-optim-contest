/*!
 * \file
 * \brief *Feature* functions. Connected-Component Analysis (CCA) functions.
 */

#pragma once

#include <stddef.h>

#include "motion/features/features_struct.h"

/**
 * Allocation of the basic features.
 * @param max_size Maximum capacity of each *feature* field (= maximum number of elements in the arrays).
 * @return Pointer of allocated RoIs.
 */
RoI_t* features_alloc_RoIs(const size_t max_size);

/**
 * Initialization of the features. Set all zeros.
 * @param RoIs Pointer of RoIs.
 */
void features_init_RoIs(RoI_t* RoIs, const size_t max_size);

/**
 * Free the features.
 * @param RoIs Pointer of RoIs.
 */
void features_free_RoIs(RoI_t* RoIs);

/**
 * Basic features extraction from a 2D array of `labels`.
 * In other words, this function converts a (sparse ?) 2-dimensional representation of connected-components (CCs) into a
 * list of CCs.
 * @param labels Input 2D array of labels (\f$[i1 - i0 + 1][j1 - j0 + 1]\f$).
 * @param i0 First \f$y\f$ index in the labels (included).
 * @param i1 Last \f$y\f$ index in the labels (included).
 * @param j0 First \f$x\f$ index in the labels (included).
 * @param j1 Last \f$x\f$ index in the labels (included).
 * @param RoIs Features.
 * @param n_RoIs Number of connected-components (= number of RoIs) in the 2D array of `labels`.
 * @see RoI_t for more explanations about the features.
 */
void features_extract(const uint32_t** labels, const int i0, const int i1, const int j0, const int j1,
                      RoI_t* RoIs, const size_t n_RoIs);

/**
 * This function performs a surface thresholding as follow: if \f$ S_{min} > S \f$ or \f$ S > S_{max}\f$, then the
 * corresponding `RoIs_id` is set to 0.
 * @param in_labels Input 2D array of labels (\f$[i1 - i0 + 1][j1 - j0 + 1]\f$).
 * @param out_labels Output 2D array of labels (\f$[i1 - i0 + 1][j1 - j0 + 1]\f$). \p out_labels can be NULL, this way
 *                   only the features will be updated. \p out_labels can also be the same pointer as \p in_labels, this
 *                   way the output labels will be computed in place.
 * @param i0 First \f$y\f$ index in the labels (included).
 * @param i1 Last \f$y\f$ index in the labels (included).
 * @param j0 First \f$x\f$ index in the labels (included).
 * @param j1 Last \f$x\f$ index in the labels (included).
 * @param RoIs Features.
 * @param n_RoIs Number of RoIs in the previous arrays.
 * @param S_min Minimum morphological threshold.
 * @param S_max Maximum morphological threshold.
 * @return Number of labels after filtering.
 * @see RoI_t for more explanations about the features.
 */
uint32_t features_filter_surface(const uint32_t** in_labels, uint32_t** out_labels, const int i0, const int i1,
                                 const int j0, const int j1, RoI_t* RoIs, const size_t n_RoIs, const uint32_t S_min,
                                 const uint32_t S_max);
/**
 * Shrink features. Remove features when feature identifier value is 0.
 * Source features (`RoIs_src[i].X`) are copied into destination features (`RoIs_dst[i].X`) if `RoIs_src[i].id` > 0.
 * @param RoIs_src Source features.
 * @param n_RoIs_src Number of RoIs in the previous arrays.
 * @param RoIs_dst Destination features.
 * @see RoIs_basic_t for more explanations about the features.
 */
void features_shrink_basic(const RoI_t* RoIs_src, const size_t n_RoIs_src, RoI_t* RoIs_dst);

/**
 * Initialize labels to zero value depending on bounding boxes.
 * @param RoIs Features (contains the bounding boxes).
 * @param n_RoIs Number of connected-components (= number of RoIs).
 * @param labels 2D array of labels (\f$[\texttt{img\_height}][\texttt{img\_width}]\f$).
 */
void features_labels_zero_init(const RoI_t* RoIs, const size_t n_RoIs, uint32_t** labels);
