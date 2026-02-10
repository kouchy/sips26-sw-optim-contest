/*!
 * \file
 * \brief Connected-Component Analysis (CCA) structures (this is also known as *features*).
 * Generally these structures represents characteristics (= *features*) of the Regions of Interest (RoIs).
 */

#pragma once

#include <stdint.h>

/**
 *  Features: bounding box, surface, centroid & associations (matching).
 *  A bounding box represents a rectangular box around the RoI.
 *  The surface is the number of pixels that are in the connected-component (CC).
 *  The centroid is the center of mass of the RoI.
 *  Associations between RoIs. \f$RoI_{t - 1} \leftrightarrow RoI_{t}\f$ and \f$ RoI_{t} \leftrightarrow RoI_{t + 1}\f$.
 *  Generally these associations are computed by a \f$k\f$-Nearest Neighbors (\f$k\f$-NN) matching algorithm.
 */
typedef struct {
    uint32_t id; /**< RoI unique identifiers. A RoI identifier should starts from 1 while 0 should be reserved for
                      uninitialized structure. */
    uint32_t xmin; /**< Minimum \f$x\f$ coordinates of the bounding box. */
    uint32_t xmax; /**< Maximum \f$x\f$ coordinates of the bounding box. */
    uint32_t ymin; /**< Minimum \f$y\f$ coordinates of the bounding box. */
    uint32_t ymax; /**< Maximum \f$y\f$ coordinates of the bounding box. */
    uint32_t S; /**< Numbers of points/pixels = surfaces of the RoIs. */
    float x; /**< \f$x\f$ coordinates of the centroid (\f$ x = S_x / S \f$). */
    float y; /**< \f$y\f$ coordinates of the centroid (\f$ y = S_y / S \f$). */
    uint32_t prev_id; /**< Previous corresponding RoI identifiers (\f$RoI_{t - 1} \leftrightarrow RoI_{t}\f$). */
    uint32_t next_id; /**< Next corresponding RoI identifiers (\f$ RoI_{t} \leftrightarrow RoI_{t + 1}\f$). */
} RoI_t;
