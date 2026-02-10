/*!
 * \file
 * \brief Tracking structures.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "motion/features/features_struct.h"

/**
 *  Enumeration of the states in the tracking finite-state machine.
 */
enum state_e { STATE_UNKNOWN = 0, /*!< Unknown (= uninitialized). */
               STATE_UPDATED, /*!< Track has been updated (or created). */
               STATE_LOST, /*!< Track has not been updated, it is lost. */
               STATE_FINISHED, /*!< Track is finished. */
               N_STATES /*!< Number of states in the enumeration. */
};

/**
 *  Vector of `uint32_t`, to use with C vector lib.
 */
typedef uint32_t* vec_uint32_t;

typedef struct {
    RoI_t r;
    uint32_t frame;
    uint32_t time_motion;
    uint8_t is_extrapolated;
} RoI4track_t;

/**
 *  Description of a track.
 */
typedef struct {
    uint32_t id; /**< Track unique identifiers. A track identifier should starts from 1 while 0 should be reserved for
                      uninitialized structure. */
    RoI4track_t begin; /**< First RoI corresponding to this track. */
    RoI4track_t end; /**< Last RoI corresponding to this track. */
    uint32_t frame_begin; /**< Frame number of the beginning of the track. */
    uint32_t frame_end; /**< Frame number of the end of the track. */
    float extrapol_x1; /**< Last \f$x\f$ position of the extrapolated track. */
    float extrapol_y1; /**< Last \f$y\f$ position of the extrapolated track. */
    float extrapol_x2; /**< Before last \f$x\f$ position of the extrapolated track. */
    float extrapol_y2; /**< Before last \f$y\f$ position of the extrapolated track. */
    float extrapol_dx; /**< Velocity \f$x\f$ estimation of the track for extrapolation between `extrapol_x1` and
                            `extrapol_x2`. */
    float extrapol_dy; /**< Velocity \f$y\f$ estimation of the track for extrapolation between `extrapol_y1` and
                            `extrapol_y2`. */
    uint8_t extrapol_order; /**< Number of times this track has been extrapolated (used only if `state` ==
                                 `STATE_LOST`). */
    enum state_e state; /**< State of the track. */
    vec_uint32_t RoIs_id; /**< Vector of the RoI ids history of this track. */
} track_t;

/**
 *  Vector of `track_t`, to use with C vector lib.
 */
typedef track_t* vec_track_t;

/**
 *  History of the previous RoI features and motions.
 *  This structure allows to access RoI/motion in the past frames.
 *  RoIs at \f$t\f$ are stored in the first array element while RoIs at \f$t-\texttt{\_size}\f$ are store in the
 *  \f$\texttt{\_size} - 1\f$ element.
 *  The memory layout is a Structure of Arrays (SoA), each field is an array of `_max_size` capacity (except for
 * `_max_size` itself and `_size` fields that are both scalar values).
 */
typedef struct {
    RoI4track_t** RoIs; /**< 2D array of RoIs, the first dimension is the time and the second dimension is the RoIs at a
                             given time. */
    uint32_t* n_RoIs; /**< Array of numbers of RoIs. */
    uint32_t _max_n_RoIs; /**< Maximum number of RoIs. */
    size_t _size; /**< Current size/utilization of the fields. */
    size_t _max_size; /**< Maximum capacity of data that can be contained in the fields. */
} History_t;

/**
 *  Inner data used by the tracking.
 */
typedef struct {
    vec_track_t tracks; /**< Vector of tracks. */
    History_t* history; /**< RoIs and motions history. */
    RoI4track_t* RoIs_list; /**< List of RoIs. This is a temporary array used to group all the RoIs belonging to a same
                                 track. */
} tracking_data_t;

/**
 * Compute the duration of a track.
 * @param track_begin First RoI of the track.
 * @param track_end Last RoI of the track.
 * @return The elapsed time (in number of frames).
 */
size_t _tracking_get_track_time(const RoI4track_t track_begin, const RoI4track_t track_end);

/**
 * Compute the duration of a track.
 * @param tracks A vector of tracks.
 * @param t The position of one track in the tracks array.
 * @return The elapsed time (in number of frames).
 */
size_t tracking_get_track_time(const vec_track_t tracks, const size_t t);

/**
 * Counts the number of tracks in a vector of tracks.
 * @param tracks A vector of tracks.
 * @return The real number of tracks (may be less than the \p tracks vector size).
 */
size_t tracking_count_objects(const vec_track_t tracks);
