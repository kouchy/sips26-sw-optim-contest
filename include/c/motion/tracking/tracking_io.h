/*!
 * \file
 * \brief IOs for tracks.
 */

#pragma once

#include <stdio.h>

#include "motion/tracking/tracking_struct.h"

/**
 * Print a table of tracks (dedicated to the terminal).
 * @param f File descriptor (in write mode).
 * @param tracks A vector of tracks.
 */
void tracking_tracks_write(FILE* f, const vec_track_t tracks);

/**
 * Print a table of tracks with internal states of the finished state machine.
 * @param f File descriptor (in write mode).
 * @param tracks A vector of tracks.
 */
void tracking_tracks_write_full(FILE* f, const vec_track_t tracks);

/**
 * Print a list of magnitudes per track. Each line corresponds to a track.
 * @param f File descriptor (in write mode).
 * @param tracks A vector of tracks.
 */
void tracking_tracks_RoIs_id_write(FILE* f, const vec_track_t tracks);
