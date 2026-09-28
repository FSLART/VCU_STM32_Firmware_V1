/**
 * @file throttle_map.h
 * @brief 3D throttle map: accelerator x vehicle speed -> relative current
 *
 * The table lives in throttle_map_data.h, written by tools/throttle_map_editor.py.
 * Edit it with the tool, then rebuild and flash. Values between table points are
 * interpolated (bilinear); outside the axes the edge value is used.
 */

#ifndef THROTTLE_MAP_H
#define THROTTLE_MAP_H

#include <stdint.h>

/**
 * @brief Relative current requested by the map
 * @param pedal_1000 Accelerator, 0..1000
 * @param speed_kmh  Vehicle speed (km/h)
 * @return -1000..1000 per mille of the inverter maximum: > 0 drive, < 0 regen, 0 coast
 */
int16_t throttle_map_lookup(uint16_t pedal_1000, float speed_kmh);

#endif  // THROTTLE_MAP_H
