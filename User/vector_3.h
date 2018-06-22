/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
  vector_3.cpp - Vector library for bed leveling
  Copyright (c) 2012 Lars Brubaker.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef VECTOR_3_H
#define VECTOR_3_H
#include "XZK_Configuration.h"

typedef struct vector_3 {
  float x, y, z;
}vector_3;

typedef struct matrix_3x3 {
  float matrix[9];
}matrix_3x3;

void vector_3_new(vector_3 *dst);

void vector_3_init(vector_3 *dst, float x_, float y_, float z_);

void vector_3_cross(vector_3 *dst, vector_3 left, vector_3 right);

void vector_3_add(vector_3 *dst, vector_3 a, vector_3 b);

void vector_3_del(vector_3 *dst, vector_3 a, vector_3 b);

float vector_3_get_length(vector_3 a);

void vector_3_normalize(vector_3* dst);

void vector_3_get_normal(vector_3* dst, vector_3 a);

void vector_3_apply_rotation(matrix_3x3 matrix, vector_3* dst);

void vector_3_debug(const char title[]);

void apply_rotation_xyz(matrix_3x3 matrix, float *x, float *y, float *z);

void matrix_3x3_create_from_rows(matrix_3x3 *dst, vector_3 row_0, vector_3 row_1, vector_3 row_2);

void matrix_3x3_set_to_identity( matrix_3x3 *a );

void matrix_3x3_create_look_at(matrix_3x3 *dst, vector_3 target);

void matrix_3x3_transpose(matrix_3x3 *dst, matrix_3x3 original);

void matrix_3x3_debug(const char title[]);



void apply_rotation_xyz(matrix_3x3 rotationMatrix, float *x, float *y, float *z);

#endif // VECTOR_3_H
