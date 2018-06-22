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
#include <math.h>
#include "vector_3.h"
#ifdef ENABLE_AUTO_BED_LEVELING

void vector_3_new(vector_3 *dst)
{
  dst->x = 0;
  dst->y = 0;
  dst->z = 0;
}

void vector_3_init(vector_3 *dst, float x_, float y_, float z_)
{
  dst->x = x_;
  dst->y = y_;
  dst->z = z_; 
}

/*两个三维向量的向量积运算
dst:用于存放运算后的向量积
left:左向量，其中一个
right:右向量，其中另一个
*/
void vector_3_cross(vector_3 *dst, vector_3 left, vector_3 right) 
{
  dst->x = left.y * right.z - left.z * right.y;
  dst->y = left.z * right.x - left.x * right.z;
  dst->z = left.x * right.y - left.y * right.x;
}

void vector_3_add(vector_3 *dst, vector_3 a, vector_3 b) 
{  
  vector_3_init(dst, (a.x + b.x), (a.y + b.y), (a.z + b.z)); 
}

void vector_3_del(vector_3 *dst, vector_3 a, vector_3 b) 
{ 
  vector_3_init(dst, (a.x - b.x), (a.y - b.y), (a.z - b.z)); 
}

/*获取向量的长度
*/
float vector_3_get_length(vector_3 a) 
{ 
  return sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z)); 
}

/*向量变成单位向量
*/
void vector_3_normalize(vector_3* dst) {
  float length = vector_3_get_length(*dst);
  dst->x /= length;
  dst->y /= length;
  dst->z /= length;
}

/*获取向量的单位向量
dst:用于存放单位向量
a:要获取的向量源
*/
void vector_3_get_normal(vector_3* dst, vector_3 a) {
  vector_3_init(dst, a.x, a.y, a.z);
  vector_3_normalize(dst);
}

/*向量旋转变换
matrix:旋转变换矩阵
dat:用于存放变换后的向量
*/
void vector_3_apply_rotation(matrix_3x3 matrix, vector_3* dst) {
  float resultX = dst->x * matrix.matrix[3 * 0 + 0] + dst->y * matrix.matrix[3 * 1 + 0] + dst->z * matrix.matrix[3 * 2 + 0];
  float resultY = dst->x * matrix.matrix[3 * 0 + 1] + dst->y * matrix.matrix[3 * 1 + 1] + dst->z * matrix.matrix[3 * 2 + 1];
  float resultZ = dst->x * matrix.matrix[3 * 0 + 2] + dst->y * matrix.matrix[3 * 1 + 2] + dst->z * matrix.matrix[3 * 2 + 2];
  dst->x = resultX;
  dst->y = resultY;
  dst->z = resultZ;
}

void vector_3_debug(const char title[]) {
#if 0  
  SERIAL_PROTOCOL(title);
  SERIAL_PROTOCOLPGM(" x: ");
  SERIAL_PROTOCOL_F(x, 6);
  SERIAL_PROTOCOLPGM(" y: ");
  SERIAL_PROTOCOL_F(y, 6);
  SERIAL_PROTOCOLPGM(" z: ");
  SERIAL_PROTOCOL_F(z, 6);
  SERIAL_EOL;
#endif
}

void apply_rotation_xyz(matrix_3x3 matrix, float *x, float *y, float *z) {
  vector_3 vector;
  vector_3_init(&vector,*x, *y, *z);
  vector_3_apply_rotation(matrix, &vector);
  *x = vector.x;
  *y = vector.y;
  *z = vector.z;
}


void matrix_3x3_create_from_rows(matrix_3x3 *dst, vector_3 row_0, vector_3 row_1, vector_3 row_2) {
  //row_0.debug("row_0");
  //row_1.debug("row_1");
  //row_2.debug("row_2");
  dst->matrix[0] = row_0.x; dst->matrix[1] = row_0.y; dst->matrix[2] = row_0.z;
  dst->matrix[3] = row_1.x; dst->matrix[4] = row_1.y; dst->matrix[5] = row_1.z;
  dst->matrix[6] = row_2.x; dst->matrix[7] = row_2.y; dst->matrix[8] = row_2.z;
}

/*设置矩阵为单位矩阵
*/
void matrix_3x3_set_to_identity( matrix_3x3 *a ) {
  a->matrix[0] = 1; a->matrix[1] = 0; a->matrix[2] = 0;
  a->matrix[3] = 0; a->matrix[4] = 1; a->matrix[5] = 0;
  a->matrix[6] = 0; a->matrix[7] = 0; a->matrix[8] = 1;
}

/*创建旋转变换矩阵
dst:用于存放旋转变换矩阵
*/
void matrix_3x3_create_look_at(matrix_3x3 *dst, vector_3 target) {
  vector_3 temp, x_row, y_row, z_row;
  
  vector_3_get_normal(&z_row, target);
  
  vector_3_init(&temp, 1, 0, -target.x / target.z);
  vector_3_get_normal(&x_row, temp);
  
  vector_3_cross(&temp, z_row, x_row);
  vector_3_get_normal(&y_row, temp);

  // x_row.debug("x_row");
  // y_row.debug("y_row");
  // z_row.debug("z_row");

  // create the matrix already correctly transposed
  matrix_3x3_create_from_rows(dst, x_row, y_row, z_row);
}

/*获取矩阵的转置矩阵
dst:用于存放转置矩阵
original:矩阵源
*/
void matrix_3x3_transpose(matrix_3x3 *dst, matrix_3x3 original) {
  dst->matrix[0] = original.matrix[0]; dst->matrix[1] = original.matrix[3]; dst->matrix[2] = original.matrix[6];
  dst->matrix[3] = original.matrix[1]; dst->matrix[4] = original.matrix[4]; dst->matrix[5] = original.matrix[7];
  dst->matrix[6] = original.matrix[2]; dst->matrix[7] = original.matrix[5]; dst->matrix[8] = original.matrix[8];
}

void matrix_3x3_debug(const char title[]) {
#if 0
  SERIAL_PROTOCOLLN(title);
  int count = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (matrix[count] >= 0.0) SERIAL_PROTOCOLCHAR('+');
      SERIAL_PROTOCOL_F(matrix[count], 6);
      SERIAL_PROTOCOLCHAR(' ');
      count++;
    }
    SERIAL_EOL;
  }
#endif
}

#endif // AUTO_BED_LEVELING_FEATURE

