#ifndef MAPPING_H
#define MAPPING_H

#include <string>
#include "structures.h"

/*
 *  Runtimes
 *  --------
 */


/*
 *  Select centers
 *  ----------------
 */
void addingCenters(Pts &pts, Pts &raw);

/*
 *  Matching pattern
 *  ----------------
 */
void getCorners(const Pts &pts,
                      uint &IL, uint &IR,
                      uint &SL, uint &SR,
                      const uint &n_colsSRC,
                      const uint &n_rowsSRC);

void initPatron(const Pts  &points,
                Grid &patron,
                Line &L1, Line &L2,
                const uint &IL, const uint &IR,
                const uint &SL, const uint &SR,
                const uint &len_x, const uint &len_y,
                uint &n_rows, uint &n_cols);

void addPatron(Pts  &points,
               Grid &patron,
               Line &L,
               const uint &n_rows, const uint &n_cols,
               const uint &position,
               const bool &horz = true);

void mapping(Pts  &points,
             Grid &patron,
             const uint &n_rowsImg, const uint &n_colsImg,
             const uint &len_x, const uint &len_y);

#endif