#ifndef MAPPING_H
#define MAPPING_H

#include "structures.h"
typedef std::vector<Point> vPoint;

T umb = 10.0;


/*
 *  Runtimes
 *  --------
 */


/*
 *  Select centers
 *  ----------------
 */
void addingCenters(vPoint &pts,
                   vPoint &raw);


/*
 *  Matching pattern
 *  ----------------
 */
void getCorners(const vPoint &pts,
                      uint &IL, uint &IR,
                      uint &SL, uint &SR,
                      const uint &n_colsSRC,
                      const uint &n_rowsSRC);

void initPatron(const std::vector<Point>  &pts,
                std::vector< std::vector<Point> > &patron,
                Line &L1, Line &L2,
                const uint &IL, const uint &IR,
                const uint &SL, const uint &SR,
                const uint &len_x, const uint &len_y,
                uint &n_rows, uint &n_cols);

void addPatron(std::vector<Point>  &pts,
               std::vector< std::vector<Point> > &patron,
               Line &L,
               const uint &n_rows, const uint &n_cols,
               const uint &position,
               const bool &horz = true);

void mapping(std::vector<Point>  &pts,
             std::vector< std::vector<Point> > &patron,
             const uint &n_rowsImg, const uint &n_colsImg,
             const uint &len_x, const uint &len_y);

#endif