#include "structures.h"

void getCorners(const std::vector<Point>  &pts,
                            uint &IL, uint &IR,
                            uint &SL, uint &SR,
                            const uint &n_colsSRC,
                            const uint &n_rowsSRC);

void initPatron(const std::vector<Point>  &pts,
                std::vector< std::vector<Point> > &patron,
                Line &L1, Line &L2,
                const uint &IL, const uint &IR,
                const uint &SL, const uint &SR,
                uint &n_rows, uint &n_cols);

void addPatron(std::vector<Point>  &pts,
               std::vector< std::vector<Point> > &patron,
               Line &L,
               const uint &n_rows, const uint &n_cols,
               const bool &horz = true);
