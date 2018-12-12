#include "mapping.h"

T umb = 10.0;

//  ===============================================================
//  Runtimes
//  ===============================================================



//  ===============================================================
//  Select centers
//  ===============================================================




//  ===============================================================
//  Matching pattern
//  ===============================================================

/*
 *  Sort functions
 *  --------------
 */
bool sortDistance(const Pt &a, const Pt &b){
    return ( a.d < b.d );
}
bool sortX(const Pt &a, const Pt &b){
    return ( a.x < b.x );
}
bool sortY(const Pt &a, const Pt &b){
    return ( a.y < b.y );
}

/*
 *  Get corners
 *  -----------
 */
void getCorners(const Pts &points,
                    uint &IL, uint &IR,
                    uint &SL, uint &SR,
                    const uint &n_colsSRC,
                    const uint &n_rowsSRC){
    T x_cols,y_rows,aux;
    T x,y;
    T MAX_T = std::numeric_limits<T>::max();
    T IL_min = MAX_T, SL_min = MAX_T, 
      IR_min = MAX_T, SR_min = MAX_T;
    for( uint i = 0; i < points.size(); i++ ){
        x = points[i].x; x_cols = n_colsSRC - x;
        y = points[i].y; y_rows = n_rowsSRC - y;

        // Inferior Left
        aux = x*x + y*y;
        if (IL_min>aux){ IL_min = aux;
                         IL     =   i;}

        // Superior Left
        aux = x*x + y_rows*y_rows;
        if (SL_min>aux){ SL_min = aux;
                         SL     =   i;}

        // Inferior Right
        aux = x_cols*x_cols + y*y;
        if (IR_min>aux){ IR_min = aux;
                         IR     =   i;}

        // Superior Right
        aux = x_cols*x_cols + y_rows*y_rows;
        if (SR_min>aux){ SR_min = aux;
                         SR     =   i;}
    }
}



void initPatron(const Pts  &points,
                Grid &patron,
                Line &L1, Line &L2,
                const uint &IL, const uint &IR,
                const uint &SL, const uint &SR,
                const uint &len_x, const uint &len_y,
                uint &n_rows, uint &n_cols){

/*
 *  Definir orientacion
 *  -------------------
 */ 
    Pts ptsL1(points); 
    Pts ptsL2(points); 
    T sumDL1p = 0; T sumDL2p = 0;

//- Line create: L1
    for(uint i = 0; i<points.size(); ++i) 
        distance(ptsL1[i], L1, ptsL1[i].d);
    std::sort(ptsL1.begin(),ptsL1.end(),sortDistance);

    /// Calculte distance sum
    for(uint i = 0; i<len_x-2; ++i) 
        sumDL1p += ptsL1[i].d; 
    
//- Line create: L2
    for(uint i = 0; i<ptsL2.size(); ++i) 
        distance(ptsL2[i], L2, ptsL2[i].d);
    std::sort(ptsL2.begin(),ptsL2.end(),sortDistance);

    /// Calculte distance sum
    for(uint i = 0; i<len_x-2; ++i) 
        sumDL2p += ptsL2[i].d; 

//- Define direction
    if( sumDL1p<sumDL2p ){ n_rows = len_y; n_cols = len_x;}
    else                 { n_rows = len_x; n_cols = len_y;} 

    std::sort(ptsL1.begin(),ptsL1.begin()+n_cols-2,sortX); // Sort x
    std::sort(ptsL2.begin(),ptsL2.begin()+n_rows-2,sortY); // Sort y

/*
 *  Points Array
 *  ------------
 */ 
    patron = Grid(n_rows, Pts(n_cols));
    patron[    0   ][    0   ] = points[ IL ];
    patron[n_rows-1][    0   ] = points[ SL ];
    patron[    0   ][n_cols-1] = points[ IR ];
    patron[n_rows-1][n_cols-1] = points[ SR ];

    for(uint i = 0; i<n_cols-2; ++i){
        ptsL1[i].check = false;
        patron[0][i+1] = ptsL1[i];
    }
    for(uint j = 0; j<n_rows-2; ++j){ 
        ptsL2[j].check = false;
        patron[j+1][0] = ptsL2[j];
    }
}


void addPatron(Pts  &points,
               Grid &patron,
               Line &L,
               const uint &n_rows, const uint &n_cols,
               const uint &position,
               const bool &horz){
    uint i;

//- Line horizontal
    if(horz){
        // Calculate distance
        for(i = 0; i<points.size(); ++i) 
            distance(points[i], L, points[i].d);
        
        // Sort
        std::sort(points.begin(),points.end(),sortDistance);
        std::sort(points.begin(),points.begin()+n_cols-2,sortX);
        
        for(i = 0; i<n_cols-2; ++i){
            points[i].check = false;
            patron[position][i+1] = points[i];
        }
    }

//- Line Vertical
    else{
        // Calculate distance
        for(i = 0; i<points.size(); ++i) 
            distance(points[i], L, points[i].d);

        // Sort
        std::sort(points.begin(),points.end(),sortDistance);
        std::sort(points.begin(),points.begin()+n_rows-2,sortY);

        for(i = 0; i<n_rows-2; ++i){
            points[i].check = false;
            patron[i+1][position] = points[i];
        }
    }
}


void mapping(Pts  &points,
             Grid &patron,
             const uint &n_rowsImg, const uint &n_colsImg,
             const uint &len_x, const uint &len_y){
/*
 *  Corners
 *  -------
 */ 
    uint IL, IR, SL, SR;
    getCorners(points,IL,IR,SL,SR, n_colsImg, n_rowsImg);
    
    points[ IL ].check = false; points[ IR ].check = false;
    points[ SL ].check = false; points[ SR ].check = false;
    
/*
 *  Definir orientacion
 *  -------------------
 */ 
    uint n_rows, n_cols;

    Line L1 = Line(points[ IL ],points[ IR ]);
    Line L2 = Line(points[ IL ],points[ SL ]);

    initPatron(points,patron,L1,L2,IL,IR,SL,SR,len_x,len_y,n_rows,n_cols);

/*
 *  Two more
 *  --------
 */ 
    Line L3 = Line(points[SL],points[SR]);
    Line L4 = Line(points[IR],points[SR]);

    addPatron(points,patron,L3,n_rows, n_cols,n_rows-1,true );
    addPatron(points,patron,L4,n_rows, n_cols,n_cols-1,false);

    
/*
 *  Main Loop
 *  ---------
 */ 
    Line L;
    for(uint i = 1; i<n_rows-1; ++i){
        L = Line(patron[i][0],patron[i][n_cols-1]);
        addPatron(points,patron,L,n_rows, n_cols,i);
    }
    
    
    std::cout << "\nPatron!:\n========" << std::endl;
    for(uint i = 0; i<n_rows; ++i){
        for(uint j = 0; j<n_cols; ++j){ 
            std::cout << "(" << patron[i][j].x << "," << patron[i][j].y << ")\t";
        }
        std::cout << std::endl;
    }
}
