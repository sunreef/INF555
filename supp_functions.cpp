//
//  supp_functions.cpp
//  INF555___Project
//
//  Created by Galashov Alexandr on 08/11/2015.
//
//

#include "supp_functions.hpp"

int phys_to_graph(int window_size, double value)
{
   // double left = value * window_size;
    return (int) ((double) value * window_size - (double) window_size / 2.0);
}
double graph_to_phys(int window_size, int value)
{
    return (int)(((double)value + (double) window_size / 2.0 )  / (double)window_size );
}