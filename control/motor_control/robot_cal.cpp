#include <iostream> 
#include <fstream> 
#include <string>
#include <vector>
#include <stdlib.h> 
#include "gyems_can_functions.h" 
#include "renishaw_can_functions.hpp" 
using namespace std; 

#include <unistd.h> 
#include <signal.h>  
#include <cmath> 
#include <stdio.h>

#include <Eigen/Dense>

using namespace Eigen


void Jacobian(double theta_1_t, double theta_2_t, const <MatrixXd> J):
{
    MatrixXd m(2,2); 
    m(0, 0) = 0.0; 
    m(1, 0) = 0.0; 

    m(1, 0) = 0.0; 
    m(1, 1) = 0.0; 
}