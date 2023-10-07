/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-17 11:03:25
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-10-07 14:26:48
 */
#include <iostream>
#include "../../include/assignment/Miller.h"
using namespace std;

double cost_c[10][10] = {
    1.74816487, 2.38917233, 1.71076069, 3.39056081, 8.97918614, 8.8463572,
    5.72492498, 3.51112043, 2.83719919, 1.47646523, 6.90482939, 6.54823362,
    2.97598861, 8.58927493, 1.27471997, 6.63336473, 8.00192929, 5.53644708,
    8.17686098, 6.53984023, 5.12970743, 5.15610536, 6.76563599, 3.63906049,
    9.5657066, 0.9938076, 4.65050956, 9.40180311, 7.40438347, 2.76392061,
    0.14804986, 2.46669343, 7.33323472, 5.8211227, 7.97660068, 4.25715621,
    8.70762212, 3.84921524, 3.19890027, 2.28689383, 8.34067808, 2.06432393,
    5.28740296, 4.65337427, 7.83300603, 8.53227427, 5.38427513, 1.03191784,
    6.6057736, 7.68792094, 4.62337316, 9.95950717, 7.65598475, 2.33958583,
    4.71393984, 8.73278614, 5.13445941, 8.88417655, 5.28262101, 1.08137045,
    6.5017676, 3.71347059, 8.90070478, 6.89908671, 9.3396071, 9.69360009,
    8.39359751, 9.25831462, 9.28462701, 4.67101498, 0.19922305, 8.61400931,
    4.97661521, 2.94110684, 4.14077323, 4.74816978, 4.42211109, 3.70811997,
    2.46486932, 6.42482562, 7.4039233, 3.37486973, 0.27083053, 0.18565782,
    5.25106232, 2.51429459, 8.12555989, 2.01157174, 9.21221066, 2.54981598,
    7.40352095, 7.36382558, 0.7780371, 1.78572676, 1.72834597, 8.56007773,
    8.72230221, 7.66976083, 7.88648666, 0.24672};

int main()
{
    // use cols > rows!
    MurtyMiller<double>::WeightMatrix c_ij(10, 10);
    typedef MurtyMiller<double>::Edges Edges;

    c_ij = MurtyMiller<double>::WeightMatrix::Random(10, 10);

    for (size_t r = 0; r < c_ij.rows(); ++r)
        for (size_t c = 0; c < c_ij.cols(); ++c)
            // if ( c_ij(r, c) < 0 ) c_ij(r, c) = -c_ij(r, c);
            c_ij(r, c) = cost_c[r][c];

    c_ij /= c_ij.maxCoeff();

    // std::cerr << "c_ij = \n" << c_ij << std::endl;

    /* Begin Time */
    clock_t start = clock();
    std::vector<Edges> solutions = MurtyMiller<double>::getMBestAssignments(c_ij, 3);
    clock_t end = clock();
    double time = (double)(end - start) / CLOCKS_PER_SEC * 1000.0;
    cout << "Total CPU time:\t" << time << endl;

    for (const auto &s : solutions)
    {
        // for ( const auto & e : s )
        //     std::cerr << "(" << e.x << ", " << e.y << ") ";
        std::cerr << "sum = " << MurtyMiller<double>::objectiveFunctionValue(s) << std::endl;
    }

    return 0;
}
