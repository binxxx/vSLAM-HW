#include <iostream>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

//#define MATRIX_SIZE 50 // maximum is 50 for fixed length Matrix
#define MATRIX_SIZE 100


using namespace std;

int main(int argc, char** argv)
{
//    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    // make the coefficient matrix positive definite
    matrix_NN = matrix_NN.transpose() * matrix_NN;
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    Eigen::Matrix<double, MATRIX_SIZE, 1> x;

    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "QR solver result:" << endl;
    cout << "x = \n" << x << endl;

    x = matrix_NN.llt().solve(v_Nd);
    cout << "Cholesky solver result:" << endl;
    cout << "x = \n" << x << endl;

    return 0;
}