#include <iostream>
#include <Eigen/Dense> // Required for dense matrices and vectors

int main()
{
    // --- Matrix-Matrix Multiplication Example ---

    // Define two 3x3 matrices of doubles with fixed sizes
    Eigen::Matrix3d A;
    A << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;
    
    Eigen::Matrix3d B;
    B << 9.0, 8.0, 7.0,
         6.0, 5.0, 4.0,
         3.0, 2.0, 1.0;

    // Perform the multiplication and store the result in a new 3x3 matrix C
    const auto C = A * B;

    std::cout << "Matrix A:\n" << A << std::endl;
    std::cout << "\nMatrix B:\n" << B << std::endl;
    std::cout << "\nProduct of A * B:\n" << C << std::endl;

    // --- Matrix-Vector Multiplication Example ---
    
    // Define a 3x1 vector of doubles (Vector3d is a special case of Matrix<double, 3, 1>)
    Eigen::Vector3d v(0.5, 3.0, -0.4);

    // Perform matrix-vector multiplication
    Eigen::Vector3d Av = A * v;

    std::cout << "\nVector v:\n" << v << std::endl;
    std::cout << "\nProduct of A * v:\n" << Av << std::endl;

    return 0;
}
