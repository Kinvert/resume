#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

// Begin https://stackoverflow.com/questions/8286668/how-to-read-mnist-data-in-c
int reverseInt(int i)
{
    unsigned char c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return ((int)c1 << 24) + ((int)c2 << 16) + ((int)c3 << 8) + c4;
}
// End https://stackoverflow.com/questions/8286668/how-to-read-mnist-data-in-c

int main()
{
    vector<Eigen::MatrixXi> X_train;
    // LOAD THE MNIST DATA http://yann.lecun.com/exdb/mnist/
    // MNIST Data https://stackoverflow.com/questions/23253485/little-endian-reading-mnist-file-numbers-out-of-range
    // MNIST Data https://stackoverflow.com/questions/8286668/how-to-read-mnist-data-in-c
    // MNIST Data https://stackoverflow.com/questions/12993941/how-can-i-read-the-mnist-dataset-with-c
    // MNIST Data https://stackoverflow.com/questions/16871512/how-to-read-pixels-from-mnist-digit-database-and-create-the-iplimage?noredirect=1&lq=1
    // 3D Eigen https://stackoverflow.com/questions/17098218/most-efficient-option-for-build-3d-structures-using-eigen-matrices
    std::ifstream file("train-images.idx3-ubyte", std::ios::binary); // Must be uncompressed before use
    if (file.is_open())
    {
        int magic_number = 0;
        int number_of_images = 0;
        int n_rows = 0;
        int n_cols = 0;
        file.read((char*)&magic_number, sizeof(magic_number));
        magic_number = reverseInt(magic_number);
        file.read((char*)&number_of_images, sizeof(number_of_images));
        number_of_images = reverseInt(number_of_images);
        std::cout << number_of_images << endl;
        file.read((char*)&n_rows, sizeof(n_rows));
        n_rows = reverseInt(n_rows);
        std::cout << "n_rows = " << n_rows << endl;
        file.read((char*)&n_cols, sizeof(n_cols));
        n_cols = reverseInt(n_cols);
        std::cout << "n_cols = " << n_cols << endl;
        int thing;
        for (int i = 0; i < number_of_images; ++i)
        {
            Eigen::MatrixXi thisImage(28, 28);
            for (int r = 0; r < n_rows; ++r)
            {
                for (int c = 0; c < n_cols; ++c)
                {
                    unsigned char temp = 0;
                    file.read((char*)&temp, sizeof(temp));
                    thing = temp;
                    thisImage(r, c) = thing;
                }
            }
            X_train.push_back(thisImage);
        }
    }
    else {
        std::cout << "FAILED TO LOAD THE FILE";
        return 0;
    }

    cout << X_train[0];

    





    // Dynamic Matrix - Resizable
    // Stored in HEAP
    // Slower, but should be used for matrices larger than say 6X6
    Eigen::MatrixXd d; // "X" means size of matrix is unknown and "d" is for Double "f" would be float
    // d.resize(5,9);
    d = Eigen::MatrixXd::Random(1, 786);
    // Try to find an easy way to do a normal distribution

    // Fixed Size Matrix - Can't Resize
    // Stored in Stack - Faster
    // Use if 4X4 or smaller
    Eigen::Matrix3d f;
    f << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;

    f = Eigen::Matrix3d::Random(); // Maybe a way to get a normal distribution would be here https://stackoverflow.com/questions/11024820/set-coefficients-of-an-eigenmatrix-according-an-arbitrary-distribution

    cout << f;





    return 0;
}
