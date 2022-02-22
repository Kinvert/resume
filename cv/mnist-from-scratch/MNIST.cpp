#include <Eigen/Dense>
#include <Eigen/src/Core/Product.h>

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

Eigen::MatrixXd np_dot(Eigen::MatrixXd error, Eigen::MatrixXd weights) {
    /*
    Basically a Numpy Dot Product. Eigen doesn't seem to do this but who knows maybe they do.
    */
    Eigen::MatrixXd output(1, weights.cols());
    for (int i = 0; i < weights.cols(); i++) {
        double thesum = 0;
        for (int j = 0; j < weights.rows(); j++) {
            thesum += error(j) * weights(j, i);
        }
        output(i) = thesum;
    }
    return output;
}

Eigen::MatrixXd relu_grad(Eigen::MatrixXd error, Eigen::MatrixXd res_rel) {
    /*
       error = np.dot(out.T, error) * (res_rel1 > 0)     This is the (res_rel > 0) portion
    */
    Eigen::MatrixXd output(1, res_rel.rows());
    for (int i = 0; i < res_rel.rows(); i++) {
        if (res_rel(i) > 0) {
            output(i) = error(i);
        }
        else {
            output(i) = 0;
        }
    }
    return output;
}

int main()
{
    /*
    ######################################################
                       LOAD Y DATA
    ######################################################
    */
    int div_by = 100;
    int epochs = 8;
    double lr = 0.25;
    
    vector<Eigen::MatrixXi> Y_train;
    std::ifstream filey("train-labels.idx1-ubyte", std::ios::binary);
    if (filey.is_open())
    {
        int magic_number = 0;
        int number_of_images = 0;
        int n_rows = 0;
        int n_cols = 0;
        filey.read((char*)&magic_number, sizeof(magic_number));
        magic_number = reverseInt(magic_number);
        filey.read((char*)&number_of_images, sizeof(number_of_images));
        number_of_images = reverseInt(number_of_images);
        std::cout << number_of_images << endl;
        int label_int;
        for (int i = 0; i < number_of_images/div_by; i++)
        {
            Eigen::MatrixXi label;
            label = Eigen::MatrixXi::Zero(1, 10);
            unsigned char temp = 0;
            filey.read((char*)&temp, sizeof(temp));
            label_int = temp;
            label(0, label_int) = 1;
            Y_train.push_back(label);
        }
    }
    else {
        std::cout << "FAILED TO LOAD THE FILE";
        return 0;
    }
    

    /*
    ######################################################
                       LOAD X DATA
    ######################################################
    */
     
    vector<Eigen::MatrixXd> X_train;
    vector<Eigen::MatrixXd> X_train_flat;
    // LOAD THE MNIST DATA http://yann.lecun.com/exdb/mnist/
    // MNIST Data https://stackoverflow.com/questions/23253485/little-endian-reading-mnist-file-numbers-out-of-range
    // MNIST Data https://stackoverflow.com/questions/8286668/how-to-read-mnist-data-in-c
    // MNIST Data https://stackoverflow.com/questions/12993941/how-can-i-read-the-mnist-dataset-with-c
    // MNIST Data https://stackoverflow.com/questions/16871512/how-to-read-pixels-from-mnist-digit-database-and-create-the-iplimage?noredirect=1&lq=1
    // 3D Eigen https://stackoverflow.com/questions/17098218/most-efficient-option-for-build-3d-structures-using-eigen-matrices
    std::ifstream file("train-images.idx3-ubyte", std::ios::binary);
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
        //int thing;
        //int mnist_int;
        //double mnist_double;
        for (int i = 0; i < number_of_images/div_by; i++)
        {
            Eigen::MatrixXd thisImage(28, 28);
            Eigen::MatrixXd thisImageFlat(1, 784);
            for (int r = 0; r < n_rows; ++r)
            {
                for (int c = 0; c < n_cols; ++c)
                {
                    unsigned char temp = 0;
                    file.read((char*)&temp, sizeof(temp));
                    int mnist_int = temp;
                    //int idx = r * 28 + c;
                    double mnist_double = mnist_int / 255.0;
                    thisImage(r, c) = mnist_double;
                    //thisImageFlat(0, idx);
                }
            }
            thisImage.resize(1, 784);
            X_train.push_back(thisImage);
            //X_train_flat.push_back(thisImageFlat);
        }
    }
    else {
        std::cout << "FAILED TO LOAD THE FILE";
        return 0;
    }

    // Layers
    Eigen::MatrixXd w0;
    Eigen::MatrixXd w1;
    Eigen::MatrixXd out;
    w0 = Eigen::MatrixXd::Random(64, 784);
    w1 = Eigen::MatrixXd::Random(32, 64);
    out = Eigen::MatrixXd::Random(10, 32);
    //w0 = w0.array().abs() * sqrt(1 / (double)(64 + 784));
    //w1 = w1.array().abs() * sqrt(1 / (double)(32 + 64));
    //out = out.array().abs() * sqrt(1 / (double)(10 + 32));
    w0 = ((w0.array() + 1) / (double)2) * sqrt(1 / (double)(64 + 784));
    w1 = ((w1.array() + 1) / (double)2) * sqrt(1 / (double)(32 + 64));
    out = ((out.array() + 1) / (double)2) * sqrt(1 / (double)(10 + 32));

    // Main Loop
    for (int epoch = 0; epoch < epochs; epoch++) {
        double correct_percent;
        int corrects_sum = 0;
        int img_num = 0;
        vector<int> corrects;
        Eigen::MatrixXd old_dx_out;
        Eigen::MatrixXd old_dx_w1;
        Eigen::MatrixXd old_dx_w0;
        old_dx_out = Eigen::MatrixXd::Zero(10, 32);
        old_dx_w1 = Eigen::MatrixXd::Zero(32, 64);
        old_dx_w0 = Eigen::MatrixXd::Zero(64, 784);
        Eigen::MatrixXd vold_dx_out;
        Eigen::MatrixXd vold_dx_w1;
        Eigen::MatrixXd vold_dx_w0;
        vold_dx_out = Eigen::MatrixXd::Zero(10, 32);
        vold_dx_w1 = Eigen::MatrixXd::Zero(32, 64);
        vold_dx_w0 = Eigen::MatrixXd::Zero(64, 784);

        for (int i = 0; i < int(60000/div_by); i++) {

            // Data Matrices
            Eigen::MatrixXd x(1, 784);
            x = X_train[i];
            Eigen::MatrixXi y(1, 10);
            y = Y_train[i];

            // Forward Pass
            Eigen::MatrixXd res_w0(64,1);
            Eigen::MatrixXd res_rel0(64, 1);
            Eigen::MatrixXd res_w1(32, 1);
            Eigen::MatrixXd res_rel1(32, 1);
            Eigen::MatrixXd res_out(10, 1);
            Eigen::MatrixXd guess(10, 1);
            Eigen::MatrixXd error(10, 1);
            
            res_w0 << w0 * x.transpose();
            res_rel0 << res_w0.cwiseMax(0); // ReLU

            res_w1 << w1 * res_rel0;
            res_rel1 << res_w1.cwiseMax(0);

            res_out << out * res_rel1;
            guess << exp(res_out.array() - res_out.maxCoeff()) / (exp(res_out.array() - res_out.maxCoeff())).sum(); // guess = np.exp(res_out - res_out.max()) / np.sum(np.exp(res_out - res_out.max()), axis=0)
            error << guess(0) - y(0), guess(1) - y(1), guess(2) - y(2), guess(3) - y(3), guess(4) - y(4), guess(5) - y(5), guess(6) - y(6), guess(7) - y(7), guess(8) - y(8), guess(9) - y(9);
            
            // Record Corrects After Forward Pass
            double guess_max = 0;
            int guess_idx = 0;
            for (int k = 0; k < 10; k++) {
                if (guess(k) > guess_max) {
                    guess_max = guess(k);
                    guess_idx = k;
                }
            }
            if (y(guess_idx) == 1) {
                corrects.push_back(1);
                corrects_sum += 1;
            }
            else {
                corrects.push_back(0);
            }
            //cout << endl << "guess = " << guess;
            //cout << endl << "y     = " << y << endl << endl;

            // Backward Pass
            Eigen::MatrixXd dd(10, 1);
            Eigen::MatrixXd dx_out(10, 32);
            Eigen::MatrixXd error_new(32,1);
            Eigen::MatrixXd dx_w1(32, 64);
            Eigen::MatrixXd error_final(64, 1);
            Eigen::MatrixXd dx_w0(64, 784);

            dd << guess.array() * (1 - guess.array()); // (10, 1)
            error << error(0) * dd(0), error(1) * dd(1), error(2) * dd(2), error(3) * dd(3), error(4) * dd(4), error(5) * dd(5), error(6) * dd(6), error(7) * dd(7), error(8) * dd(8), error(9) * dd(9); // (10, 1)    This isn't what I had in Python
            dx_out << error * res_rel1.transpose(); // (10, 32)
            error_new = relu_grad( np_dot(error.transpose(), out), res_rel1); // (1, 32)
            dx_w1 = error_new.transpose() * res_rel0.transpose(); // (32, 64)
            error_final = relu_grad(np_dot(error_new, w1), res_rel0); // (64, 1)
            dx_w0 = error_final.transpose() * x; // (64, 784)

            // Adjust Weights
            out = out.array() - lr * dx_out.array() - 0.5 * lr * old_dx_out.array() - 0.25 * lr * vold_dx_out.array();
            w0 = w0.array() - lr * dx_w0.array() - 0.5 * lr * old_dx_w0.array() - 0.25 * lr * vold_dx_w0.array();
            w1 = w1.array() - lr * dx_w1.array() - 0.5 * lr * old_dx_w1.array() - 0.25 * lr * vold_dx_w1.array();
            if (i > 0) {
                correct_percent = corrects_sum / (double)i;
            }
            vold_dx_out = old_dx_out;
            vold_dx_w1 = old_dx_w1;
            vold_dx_w0 = old_dx_w0;
            old_dx_out = dx_out;
            old_dx_w1 = dx_w1;
            old_dx_w0 = dx_w0;
        }
        cout << endl << "Correct Percent = " << correct_percent;
    }
    return 0;
}
