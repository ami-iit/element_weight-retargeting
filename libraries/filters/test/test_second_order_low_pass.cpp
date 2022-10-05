#include <iostream>
#include <matioCpp/matioCpp.h>
#include <Eigen/Dense>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>

#include "filters/SecondOrderLowPass.h"


bool readStateFromFile(const std::string& filename, matioCpp::MultiDimensionalArray<double> & signal)
{
    std::deque<Eigen::VectorXd> data;

    matioCpp::File input(filename);

    if (!input.isOpen())
    {
        std::cout << "[Module::readStateFromFile] Failed to open " << filename << "." << std::endl;
        return false;
    }
    else
    {
        signal = input.read("FTs").asMultiDimensionalArray<double>(); // Read a multi dimensional
                                                                       // array named "traj"
        if (!signal.isValid())
        {
            std::cerr << "[Module::readStateFromFile] Error reading input file: " << filename << "."
                      << std::endl;
            return false;
        }

        return true;
    }
}

int main()
{

    double fc = 1.0;
    double fs = 50.0;

    SecondOrderLowPassFilter filter;

    if (!filter.init(fc, fs, 3))
    {
        std::cerr << "Unable to init the filter" << std::endl;
        return 1;
    }

    matioCpp::MultiDimensionalArray<double> mat_fts;

    if (!readStateFromFile("matlab.mat", mat_fts))
    {
        std::cerr << "Unable to read mat file" << std::endl;
        return 1;
    }

    auto fts = BipedalLocomotion::Conversions::toEigen(mat_fts);

    for (int i = 0; i < fts.rows(); i++)
    {
        auto filtered = filter.filt(fts.row(i));
        std::cout << "Filtered signal:" << std::endl;
        for (int j = 0; j < fts.cols(); j++)
        {
            std::cout << filtered[j] << std::endl;
        }
    }

    return 0;
}
