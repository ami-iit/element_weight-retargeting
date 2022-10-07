#include <math.h>
#include <iostream>

#include <yarp/eigen/Eigen.h>

#include "filters/SecondOrderLowPass.h"


bool SecondOrderLowPassFilter::init(const double cutFrequency,
                                    const double sampleFrequency,
                                    size_t dimension)
{
    if (cutFrequency <= 0)
    {
        std::cerr << "Requested a positive cut frequency" << std::endl;
        return false;
    }

    if (sampleFrequency <= 0)
    {
        std::cerr << "Requested a positive sampling frequency" << std::endl;
        return false;
    }

    if (dimension <= 0)
    {
        std::cerr << "Requested a positive dimension" << std::endl;
        return false;
    }

    fc = cutFrequency;

    fs = sampleFrequency;

    computeCoeff();

    u_old.resize(2);
    y_old.resize(2);

    for (int i = 0; i < 2; i++)
    {
        u_old[i] = Eigen::VectorXd::Zero(dimension);
        y_old[i] = Eigen::VectorXd::Zero(dimension);
    }

    return true;
}

void SecondOrderLowPassFilter::computeCoeff()
{
    double alfa = 1.0 / tan(M_PI * fc / fs);
    double Q = sqrt(2);
    double alfa_squared = alfa * alfa;

    num_0 = 1.0 / (1 + Q * alfa + alfa_squared);
    num_1 = 2 * num_0;
    num_2 = num_0;

    den_0 = 1;
    den_1 = -2.0 * (alfa_squared - 1.0) * num_0;
    den_2 = (1 - Q * alfa + alfa_squared) * num_0;
}

Eigen::VectorXd SecondOrderLowPassFilter::filt(Eigen::Ref<const Eigen::VectorXd> u)
{
    Eigen::VectorXd y = num_0 * u;

    y += num_1 * u_old[1];

    y += num_2 * u_old[0];

    y -= den_1 * y_old[1];

    y -= den_2 * y_old[0];

    u_old.pop_front();
    u_old.push_back(u);

    y_old.pop_front();
    y_old.push_back(y);

    return y;
}

yarp::sig::Vector SecondOrderLowPassFilter::filt(yarp::sig::Vector u)
{
    yarp::sig::Vector y_yarp;

    Eigen::VectorXd y_eigen;

    y_eigen = filt(yarp::eigen::toEigen(u));

    y_yarp.reserve(y_eigen.size());
    for (int i = 0; i < y_eigen.size(); i++)
    {
        y_yarp.push_back(y_eigen[i]);
    }

    return y_yarp;
}

std::vector<double> SecondOrderLowPassFilter::filt(std::vector<double> u)
{
    std::vector<double> y_ret;
    y_ret.resize(u.size());

    Eigen::VectorXd y_eigen;

    y_eigen = filt(Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(u.data(), u.size()));

    for(int i=0; i<u.size(); i++)
    {
        y_ret[i] = y_eigen[i];
    }

    return y_ret;
}
