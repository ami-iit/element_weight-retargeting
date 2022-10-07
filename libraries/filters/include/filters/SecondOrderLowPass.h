#ifndef RT_FILTERS_H
#define RT_FILTERS_H

#include <Eigen/Dense>
#include <deque>
#include <yarp/sig/Vector.h>


class SecondOrderLowPassFilter
{
protected:
    double fc;              // cut frequency
    double fs;              // sampling frequency

    double den_0;
    double den_1;
    double den_2;

    double num_0;
    double num_1;
    double num_2;

    std::deque<Eigen::VectorXd> u_old;
    std::deque<Eigen::VectorXd> y_old;

    void computeCoeff();

public:
    /**
    * Initialize a filter with specified parameters
    * @param cutFrequency cut frequency (Hz).
    * @param samplingFrequency sampling frequency (Hz).
    * @param dimension dimension output.
    */
    bool init(const double cutFrequency, const double samplingFrequency, size_t dimension);

    /**
    * Performs filtering on the actual input.
    * @param u reference to the actual input.
    * @return the corresponding output.
    */
    Eigen::VectorXd filt(Eigen::Ref<const Eigen::VectorXd> u);

    /**
    * Performs filtering on the actual input.
    * @param u reference to the actual input.
    * @return the corresponding output.
    */
    yarp::sig::Vector filt(yarp::sig::Vector u);

    /**
    * Performs filtering on the actual input.
    * @param u reference to the actual input.
    * @return the corresponding output.
    */
    std::vector<double> filt(std::vector<double> u);

};


#endif
