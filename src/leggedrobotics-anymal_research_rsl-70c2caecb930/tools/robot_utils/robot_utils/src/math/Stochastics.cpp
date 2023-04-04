/*!
 * @file 	Stochastics.cpp
 * @author 	Christian Gehring
 % @date 	June 27, 2013
 * @version 1.0
 * @ingroup robot_utils
 * @brief Copied and adapted from SL.
 */
#include "robot_utils/math/Stochastics.hpp"
#include <cmath>
#include <cstdio>  /* printf, NULL */
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */

// boost headers for random sampling
#include <boost/math/distributions.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

// eigen
#include <Eigen/LU>

namespace robot_utils {

#ifndef MY_RAND_MAX
#define MY_RAND_MAX 32767
#endif

/*!*****************************************************************************
 *******************************************************************************
 \note  myRan0
 \date  11/10/91

 \remarks

 this is the numerical recipes function to smoothen the problems with
 the C-function rand()

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     seed		: used for seeding rand() when function is called the first
 time, or when seed < 0!

 note: the returned number is between 0.0 and 1.0

 ******************************************************************************/
double myRan0(int* idum) {
  static double y, maxran, v[98];
  // double dum;
  static int iff = 0;
  int j;

  if (*idum < 0 || iff == 0) {
    iff = 1;
    maxran = MY_RAND_MAX + 1.0;
    srand(*idum);
    *idum = 1;
    // for (j = 1; j <= 97; j++) dum = rand() % MY_RAND_MAX;
    for (j = 1; j <= 97; j++) {
      v[j] = rand() % MY_RAND_MAX;
    } /* ensure sun-compatib. */
    y = rand() % MY_RAND_MAX;
  }

  j = static_cast<int>(1. + 97.0 * y / maxran);

  if (j > 97 || j < 1) {
    printf("Error: EstimatorManager::myRan0: This is nonsense\n");
    return 0;
  }
  y = v[j];
  v[j] = rand() % MY_RAND_MAX;

  return (y / maxran);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  myGasdev
 \date  11/10/91

 \remarks

 this is the numerical recipes function which returns a normally
 distributed deviate with zero mean and unit variance, using myRan0

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     seed		: used for seeding rand() when function is called the first
 time, or when seed < 0!


 ******************************************************************************/
double myGasdev(int* idum) {
  static int iset = 0;
  static double gset;
  double fac, r, v1, v2;

  if (iset == 0) {
    do {
      v1 = 2.0 * myRan0(idum) - 1.0;
      v2 = 2.0 * myRan0(idum) - 1.0;
      r = v1 * v1 + v2 * v2;
    } while (r >= 1.0);

    fac = sqrt(-2.0 * log(r) / r);
    gset = v1 * fac;
    iset = 1;
    return v2 * fac;

  } else {
    iset = 0;
    return gset;
  }
}

/*!*****************************************************************************
 *******************************************************************************
 \note  gaussian
 \date  August 17, 92	void getMiscSensors(std::vector<double>& miscSensors);
        double getMiscSensor(robotModel::miscSensorsEnum idx);

 \remarks

 randomly adds Gaussian	noise to a given value
 the noise level is interpreted as the 99% cutoff of the gaussian

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     value		: value to which the noise should be added
 \param[in]     std            : the standard deviation of the gaussian noise

 ******************************************************************************/
double gaussian(double value, double std) {
  static bool firsttime = true;
  double temp;
  int seed = 1;

  if (std == 0) {
    return value;
  }

  /* initialize the random function with an arbitrary number */

  if (firsttime) {
    firsttime = false;
    seed = -abs(static_cast<int>(time(nullptr)) % MY_RAND_MAX);
  }

  /* get me a value from myGasdev,
     which is normal distributed about 0, var=1 */

  temp = myGasdev(&seed) * std;

  return (value + temp);
}

double normalPdf(const Eigen::VectorXd& value, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov) {
  const Eigen::VectorXd unbiasedValue = value - mean;
  return (std::exp(static_cast<double>(-0.5 * unbiasedValue.transpose() * cov.inverse() * unbiasedValue)) /
          (std::sqrt(std::pow(2.0 * M_PI, unbiasedValue.size()) * cov.determinant())));
}

double normalPdf(double value, double mean, double std) {
  boost::math::normal norm(mean, std);
  return pdf(norm, value);
}

double normalCdf(double value, double mean, double std) {
  boost::math::normal norm(mean, std);
  return cdf(norm, value);
}

double sampleUniform() {
  static unsigned int seed = 0;
  boost::random::mt19937 rng;
  rng.seed((++seed) + time(nullptr));
  boost::uniform_real<> uni_dist(0, 1);
  boost::variate_generator<boost::random::mt19937, boost::uniform_real<> > uni(rng, uni_dist);
  return (uni)();  // always the same value.
}

double inverseNormal(double prob, double mean, double sd) {
  boost::math::normal myNormal(mean, sd);
  return quantile(myNormal, prob);
}

}  // namespace robot_utils
