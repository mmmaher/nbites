/**
 * KalmanFilter - Implementation of a Kalman filter for ball tracking
 *
 * @author EJ Googins <egoogins@bowdoin.edu>
 * @date   March 2013
 */

#pragma once

#include "NBMath.h"
#include "NBMatrixMath.h"
#include "NBVector.h"
#include "Common.h"

#include "BallModel.pb.h"
#include "Motion.pb.h"
#include "RobotLocation.pb.h"

#include "BallTrackStructs.h"

using namespace NBMath;

namespace man {
namespace balltrack {

static const int BUFF_LENGTH = 6;

// HACK HACK HACK - Currently completely made up and untested
// @TODO test this and determine correct values
static const KalmanFilterParams DEFAULT_PARAMS = {
    .5f,             // transXDeviation   CALC March 2013
    .5f,             // trandYDeviation   CALC March 2013
    .5f,             // rotationDeviation CALC March 2013
    10.f,            // obsvRelX deviation
    10.f,            // obsvRelY deviation
    .1f,             // processDeviation
    .1f,
    .1f,
    .1f,
    -20.f            // ballFriction?
};

class KalmanFilter {
public:
    KalmanFilter(bool stationary_ = true,
                 KalmanFilterParams params_ = DEFAULT_PARAMS);

    void update(messages::VisionBall visionBall,
                messages::RobotLocation motion);

    bool isUpdated() { return m_updated; }
    void setUpdate(bool updated_) { m_updated = updated_; }

    void initialize();
    void initialize(ufvector4 x_, ufmatrix4 cov_);

    void predict(messages::RobotLocation odometry);
    void predict(messages::RobotLocation odometry, float deltaT);
    void updateWithObservation(messages::VisionBall visionBall);
    void predictBallDest();

    // FOR OFFLINE TESTING
    messages::RobotLocation genOdometry(float x, float y, float h) {
        messages::RobotLocation odometry;
        odometry.set_x(x);
        odometry.set_y(y);
        odometry.set_h(h);

        return odometry;
    };

    messages::VisionBall genVisBall(float dist, float bear) {
        messages::VisionBall obsv;
        obsv.set_rel_x_variance(5.f);
        obsv.set_rel_y_variance(5.f);
        obsv.set_distance(dist);
        obsv.set_bearing(bear);
        obsv.set_on(true);

        return obsv;
    };

    ufvector4 getStateEst() { return m_x; }
    float getRelXPosEst() { return m_x(0); }
    float getRelYPosEst() { return m_x(1); }
    float getRelXVelEst() { return m_x(2); }
    float getRelYVelEst() { return m_x(3); }

    ufmatrix4 getCovEst() { return m_cov; }
    float getCovXPosEst() { return m_cov(0,0); }
    float getCovYPosEst() { return m_cov(1,1); }
    float getCovXVelEst() { return m_cov(2,2); }
    float getCovYVelEst() { return m_cov(3,3); }

    float getFilteredDist() { return m_filtered_dist; }
    float getFilteredBear() { return m_filtered_bear; }

    float getRelXDest() { return m_rel_x_dest; }
    float getRelYDest() { return m_rel_y_dest; }
    float getRelYIntersectDest() { return m_rel_y_intersect_dest; }

    bool isStationary() { return m_stationary; }

    float getSpeed() { return std::sqrt(m_x(2) * m_x(2) + m_x(3) * m_x(3)); }

    void printEst(){std::cout << "Filter Estimate:\n\t"
                              << "'Stationary' is\t" << m_stationary << "\n\t"
                              << "X-Pos:\t" << m_x(0) << "Y-Pos:\t" << m_x(1)
                              << "\n\t"
                              << "x-Vel:\t" << m_x(2) << "y-Vel:\t" << m_x(3)
                              << "\n\t"
                              << "Uncertainty x:\t" << m_cov(0,0) << "\t,\t"
                              << "y:\t" << m_cov(1,1) << std::endl; }

    void clearErrorBuffer() { m_correction_buffer.clear(); }
    double getAvgErr() { return m_correction_buffer.get_average_error(); }

protected:
    /** Inner class to track the correction buffer
     *
     *  If full, return the average error across the buffer
     *  Can add or clear */
    class correction_buffer_ {
    public:
        correction_buffer_() : m_correction_mag_buffer(new float[BUFF_LENGTH]),
            m_cur_index(0), m_full_buffer(false) {}

        ~correction_buffer_()
        {
            delete m_correction_mag_buffer;
        }

        bool is_full() { return m_full_buffer; }

        /** Load new value, update index, determine if full */
        void add_correction(double correction)
        {
            std::cout << "add correction" << std::endl;
            m_correction_mag_buffer[m_cur_index] = correction;

            m_cur_index = (m_cur_index + 1) % BUFF_LENGTH;
            std::cout << "m_cur_index: " << m_cur_index << std::endl;
            m_full_buffer = m_full_buffer || m_cur_index == 0;
        }

        void clear()
        {
            m_cur_index = 0;
            m_full_buffer = false;
        }

        /** If full buffer,return average err, else return -1 */
        double get_average_error()
        {
            // Need some outlier rejection

            if (is_full()) {
                double sum_error = 0;

                for (int i=0; i < BUFF_LENGTH; ++i) {
                    sum_error += m_correction_mag_buffer[i];
                }

                return sum_error / BUFF_LENGTH;
            } else {
                return -1;
            }
        }

    protected:
        float* m_correction_mag_buffer;

        int m_cur_index;

        bool m_full_buffer;

    };

    correction_buffer_ m_correction_buffer;

    KalmanFilterParams m_params;

    bool m_updated;

    void updateDeltaTime();

    // x is the state estimate, it holds relX & relY position and
    //                                   relX & relY velocity
    // This is the convention for all matrices and vectors
    ufvector4 m_x;

    // Covariance matrix for the state estimation
    ufmatrix4 m_cov;

    // Last calculated kalman gain
    float m_gain;

    // Time passed since last vision observation
    long long int m_last_update_time;
    float m_delta_time;

    // true if the filter assumes the ball is stationary, if stationary
    // is true, the velocitys should be zero
    bool m_stationary;

    // temp move to public for testing
    //    void predict(messages::RobotLocation odometry, float deltaT);

    float m_filtered_dist;
    float m_filtered_bear;

    float m_rel_x_dest;
    float m_rel_y_dest;
    float m_rel_y_intersect_dest;

    // For the MMKalman
    float m_score;
    float m_weight;
    int m_cur_entry;
    float m_uncertainty;

    messages::RobotLocation m_last_odometry;
};

} // balltrack
} // man
