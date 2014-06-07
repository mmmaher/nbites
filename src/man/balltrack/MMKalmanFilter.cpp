#include "MMKalmanFilter.h"

namespace man {
namespace balltrack {

/**
 * @ Brief- Constructor of my 'Puppetmaster'
 *          Grab the params, gen a bunch of filters to avoid nulls
 *          Set frames w/o ball high so we re-init based on initial observations
 *          Clear consecutive observation is false if we havent seen the ball...
 *          State est and vis history can stay zero,
 *          Same with bestFilter, stationary, and lastUpdateTime
 */
MMKalmanFilter::MMKalmanFilter(MMKalmanFilterParams params_)
{
    m_params = params_;

    m_frames_without_ball = m_params.framesTillReset;
    m_consecutive_observation = false;
    m_best_filter = 0;
    m_obsv_buffer = new CartesianObservation[m_params.bufferSize];
    m_cur_entry = 0;
    m_full_buffer = false;

    m_prev_state_est = boost::numeric::ublas::zero_vector<float> (4);
    m_prev_cov_est   = boost::numeric::ublas::identity_matrix <float>(4);
    m_state_est     = boost::numeric::ublas::zero_vector<float> (4);
    m_cov_est       = boost::numeric::ublas::identity_matrix <float>(4);

    initialize();
}

/**
 * @Brief - #nomememoryleaks
 *           (hopefully)
 */
MMKalmanFilter::~MMKalmanFilter()
{
    // EJ--fixme?
    //delete &filters;
}

/**
 * @Brief - Main interface, takes in an update with a visionBall and a motion message
 *          Should be called whenever new information, and simply dont pass it the same
 *               message twice!
 * @params - visionball is a NEW vision message
           - motion is a NEW motion message
 */
void MMKalmanFilter::update(messages::VisionBall    visionBall,
                            messages::RobotLocation odometry)
{
    // Predict the filters given odometry
    predictFilters(odometry);

    std::cout << "vision ball on: " << visionBall.on() << std::endl;
    std::cout << "          dist: " << visionBall.distance() << std::endl;

    // Determine if provided with a valid observation
    if (visionBall.on() && (visionBall.distance() > 5) && (visionBall.distance() < 800)) {

    //    If haven't seen ball for threshold
        if (m_frames_without_ball >= m_params.framesTillReset) {
    //       re initialize filters
            initialize(m_vis_rel_x, m_vis_rel_y, m_params.initCovX, m_params.initCovY);
    //       reset observation buffer
            m_full_buffer = false;
            m_cur_entry = 0;
        }

    //    Update visual observation history
    //       Update previous relx/y and calculate current
        // Easy acces to last frame values
        m_last_vis_rel_x = m_vis_rel_x;
        m_last_vis_rel_y = m_vis_rel_y;

        // Calc relx and rely from vision
        float sinB, cosB;
        sincosf(visionBall.bearing(), &sinB, &cosB);
        m_vis_rel_x = visionBall.distance()*cosB;
        m_vis_rel_y = visionBall.distance()*sinB;

    //       Update the history buffer
        // Add to the observation buffer
        m_cur_entry = (m_cur_entry + 1) % m_params.bufferSize;
        m_obsv_buffer[m_cur_entry] = CartesianObservation(m_vis_rel_x, m_vis_rel_y);

    //       Determine if buffer is full
        // If buffer wasnt full but is now
        if (!m_full_buffer && m_cur_entry == 0) {
            m_full_buffer = true;
        }

    //    Determine if consecutive observation
        // Decide if we saw the ball twice in a row
        m_consecutive_observation = (m_frames_without_ball == 0) ? true : false;

    //    Update all filters with current observation
        // Now correct our filters with the vision observation
        updateWithVision(visionBall);
        //updatePredictions();
    } else {
    // If no valid observation
    //    Kill buffer
        m_full_buffer = false;
        m_cur_entry = 0;
    //    Kill consecutive observations
        m_consecutive_observation = false;

        // Inform each KF err buffer of the lack of observation
        clearFilterErr();

    }

    // Cycle through filters, reinit as necessary
    cycleFilters();

    // Select the optimal filter
    setBestFilter();

    // House clean; stationary, frames without ball, prev ests
    m_prev_state_est = m_state_est;
    m_prev_cov_est   = m_cov_est;

    m_state_est = m_best_filter->getStateEst();
    m_cov_est   = m_best_filter->getCovEst();

    if (m_best_filter->isStationary()) {
        std::cout << "stationary" << std::endl;
    } else {
        std::cout << "MOVING: " << std::endl;
    }

    std::cout << "speed: " << m_best_filter->getSpeed() << std::endl;

    // Housekeep
    m_frames_without_ball = (visionBall.on()) ? (0) : (m_frames_without_ball+1);
    m_stationary = m_best_filter->isStationary();
}

/**
 * @brief - In charge of cycling through the filters, finding the worst stationary
 *           and replacing it with a new init filter. Also re-inits a new moving filter
 *            if we have had two consecutive observations and can calculate velocity
 */
void MMKalmanFilter::cycleFilters()
{
    //Find the two worst filters
    KalmanFilter* worstStationary = 0;
    KalmanFilter* worstMoving = 0;

    // Track the number of filters actively competing. At least have 6 filters
    int num_active_filters = 0;

    for (std::vector<KalmanFilter*>::iterator it = m_filters.begin(); it != m_filters.end(); it++) {
        KalmanFilter* kf = *it;

        if (kf->getAvgErr() > 0) {
            ++num_active_filters;
            if (kf->isStationary()) {
                if (!worstStationary || kf->getAvgErr() > worstStationary->getAvgErr()) {
                    worstStationary = kf;
                }
            } else {
                if (!worstMoving|| kf->getAvgErr() > worstMoving->getAvgErr()) {
                    worstMoving= kf;
                }
            }
        }
    }

    // If valid worst stationary filter, re-initialize

    // Create init vector/matrix if will be re-initializing a filter
    if ((!worstMoving && !worstStationary) || num_active_filters < 8) return;
    ufvector4 newX = boost::numeric::ublas::zero_vector<float>(4);
    newX(0) = m_last_vis_rel_x;
    newX(1) = m_last_vis_rel_y;
    ufmatrix4 newCov = boost::numeric::ublas::zero_matrix<float>(4);
    newCov(0,0) = m_params.initCovX;
    newCov(1,1) = m_params.initCovY;

    // Re-init the worst stationary filter
    if (worstStationary) {
        worstStationary->initialize(newX, newCov);
    }

    // Re-init the worst moving filter if it exists
    if (worstMoving) {
        if (m_full_buffer) {
            // Compute average velocity throughout buffer. Need outlier rejection
        }

        newX(2) = (m_vis_rel_x - m_last_vis_rel_x) / m_delta_time;
        newX(3) = (m_vis_rel_y - m_last_vis_rel_y) / m_delta_time;

        // HACK - magic number. need this in master asap though
        newCov(2,2) = 30.f;
        newCov(3,3) = 30.f;

        std::cout << "initialize new moving filter: " << std::endl;

        worstMoving->initialize(newX, newCov);
    }
}

/**
 * @brief - Initialize all the filters!
 * @params- given a relX and relY for the position mean
 *          also a covX and covY since we may want to init
 *          w/ diff certainties throughout the life
 * @choice  I chose to have the velocities randomly initialized since there are
            soooo many combos
 */
void MMKalmanFilter::initialize(float relX, float relY, float covX, float covY)
{
    // clear the filters
    m_filters.clear();

    // make a random generator for initilizing different filters
    boost::mt19937 rng;
    rng.seed(std::time(0));
    boost::uniform_real<float> posCovRange(-2.f, 2.f);
    boost::variate_generator<boost::mt19937&,
            boost::uniform_real<float> > positionGen(rng, posCovRange);
    boost::uniform_real<float> randVelRange(-30.f, 30.f);
    boost::variate_generator<boost::mt19937&,
            boost::uniform_real<float> > velocityGen(rng, randVelRange);

    // make stationary
    for (int i=0; i < m_params.numFilters/2; i++) {
        // Needs to be stationary, have given mean, and add noise
        //   to the covariance matrix
        KalmanFilter* stationaryFilter = new KalmanFilter(true);
        ufvector4 x = boost::numeric::ublas::zero_vector<float>(4);
        x(0) = relX;
        x(1) = relY;
        x(2) = 0.f;
        x(3) = 0.f;

        ufmatrix4 cov = boost::numeric::ublas::zero_matrix<float>(4);
        cov(0,0) = covX + positionGen();
        cov(1,1) = covY + positionGen();

        // init and push it back
        stationaryFilter->initialize(x, cov);
        m_filters.push_back(stationaryFilter);
    }

    // make moving
    for (int i=0; i<m_params.numFilters/2; i++) {
        // Needs to be moving, have given mean, and add noise
        //   to the covariance matrix
        KalmanFilter* movingFilter = new KalmanFilter(false);
        ufvector4 x = boost::numeric::ublas::zero_vector<float>(4);
        x(0)= relX;
        x(1)= relY;
        x(2) = velocityGen();
        x(3) = velocityGen();

        // Choose to assum obsv mean is perfect and just have noisy velocity
        ufmatrix4 cov = boost::numeric::ublas::zero_matrix<float>(4);
        cov(0,0) = covX;
        cov(1,1) = covY;
        cov(2,2) = 20.f;
        cov(3,3) = 20.f;

        movingFilter->initialize(x, cov);
        m_filters.push_back(movingFilter);
    }
}

// for offline testing, need to be able to specify the time which passed
void MMKalmanFilter::predictFilters(messages::RobotLocation odometry, float t)
{
    m_delta_time = t;
    for (std::vector<KalmanFilter*>::iterator it = m_filters.begin();
         it != m_filters.end(); it++) {
        (*it)->predict(odometry, m_delta_time);
    }
}
/**
 * @brief - Predict each of the filters given info on where robot has moved
 *          Grab delta time from the system and then call the predict on each filter
 *
 */
void MMKalmanFilter::predictFilters(messages::RobotLocation odometry)
{
    // Update the time passed
    updateDeltaTime();

    // Update each filter
    for (std::vector<KalmanFilter*>::iterator it = m_filters.begin();
         it != m_filters.end(); it++) {
        (*it)->predict(odometry, m_delta_time);
    }
}

/**
 * @brief - Correct each filter given an observation
 *          Pretty straightforward...
 */
void MMKalmanFilter::updateWithVision(messages::VisionBall visionBall)
{
    for (std::vector<KalmanFilter *>::iterator it = m_filters.begin();
         it != m_filters.end(); it++) {
        std::cout << "MM upate with observation" << std::endl;
        (*it)->updateWithObservation(visionBall);
    }
}

/**
 * @brief - update the filters predictions for where the ball will stop moving
 */
void MMKalmanFilter::updatePredictions()
{
    for (std::vector<KalmanFilter*>::iterator it = m_filters.begin();
         it != m_filters.end(); it++) {
        (*it)->predictBallDest();
    }
}

/**
 * @brief - Update the delta time from the system. Delta time is in Seconds
 *
 */
void MMKalmanFilter::updateDeltaTime()
{
    // Get time since last update
    const long long int time = monotonic_micro_time(); // from common
    m_delta_time = static_cast<float>(time - m_last_update_time)/
            1000000.0f; // u_s to sec

    // Guard against a zero dt (maybe possible?)
    if (m_delta_time <= 0.0) {
        m_delta_time = 0.0001f;
    }

    if (m_delta_time > 1)
        m_delta_time = .03f; // Guard against first frame issues

    m_last_update_time = time;
}

CartesianObservation MMKalmanFilter::calcVelocityOfBuffer()
{
    CartesianObservation calcVel;

    float sumVelX = 0;
    float sumVelY = 0;
    for (int i=1; i < m_params.bufferSize; i++) {
        sumVelX += (m_obsv_buffer[i].relX - m_obsv_buffer[i-1].relX) / m_delta_time;
        sumVelY += (m_obsv_buffer[i].relY - m_obsv_buffer[i-1].relY) / m_delta_time;
    }

    calcVel.relX = sumVelX / (float)m_params.bufferSize;
    calcVel.relY = sumVelY / (float)m_params.bufferSize;

    // SANITY CHECKS
    // Major Concern with calculating from a large history is we don't want our
    // calculation to be watered down by observations when the ball was stationary
    // So check if there is a drastic inconsistency (off by 100 cm/s) in the speed
    bool consistent = true;
    float estSpeed = calcSpeed(calcVel.relX, calcVel.relY);

    // Also check that we're getting these values from relatively close balls
    //    ie within 300 centimeters
    float dist = calcSpeed(m_obsv_buffer[m_cur_entry].relX, m_obsv_buffer[m_cur_entry].relY);
    if (dist > 300.f) {
        consistent = false;
    }

    for (int i=1; i < m_params.bufferSize && consistent; i++) {
        // current speed
        float curSpeed = calcSpeed((m_obsv_buffer[i].relX - m_obsv_buffer[i-1].relX)/m_delta_time,
                (m_obsv_buffer[i].relY - m_obsv_buffer[i-1].relY)/m_delta_time);

        if (diff(curSpeed,estSpeed) > 100) {
            consistent = false;
        }
    }

    if (consistent) {
        return calcVel;
    } else { //wasnt consistent so return no velocity
        return CartesianObservation(0.f,0.f);
    }
}

float MMKalmanFilter::diff(float a, float b)
{
    return std::abs(std::abs(a) - std::abs(b));
}

float MMKalmanFilter::calcSpeed(float a, float b)
{
    return std::sqrt(a*a + b*b);
}

} // balltrack
} // man
