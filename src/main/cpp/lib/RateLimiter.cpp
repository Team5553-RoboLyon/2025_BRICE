#include <lib/RateLimiter.h>
#include <lib/DebugUtils.h>

RateLimiter::RateLimiter() 
    : m_increasingRateLimit(0.0),
    m_brakingRateLimit(0.0), 
    m_currentSpeed(0.0), 
    m_targetSpeed(0.0) 
{}
RateLimiter::RateLimiter(double timeToReachMax) 
    : m_currentSpeed(0.0), 
    m_targetSpeed(0.0) 
{
    SetRateLimit(timeToReachMax);
}

RateLimiter::RateLimiter(double timeToReachMaxUp, double timeToReachMaxDown) 
    : m_currentSpeed(0.0), 
    m_targetSpeed(0.0) 
{
    SetRateLimit(timeToReachMaxUp, timeToReachMaxDown);
}

void RateLimiter::SetDeltaTime(double deltaTime) 
{
    //Save the current timeToReachMax values based on the past dt
    double timeToReachMaxUp = m_dt / m_increasingRateLimit;
    double timeToReachMaxDown = m_dt / m_brakingRateLimit;

    DEBUG_ASSERT(deltaTime > 0.0, "Delta time must be positive");
    if(deltaTime > 0.0) 
    {
        m_dt = deltaTime;
    }

    SetRateLimit(timeToReachMaxUp, timeToReachMaxDown); // Update rate limits based on new dt
}

void RateLimiter::SetRateLimit(double timeToReachMax) 
{
    DEBUG_ASSERT(timeToReachMax > 0.0, "Time to reach max must be positive");
    if(timeToReachMax > 0.0) 
    {
        m_increasingRateLimit = m_dt / timeToReachMax; // Convert time to rate limit
        m_brakingRateLimit = m_increasingRateLimit; // Same for both directions
    } 
    else 
    {
        ERROR_LOG("RateLimiter: invalid timeToReachMax <= 0. Defaulting to 0.");
        m_increasingRateLimit = 0.0;
        m_brakingRateLimit = 0.0;
    }
}

void RateLimiter::SetRateLimit(double timeToReachMaxUp, double timeToReachMaxDown) 
{
    DEBUG_ASSERT(timeToReachMaxUp > 0.0 && timeToReachMaxDown > 0.0, "Time to reach max must be positive");
    if(timeToReachMaxUp > 0.0 && timeToReachMaxDown > 0.0) 
    {
        m_increasingRateLimit = m_dt / timeToReachMaxUp; // Convert time to rate limit
        m_brakingRateLimit = m_dt / timeToReachMaxDown; // Convert time to rate limit
    } 
    else 
    {
        ERROR_LOG("RateLimiter: invalid timeToReachMax <= 0. Defaulting to 0.");
        m_increasingRateLimit = 0.0;
        m_brakingRateLimit = 0.0;
    }
}

void RateLimiter::SetTarget(double target) 
{
    m_targetSpeed = target;
}

void RateLimiter::SetCurrent(double current) 
{
    m_currentSpeed = current;
}

double RateLimiter::GetCurrentSpeed() const 
{
    return m_currentSpeed; 
}
double RateLimiter::GetTargetSpeed() const 
{
    return m_targetSpeed;
}
double RateLimiter::GetRateLimitUp() const 
{
    return m_increasingRateLimit; 
}
double RateLimiter::GetRateLimitDown() const 
{
    return m_brakingRateLimit; 
}
double RateLimiter::GetDeltaTime() const 
{
    return m_dt; 
}
std::string RateLimiter::GetState() const 
{
    return "RateLimiter State: {"
           "Current Speed: " + std::to_string(m_currentSpeed) + ", "
           "Target Speed: " + std::to_string(m_targetSpeed) + ", "
           "Rate Limit Up: " + std::to_string(m_increasingRateLimit) + ", "
           "Rate Limit Down: " + std::to_string(m_brakingRateLimit) + ", "
           "Delta Time: " + std::to_string(m_dt) + "}";
}


double RateLimiter::Update()
{
    double delta = m_targetSpeed - m_currentSpeed;

    if(delta > 0.0)
    {
        m_currentSpeed += NMIN(delta, m_increasingRateLimit);
    }
    else if (delta < 0) 
    {
        // Are we braking ?
        if (m_targetSpeed * m_currentSpeed > 0) 
        {
            // Same sign → we brake (in the same direction)
            m_currentSpeed += NMAX(delta, -m_brakingRateLimit);
        } 
        else 
        {
            // Direction changing → we increase in the opposite direction
            m_currentSpeed += NMAX(delta, -m_increasingRateLimit);
        }
    }
    // else delta == 0 : we're at target
    else 
    {
        m_currentSpeed = m_targetSpeed;
    }
    return m_currentSpeed; // Return the current speed
}

double RateLimiter::Update(double target) 
{
    m_targetSpeed = target; // Set the target speed
    return Update();
}

void RateLimiter::Reset() 
{
    m_currentSpeed = 0.0;
    m_targetSpeed = 0.0;
}

void RateLimiter::Reset(double target, double current) 
{
    m_targetSpeed = target;
    m_currentSpeed = current;
}