#include <lib/RateLimiter.h>
#include <lib/DebugUtils.h>

RateLimiter::RateLimiter() 
    : m_rateLimitUp(0.0),
    m_rateLimitDown(0.0), 
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
    double timeToReachMaxUp = m_dt / m_rateLimitUp;
    double timeToReachMaxDown = m_dt / m_rateLimitDown;

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
        m_rateLimitUp = m_dt / timeToReachMax; // Convert time to rate limit
        m_rateLimitDown = m_rateLimitUp; // Same for both directions
    } 
    else 
    {
        ERROR_LOG("RateLimiter: invalid timeToReachMax <= 0. Defaulting to 0.");
        m_rateLimitUp = 0.0;
        m_rateLimitDown = 0.0;
    }
}

void RateLimiter::SetRateLimit(double timeToReachMaxUp, double timeToReachMaxDown) 
{
    DEBUG_ASSERT(timeToReachMaxUp > 0.0 && timeToReachMaxDown > 0.0, "Time to reach max must be positive");
    if(timeToReachMaxUp > 0.0 && timeToReachMaxDown > 0.0) 
    {
        m_rateLimitUp = m_dt / timeToReachMaxUp; // Convert time to rate limit
        m_rateLimitDown = m_dt / timeToReachMaxDown; // Convert time to rate limit
    } 
    else 
    {
        ERROR_LOG("RateLimiter: invalid timeToReachMax <= 0. Defaulting to 0.");
        m_rateLimitUp = 0.0;
        m_rateLimitDown = 0.0;
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
    return m_rateLimitUp; 
}
double RateLimiter::GetRateLimitDown() const 
{
    return m_rateLimitDown; 
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
           "Rate Limit Up: " + std::to_string(m_rateLimitUp) + ", "
           "Rate Limit Down: " + std::to_string(m_rateLimitDown) + ", "
           "Delta Time: " + std::to_string(m_dt) + "}";
}


double RateLimiter::Update()
{
    if(m_currentSpeed < m_targetSpeed - m_rateLimitUp) 
    { // If current speed is less than target - rate limit up
        m_currentSpeed += m_rateLimitUp; // Increase current speed by rate limit up
    } 
    else if (m_currentSpeed > m_targetSpeed + m_rateLimitDown)
    { // If current speed is greater than target + rate limit down
        m_currentSpeed -= m_rateLimitDown; // Decrease current speed by rate limit down
    } 
    else 
    {
        m_currentSpeed = m_targetSpeed; // Otherwise, set current speed to target speed
    }      
    
    return m_currentSpeed; // Return the current speed
}

double RateLimiter::Update(double target) 
{
    m_targetSpeed = target; // Set the target speed

    if(m_currentSpeed < m_targetSpeed - m_rateLimitUp) 
    { // If current speed is less than target - rate limit up
        m_currentSpeed += m_rateLimitUp; // Increase current speed by rate limit up
    } 
    else if (m_currentSpeed > m_targetSpeed + m_rateLimitDown) 
    { // If current speed is greater than target + rate limit down
        m_currentSpeed -= m_rateLimitDown; // Decrease current speed by rate limit down
    } 
    else 
    {
        m_currentSpeed = m_targetSpeed; // Otherwise, set current speed to target speed
    }      
    
    return m_currentSpeed; // Return the current speed
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