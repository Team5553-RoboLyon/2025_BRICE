#include <lib/rate_limiter.h>


double RateLimiter::Update(double target) { // on update le current
   m_target = target; 

   if(m_current < m_target - m_speedUp) // si le current est plus petit que le target - la vitesse
   {
        m_current += m_speedUp; // on ajoute la vitesse au current
   }
   else if (m_current > m_target + m_speedDown) // si le current est plus grand que le target + la vitesse
   {
        m_current -= m_speedDown; // on enleve la vitesse au current
   }
   else
   {
        m_current = m_target; // sinon le current est egal au target
   }      
    return m_current; // on retourne le current
}

double RateLimiter::Update() {
   if(m_current < m_target - m_speedUp) // si le current est plus petit que le target - la vitesse
   {
        m_current += m_speedUp; // on ajoute la vitesse au current
   }
   else if (m_current > m_target + m_speedDown) // si le current est plus grand que le target + la vitesse
   {
        m_current -= m_speedDown; // on enleve la vitesse au current
   }
   else
   {
        m_current = m_target; // sinon le current est egal au target
   }      
    return m_current; // on retourne le current
}
void RateLimiter::SetTarget(double target) { // on set le target
  m_target = target; 
}

void RateLimiter::SetCurrent(double current) { // on set le current
  m_current = current;
}

void RateLimiter::SetSpeed (double speed) { // on set la vitesse
  m_speedDown = speed;
  m_speedUp = speed;
}
void RateLimiter::SetSpeed(double speedUp, double speedDown) {
     m_speedUp = speedUp;
     m_speedDown = speedDown;
}

void RateLimiter::Reset(double target, double current, double speed) { // on reset tout
  m_target = target;
  m_current = current;
  m_speedUp = speed;
  m_speedDown = speed;
}
 
void RateLimiter::Reset(double target, double current, double speedUp, double speedDown) {
     m_target = target;
     m_current = current;
     m_speedDown = speedDown;
     m_speedUp = speedUp;
}