//from T-nor
#pragma once

#define TIME_TO_REACH_MAX(t) (0.02 / (t)) // from time to reach max to rate limiter

class RateLimiter
{

public:
  RateLimiter() : m_target(0.0), m_current(0.0), m_speedUp(0.0), m_speedDown(0.0){};

  double Update(double target);                            // redefini la target et Update la valeur de current
  double Update();                                         // Update la valeur de current
  void SetTarget(double target);                           // Redefini la valeur de la target
  void SetCurrent(double current);                         // "force" la valeur de current
  void SetSpeed(double speed);                             // defini la vitesse
  void SetSpeed(double speedUp, double speedDown);         // defini la vitesse d'augmentation et de réduction
  void Reset(double target, double current, double speed); // Reset tout avec des valeurs que l'on choisit je sais pas si c'est ce que tu pensais
  void Reset(double target, double current, double speedUp, double speedDown);
                                                           //   void SetRecul(double current, double speed_1, double speed_2); // Set le recul

  double m_target;
  double m_current;
  double m_speedUp;
  double m_speedDown;

private:
};