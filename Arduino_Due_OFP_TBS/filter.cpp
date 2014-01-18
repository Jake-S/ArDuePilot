#include <Math.h>


float low_pass(float u, float y_last, float tau, float dt, float max_delta)
{
  // Implements y/u = tau / (s + tau)
  float y;
  
    if(fabs(u - y_last) < max_delta)
      y = y_last + (tau*dt)/(1+tau*dt)*(u-y_last);
    else
      y = u;
    
  return(y);
}
