#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd RMSE(4);
  if (estimations.size() != ground_truth.size() || estimations.size() == 0 ){
     std::cout << "Invalid estimation or ground_truth data" << std::endl;
     return RMSE;
  }
  // calculate sum
   for(int i = 0;i < estimations.size();i++){
      VectorXd residual = estimations[i] - ground_truth[i];
      VectorXd residual_sq = residual.array() * residual.array();
      RMSE += residual_sq; 
   }

   //calculate mean
   RMSE = RMSE / estimations.size();

   //calculate square

   RMSE = RMSE.array().square();
   
   
   return RMSE;


}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px * px + py * py ;
  float c2 = sqrt(c1);

   Hj << px / c2 , py / c2 , 0 , 0 ,
         -py / c1 , px / c1 , 0 , 0 ,
         (py * ((vx * py) - (vy * px))) / (c1 * c2),(px *((vy * px) - (vx * py))) / (c1 * c2) , px/c2 ,py / c2;


   return Hj;




}
