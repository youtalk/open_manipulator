/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../include/open_manipulator_libs/dynamics.h"

using namespace dynamics;



/*****************************************************************************
** Dynamics Solver Using Newton-Euler method
*****************************************************************************/

///////////////////////private Function/////////////////////////

bool SolverUsingNewtonEuler::setExterrnalEffort(std::unordered_map<Name, Eigen::VectorXd> external_effort)
{
  external_effort_ = external_effort;
}

////////////////////////////////////////////////////////////////


//////////////////////virtual Function//////////////////////////
bool SolverUsingNewtonEuler::setOption(STRING param_name, const void *arg){}

bool SolverUsingNewtonEuler::setEnvironments(STRING param_name, const void *arg)
{
  bool result = false;

  if (param_name == "external_effort")
  {
    std::unordered_map<Name, Eigen::VectorXd> *external_effort = arg;
    return setExterrnalEffort(*external_effort);
  }

  return false;
}

bool SolverUsingNewtonEuler::solveForwardDynamics(Manipulator *manipulator, std::vector<double> joint_torque)
{
  Eigen::Vector3d my_center_of_mass = Eigen::Vector3d::Zero();
  Eigen::Matrix3d my_inertia_tensor = Eigen::Matrix3d::Identity();



}

bool SolverUsingNewtonEuler::solveInverseDynamics(Manipulator manipulator, std::vector<double>* joint_torque){}
////////////////////////////////////////////////////////////////
