/*
 *  Created on: December, 2017
 *      Authors: Djordje Vukcevic, Pradheep Krishna Muthukrishnan Padmanabhan
 */
#ifndef AWTTRAJECTORYGENERATOR_H_
#define AWTTRAJECTORYGENERATOR_H_

#include <Eigen/Dense>

namespace traj_gen
{

class AWT_Trajectory_Generator
{
    protected:
        int sign(double x);
        Eigen::ArrayXd sine_angles;
        Eigen::ArrayXd square_angles;

    public:
        AWT_Trajectory_Generator();

        virtual ~AWT_Trajectory_Generator();

        void generate_sine_wave(
            const double phase,
            const double amplitude,
            const int num_steps,
            Eigen::MatrixXd &sine_wave,
            double &step_size_x);

        void generate_square_wave(
            const double phase,
            const double amplitude,
            const int num_steps,
            Eigen::MatrixXd &square_wave,
            double &step_size_x);

        void generate_velocity(
            const Eigen::MatrixXd sine_wave,
            const double step_size_x,
            const double time_step,
            Eigen::MatrixXd &velocity);

        void generate_trajectory(
            const std::string type,
            const double phase,
            const double amplitude,
            const int num_steps,
            const double time_step,
            Eigen::MatrixXd &velocity);
    };

} /* namespace traj_gen */
#endif
