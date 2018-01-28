/*
Created on: December, 2017
Author(s): Djordje Vukcevic, Pradheep Padmanabhan
Copyright (c) [2017]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <cmath>
#include <stdlib.h>     /* abs */
#include <Eigen/Dense>
#include <awt_trajectory_generator.h>

namespace traj_gen
{

    AWT_Trajectory_Generator::AWT_Trajectory_Generator()
    {
    }

    AWT_Trajectory_Generator::~AWT_Trajectory_Generator()
    {
    }

    int AWT_Trajectory_Generator::sign(double x){
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }

    void AWT_Trajectory_Generator::generate_sine_wave(
        const double phase,
        const double amplitude,
        const int num_steps,
        Eigen::MatrixXd &sine_wave,
        double &step_size_x)
        {

        Eigen::VectorXf sine_angles;
        sine_angles.setLinSpaced(num_steps, 0, 2 * M_PI);
        step_size_x = sine_angles(2) - sine_angles(1);
        sine_wave = Eigen::MatrixXd::Zero(num_steps,2);

        for (int i = 0; i < num_steps; i++){
            sine_wave(i,0) = sine_angles(i);
            sine_wave(i,1) = amplitude * sin(sine_angles(i) + phase);
        }
    }

    void AWT_Trajectory_Generator::generate_square_wave(
        const double phase,
        const double amplitude,
        const int num_steps,
        Eigen::MatrixXd &square_wave,
        double &step_size_x)
        {

        Eigen::ArrayXd square_angles = Eigen::ArrayXd::LinSpaced(num_steps, 0, 2 * M_PI);
        step_size_x = square_angles(2) - square_angles(1);
        square_wave = Eigen::MatrixXd::Zero(num_steps,2);

        for (int i = 0; i < num_steps; i++){
            square_wave(i,0) = square_angles(i);
            square_wave(i,1) = amplitude * sign( sin(square_angles(i) + phase) );
        }
    }

    void AWT_Trajectory_Generator::generate_velocity(
        const Eigen::MatrixXd sine_wave,
        const double step_size_x,
        const double time_step,
        Eigen::MatrixXd &sine_velocity)
        {

        sine_velocity = Eigen::MatrixXd::Zero(sine_wave.rows(),2);

        for(int i = 0; i < sine_wave.rows() - 1; i++){
            sine_velocity(i,0) = step_size_x/time_step;
            sine_velocity(i,1) = (sine_wave(i + 1, 1) - sine_wave(i, 1))/time_step;
        }
    }

    void AWT_Trajectory_Generator::generate_trajectory(
        const std::string type,
        const double phase,
        const double amplitude,
        const int num_steps,
        const double time_step,
        Eigen::MatrixXd &velocity)
        {

        Eigen::MatrixXd wave;
        double step_size_x;

        if (type == "sine"){
            generate_sine_wave(
                        phase,
                        amplitude,
                        num_steps,
                        wave,
                        step_size_x);
        }

        else if (type == "square" ){
            generate_square_wave(
                        phase,
                        amplitude,
                        num_steps,
                        wave,
                        step_size_x);
        }

        else {
            fprintf(stderr,
                "ERROR: Undefined type of trajectory: Choose between sine and square.");
        }

        generate_velocity(wave,
                        step_size_x,
                        time_step,
                        velocity);
    }

} /* namespace traj_gen */
