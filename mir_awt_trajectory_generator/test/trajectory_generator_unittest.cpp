/*
 *  Created on: December, 2017
 *      Authors: Djordje Vukcevic, Pradheep Krishna Muthukrishnan Padmanabhan
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <awt_trajectory_generator.h>

namespace traj_gen
{
    // Tests that 'generate_sine_wave' function returns the correct result.
    TEST(AWT_Trajectory_Generator, test_sine_wave_computation)
    {
        AWT_Trajectory_Generator generator;
        Eigen::MatrixXd actual_wave;
        double actual_step_size = 0;

        Eigen::MatrixXd expected_wave(5,2);
        expected_wave << 0.0000, 0.00000,
                        1.5707,   100.000,
                        3.1415,   0.00000087,
                        4.7123,   -100.00000,
                        6.28318,  -0.000017;

        double expected_step_size = 1.57079632679;

        generator.generate_sine_wave(0, 100.0, 5, actual_wave, actual_step_size);
        ASSERT_TRUE(actual_wave.isApprox(expected_wave, 0.001));
        EXPECT_NEAR(actual_step_size, expected_step_size, 0.001);
    }

    // Tests that 'generate_square_wave' function returns the correct result.
    TEST(AWT_Trajectory_Generator, test_square_wave_computation)
    {
        AWT_Trajectory_Generator generator;
        Eigen::MatrixXd actual_wave;
        double actual_step_size = 0;

        Eigen::MatrixXd expected_wave(10,2);
        expected_wave << 0.0,         100.0,
                         0.6981317,   100.0,
                         1.3962634,   100.0,
                         2.0943951,  -100.0,
                         2.7925268,  -100.0,
                         3.4906585,  -100.0,
                         4.1887902,  -100.0,
                         4.88692191,  100.0,
                         5.58505361,  100.0,
                         6.28318531,  100.0;

        double expected_step_size = 0.698131700798;

        generator.generate_square_wave(M_PI/2, 100.0, 10, actual_wave, actual_step_size);
        ASSERT_TRUE(actual_wave.isApprox(expected_wave, 0.001));
        EXPECT_NEAR(actual_step_size, expected_step_size, 0.001);
    }

    // Tests that 'generate_velocity' function returns the correct result.
    // For sine trajectory
    TEST(AWT_Trajectory_Generator, test_velocity_computation)
    {
        AWT_Trajectory_Generator generator;
        Eigen::MatrixXd actual_wave;
        Eigen::MatrixXd actual_velocity;
        double time_step = 0.01;
        double actual_step_size = 0;

        Eigen::MatrixXd expected_velocity(5,2);
        expected_velocity << 157.07963268,  10000.0,
                             157.07963268, -10000.0,
                             157.07963268, -10000.0,
                             157.07963268,  10000.0,
                             0.0,             0.0;

        generator.generate_sine_wave(0, 100.0, 5, actual_wave, actual_step_size);
        generator.generate_velocity(
                                    actual_wave,
                                    actual_step_size,
                                    time_step,
                                    actual_velocity);

        ASSERT_TRUE(actual_velocity.isApprox(expected_velocity, 0.001));
    }

    // Tests that 'generate_trajectory' function returns the correct result.
    // For square trajectory
    TEST(AWT_Trajectory_Generator, test_trajectory_computation)
    {
        AWT_Trajectory_Generator generator;
        Eigen::MatrixXd actual_velocity;
        double time_step = 0.5;

        Eigen::MatrixXd expected_velocity(10,2);
        expected_velocity << 1.3962634,   0.0,
                             1.3962634,   0.0,
                             1.3962634,  -400.0,
                             1.3962634,   0.0,
                             1.3962634,   0.0,
                             1.3962634,   0.0,
                             1.3962634,   400.0,
                             1.3962634,   0.0,
                             1.3962634,   0.0,
                             0.0,         0.0;

        generator.generate_trajectory("square",
                                    M_PI/2, 100.0, 10,
                                    time_step,
                                    actual_velocity);

        ASSERT_TRUE(actual_velocity.isApprox(expected_velocity, 0.001));
    }

    int main(int argc, char **argv)
    {
      testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
    }
} /* namespace traj_gen */
