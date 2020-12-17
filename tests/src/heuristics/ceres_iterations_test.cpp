#include <stdio.h>
#include <cstdint>
#include <iostream>
#include "ImageBuffer.h"
#include "visualizer.h"
#include "Solver.h"
#include "util.h"
#include <X11/Xlib.h> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <fstream>

/**
 * @brief test program to find convergence rate with respect to initial perturbation from perfect 
 * and number of ceres solver iterations at each step of the overall solution.
 * Given a perfect initialization, the solver is run with 5 levels of initial perturbation:
 * level  translation (% of largest translation in true transform)  rotation (degrees)
 *     1                                                       0-5                 0-2
 *     2                                                      5-10                 2-5
 *     3                                                     10-15                 5-8
 *     4                                                     15-20                8-10
 *     5                                                     20-25               10-12
 * for each level of initial perturbation, the solver is run on 100 randomly generated initial 
 * perturbations within the level bounds.
 * This is repeated with 5, 10, 15, 20, 25, 30, 35, 40, 45, 50 ceres iterations configured for the solution.
 * Results are written to a file specified. 
 * It is recommended to run this with visualization disabled in the solver 
 */

const uint8_t NUM_PERTURBATIONS = 100;

void setPerturbations (double (&perturbation_set_)[5][NUM_PERTURBATIONS][6], double max_translation_);

void testOne (Eigen::Matrix4d perfect_init_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD_);

cam_cad::ImageBuffer ImageBuffer;
cam_cad::Util mainUtility;

ofstream fout; 


int main () {

    fout.open("/home/cameron/wkrpt300_images/testing/test_1_2.txt");

    fout << "Started... \n";

    //image and CAD data input block//

    bool read_success_camera = false, read_success_CAD = false; 

    std::vector<cam_cad::point> input_points_camera, input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);

    std::string camera_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/-3.000000_0.000000.json";
    std::string CAD_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/sim_CAD.json";
    std::cout << camera_file_location << std::endl;
    std::cout << CAD_file_location << std::endl;

    read_success_camera = ImageBuffer.readPoints(camera_file_location, &input_points_camera); 

    if (read_success_camera) fout << "camera data read success\n";

    read_success_CAD = ImageBuffer.readPoints(CAD_file_location, &input_points_CAD);

    if (read_success_CAD) fout << "CAD data read success\n";

    //*******************************//

    //input cloud operations*********//

    ImageBuffer.densifyPoints(&input_points_camera, 10);
    ImageBuffer.densifyPoints(&input_points_CAD, 10);

    //ImageBuffer.scalePoints(&input_points_CAD, 0.01);

    fout << "points scaled \n";

    ImageBuffer.populateCloud(&input_points_camera, input_cloud_camera, 0);
    ImageBuffer.populateCloud(&input_points_CAD, input_cloud_CAD, 0);

    fout << "clouds populated \n";

    
    mainUtility.originCloudxy(input_cloud_CAD);

    //Perfect inits for each test pose********//
    // Perfect init # 1 (-1,-1)
    Eigen::Matrix4d perfect_init_one;
    perfect_init_one <<     0.999717,   0.00255823,    0.0236701,    -0.843579,
                        -0.000201718,     0.995085,   -0.0990278,     -1.32106,
                            -0.0238071,    0.0989949,     0.994803,      9.63384,
                            0,            0,            0,            1;

    // Perfect init # 2 (-3,0)
    Eigen::Matrix4d perfect_init_two;
    perfect_init_two <<       0.999889,  0.00625994,   0.0135535,    0.200217,
                            -0.00447353,     0.99176,   -0.128035,    -1.33746,
                            -0.0142433,     0.12796,    0.991677,     12.2632,
                                    0,           0,           0,           1;


    // call with perfect intialization for the initial pose
    testOne(perfect_init_two, input_cloud_camera, input_cloud_CAD);

    fout.close();

    return 0;
}

void testOne (Eigen::Matrix4d perfect_init_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD_) {
    
    //check initial pose with default solution settings 
    std::shared_ptr<cam_cad::Util> solverUtility (new cam_cad::Util);
    std::shared_ptr<cam_cad::Visualizer> solverVisualizer (new cam_cad::Visualizer ("solution visualizer"));
    std::string config_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/SolutionParameters.json";

    cam_cad::Solver solver(solverVisualizer, solverUtility, config_file_location);

    solver.LoadInitialPose(perfect_init_);

    bool convergence = solver.SolveOptimization(input_cloud_CAD_, input_cloud_camera_);

    bool good_init = false;

    if (convergence) {
        Eigen::Matrix4d T_CS_final = solver.GetTransform();

        if (mainUtility.RoundMatrix(perfect_init_, 1) == mainUtility.RoundMatrix(T_CS_final, 1))
            good_init = true;

    } 
    
    if (!good_init){
        fout << "TEST ONE Failed: bad perfect initialization\n";
        return;
    }
    
    // set perturbations based on the perfect init

    double max_initial_translation = 0;

    if (perfect_init_(0,3) > max_initial_translation) max_initial_translation = perfect_init_(0,3); // x translation
    if (perfect_init_(1,3) > max_initial_translation) max_initial_translation = perfect_init_(1,3); // y translation
    if (perfect_init_(2,3) > max_initial_translation) max_initial_translation = perfect_init_(2,3); // z_translation

    // perturbation size > random perturbation > perturbation components 
    double perturbation_set[5][NUM_PERTURBATIONS][6];

    setPerturbations(perturbation_set, max_initial_translation);

    // run solver with different max ceres iteration limits for each perturbation set 

    fout << "\n\nTEST ONE RESULTS: \n";

    fout << "\ninital pose: (-3, 0)\n";

    uint16_t max_ceres_iterations[10] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50};

    // ceres iterations
    for (uint8_t ceres_init = 0; ceres_init < 10; ceres_init ++) {
        fout << "\nsolving with " << max_ceres_iterations[ceres_init] << " max ceres iterations: \n";
        fout << "-------------------------------------\n";

        // perturbation level
        for(uint8_t level = 0; level < 5; level ++) {
            fout << "level " << level << "\n";

            double num_succeeded = 0;
            double avg_sol_iterations = 0;

            // perturbation
            for (uint8_t perturbation_i = 0; perturbation_i < NUM_PERTURBATIONS; perturbation_i++) {
                Eigen::Matrix4d init_T = perfect_init_;

                Eigen::VectorXd perturbation(6, 1);
                perturbation << perturbation_set[level][perturbation_i][3], 0, 0, 0, 0, 0; 
                init_T = mainUtility.PerturbTransformDegM(init_T, perturbation); 
                perturbation << 0, perturbation_set[level][perturbation_i][4], 0, 0, 0, 0;
                init_T = mainUtility.PerturbTransformDegM(init_T, perturbation); 
                perturbation << 0, 0, perturbation_set[level][perturbation_i][4], 0, 0, 0;
                init_T = mainUtility.PerturbTransformDegM(init_T, perturbation); 
                perturbation << 0, 0, 0, perturbation_set[level][perturbation_i][0], 
                                         perturbation_set[level][perturbation_i][1], 
                                         perturbation_set[level][perturbation_i][2];
                init_T = mainUtility.PerturbTransformDegM(init_T, perturbation); 

                std::shared_ptr<cam_cad::Util> solverUtility_i (new cam_cad::Util);
                std::shared_ptr<cam_cad::Visualizer> solverVisualizer_i (new cam_cad::Visualizer ("solution visualizer"));

                cam_cad::Solver solver_i(solverVisualizer_i, solverUtility_i, config_file_location);

                solver_i.LoadInitialPose(init_T);

                solver_i.SetMaxMinimizerIterations(max_ceres_iterations[ceres_init]);

                bool convergence_i = solver_i.SolveOptimization(input_cloud_CAD_, input_cloud_camera_);

                if (convergence_i) {
                    num_succeeded ++; 
                    avg_sol_iterations += solver_i.GetSolutionIterations();
                }
            }

            avg_sol_iterations /= num_succeeded;

            fout << "percent of solutions completed successfully: " << num_succeeded << "\n";
            fout << "average solver iterations for successful solutions: " << avg_sol_iterations << "\n";

        }
    }

}

void setPerturbations (double (&perturbation_set_)[5][NUM_PERTURBATIONS][6], double max_translation_) {
    
    double min_translations_level[5] = {0, 
                                        0.051*max_translation_, 
                                        0.101*max_translation_,
                                        0.151*max_translation_,
                                        0.201*max_translation_};

    double max_translations_level[5] = {0.05, 
                                        0.10*max_translation_, 
                                        0.15*max_translation_,
                                        0.20*max_translation_,
                                        0.25*max_translation_};

    double min_rotations_level[5] = {0, 2.1, 5.1, 8.1, 10.1};

    double max_rotations_level[5] = {2, 5, 8, 10, 12};

    // generate random perturbations

    // Random seed
    std::random_device rd;

    // Initialize Mersenne Twister pseudo-random number generator
    std::mt19937 gen(rd());

    // levels 
    for (uint8_t level = 0; level < 5; level++) {

        double lower_trans_bound = min_translations_level[level];
        double upper_trans_bound = max_translations_level[level];

        double lower_rotation_bound = min_rotations_level[level];
        double upper_rotation_bound = max_rotations_level[level];

        // perturbations
        for (uint8_t perturbation = 0; perturbation < NUM_PERTURBATIONS; perturbation ++) {

            // translations
            for (uint8_t trans = 0; trans < 3; trans ++) {
                std::uniform_real_distribution<double> unif(lower_trans_bound, upper_trans_bound);
                perturbation_set_[level][perturbation][trans] = upper_trans_bound/2 - unif(gen);
            }

            // rotations
            for (uint8_t rot = 3; rot < 6; rot ++) {
                std::uniform_real_distribution<double> unif(lower_rotation_bound, upper_rotation_bound);
                std::default_random_engine re;
                perturbation_set_[level][perturbation][rot] = upper_rotation_bound/2 - unif(gen);
            }

        }
    }

}

