/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   9
 * ENTRENAMIENTO DE UN PERCEPTRON POR DESCENSO DEL GRADIENTE
 */

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "random_numbers/random_numbers.h"
#include <cmath>

#define NOMBRE "APELLIDO_PATERNO_APELLIDO_MATERNO"

class Perceptron
{
public:
    Perceptron(int inputs);
    ~Perceptron();

    std::vector<float> w;
    
    float evaluate(float* data);
};

Perceptron::Perceptron(int inputs)
{
    w.resize(inputs);
    for(int i=0; i < w.size()+1; i++)
        w[i] = 0;
}

Perceptron::~Perceptron(){}

float Perceptron::evaluate(float* data)
{
    float sum = w[0];
    for(int i = 1;i < w.size();i++){
        sum += w[i]*data[i-1];
    }
    return 1.0/(1.0 + std::exp(-sum));
}

void load_training_data(std::string folder, std::vector<cv::Mat>& training_images, std::vector<int>& training_labels)
{
    std::string   training_images_file = folder + "/train-images.idx3-ubyte";
    std::string   training_labels_file = folder + "/train-labels.idx1-ubyte";
    std::ifstream ifs_training_images(training_images_file.c_str(), std::ios::binary|std::ios::ate);
    std::ifstream ifs_training_labels(training_labels_file.c_str(), std::ios::binary|std::ios::ate);
    int images_file_size = (int)ifs_training_images.tellg();
    int labels_file_size = (int)ifs_training_labels.tellg();
    char* images_buffer = (char*)malloc(images_file_size);
    char* labels_buffer = (char*)malloc(labels_file_size);
    ifs_training_images.seekg(0, std::ios::beg);
    ifs_training_labels.seekg(0, std::ios::beg);
    ifs_training_images.read(images_buffer, images_file_size);
    ifs_training_labels.read(labels_buffer, labels_file_size);
    ifs_training_images.close();
    ifs_training_labels.close();

    //The number of images and the size of each image is contained in the file but it is assumed to have
    //the values mentioned in http://yann.lecun.com/exdb/mnist/
    int no_of_images = 60000;
    int img_rows     = 28;
    int img_cols     = 28;

    training_images.resize(no_of_images);
    training_labels.resize(no_of_images);
    for(int i=0; i < no_of_images; i++)
    {
        training_labels[i] = (int)labels_buffer[i + 8];
        training_images[i] = cv::Mat(img_rows, img_cols, CV_8UC1);
        for(int j=0; j < img_rows*img_cols; j++)
            training_images[i].data[j] = images_buffer[i*img_rows*img_cols + j + 16];
        training_images[i].convertTo(training_images[i], CV_32F, 1.f/255);
        
    }
    return;
}

void load_testing_data(std::string folder, std::vector<cv::Mat>& testing_images, std::vector<int>& testing_labels)
{
    std::string   testing_images_file = folder + "/t10k-images.idx3-ubyte";
    std::string   testing_labels_file = folder + "/t10k-labels.idx1-ubyte";
    std::ifstream ifs_testing_images(testing_images_file.c_str(), std::ios::binary|std::ios::ate);
    std::ifstream ifs_testing_labels(testing_labels_file.c_str(), std::ios::binary|std::ios::ate);
    int images_file_size = (int)ifs_testing_images.tellg();
    int labels_file_size = (int)ifs_testing_labels.tellg();
    char* images_buffer = (char*)malloc(images_file_size);
    char* labels_buffer = (char*)malloc(labels_file_size);
    ifs_testing_images.seekg(0, std::ios::beg);
    ifs_testing_labels.seekg(0, std::ios::beg);
    ifs_testing_images.read(images_buffer, images_file_size);
    ifs_testing_labels.read(labels_buffer, labels_file_size);
    ifs_testing_images.close();
    ifs_testing_labels.close();

    //The number of images and the size of each image is contained in the file but it is assumed to have
    //the values mentioned in http://yann.lecun.com/exdb/mnist/
    int no_of_images = 10000;
    int img_rows     = 28;
    int img_cols     = 28;

    testing_images.resize(no_of_images);
    testing_labels.resize(no_of_images);
    for(int i=0; i < no_of_images; i++)
    {
        testing_labels[i] = (int)labels_buffer[i + 8];
        testing_images[i] = cv::Mat(img_rows, img_cols, CV_8UC1);
        for(int j=0; j < img_rows*img_cols; j++)
            testing_images[i].data[j] = images_buffer[i*img_rows*img_cols + j + 16];
        testing_images[i].convertTo(testing_images[i], CV_32F, 1.f/255);
    }
    return;
}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 09 - ENTRENAMIENTO DE UN PERCEPTRON - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_09");
    ros::NodeHandle n("~");
    ros::Rate loop(10);

    /*
     * Important variables:
     * The three of them are received as command line parameters
     * digit_to_train: Corresponds to the digit to be trained.
     * delta: Is the 'gain' by wich the gradient is multiplied to change the weight values.
     * epochs: Maximum number of times the full data set is passed.
     */
    std::string folder = "";
    int   digit_to_train = 0;
    float delta = 1.0;
    int   max_epochs = 10;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            folder = argv[++i];
        if(strParam.compare("-c") == 0)
            digit_to_train = atoi(argv[++i]);
        if(strParam.compare("-d") == 0)
            delta = atof(argv[++i]);
        if(strParam.compare("-e") == 0)
            max_epochs = atoi(argv[++i]);
    }
    
    std::vector<cv::Mat> training_images;
    std::vector<int>     training_labels;
    std::vector<cv::Mat> testing_images;
    std::vector<int>     testing_labels;

    std::cout << "Loading dataset from folder " << folder << std::endl;
    load_training_data(folder, training_images, training_labels);
    load_testing_data(folder, testing_images, testing_labels);

    std::cout << "Loaded  " << training_images.size() << " training images."  << std::endl;
    std::cout << "Loaded  " << testing_images.size() << " testing images."  << std::endl;

    // We declare a new perceptron whose weights are initialized all to zero (see constructor's code)
    // The parameter indicates the number of input signals. 
    Perceptron p(28*28);
    
    std::cout << "Training for digit: " << digit_to_train << std::endl;
    
    float gradient;
    int epochs = 0;
    float avg_grad = INFINITY;
    while(avg_grad > 0.01 && epochs < max_epochs){
         avg_grad = 0.0;
        for(int i = 0; i < training_labels.size(); i++){
            float* mat = reinterpret_cast<float*>(training_images[i].ptr());
            float est_y = p.evaluate(mat);
            
            float des_y = training_labels[i] == digit_to_train? 1.0 : 0.0;
            float sens = (des_y - est_y)*est_y*(1.0-est_y);
            p.w[0] = delta*sens;
            gradient = 0.0;
            //std::cout <<"est:"<<est_y<< " dest:"<<des_y << " sens:" << sens << std::endl;
            for(int j = 1; j < p.w.size(); j ++){
                p.w[j] += delta*sens*mat[j];
                gradient += std::abs(sens*mat[j]);
            }
            avg_grad += gradient;
        }
        std::cout << "Finished cycle " << epochs << " avg gradient:" << avg_grad/training_labels.size()<<std::endl;
        epochs++;
    }
    std::cout << "Training done in " << epochs << "/" << max_epochs << " cycles\n";
    std::cout << "Gradient norm: " << avg_grad << std::endl;
    
    /*
     * TODO:
     * Write the code to train the perceptron using the gradient descend method.
     *
     * WHILE Gradient magnitud < tolerance AND epochs < max_epochs:
     *    FORALL image in training data set:
     *       estimated_y = perceptron's output for the image
     *       desired_output = 1, if the label of the image corresponds to the digit to be trained, 0, otherwise.
     *       FORALL weight in perceptron's weights:
     *          weight += delta * gradient_i (calculated using the gradient's rule)
     *       threshold += delta * gradient_t (similar to gradient_i but considering the input as 1)
     *    Calculate gradient's magnitude
     */
    
    

    //Once the perceptron is trained, we proceed to test it .
    //We choose a random image of the testing set. The image is displayed and the output of the
    //Perceptron is calculated.
    char cmd = 0;
    random_numbers::RandomNumberGenerator rng;
    int idx = 0;
    while(ros::ok() && (cmd = cv::waitKey(15)) != 27)
    {
        if(cmd == 13)
        {
            idx = rng.uniformInteger(0, testing_images.size()-1);
            float result = p.evaluate((float*)testing_images[idx].data);
            std::string success = (digit_to_train == testing_labels[idx]) == (result > 0.5) ? "Correct!!!" : "Wrong";
            std::cout<<"Label: "<<testing_labels[idx]<< "\tOutput: " << result << "\tClassification: " << success << std::endl;
        }
        cv::imshow("Testing", testing_images[idx]);
        ros::spinOnce();
        loop.sleep();
    }
}
