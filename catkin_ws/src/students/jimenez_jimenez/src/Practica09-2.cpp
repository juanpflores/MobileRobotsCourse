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

#define NOMBRE "EGUIARTE_MORETT"
 
class Perceptron
{
public:
    Perceptron(int, int, int, float, float);
    ~Perceptron();

    std::vector<float> w;
    float theta, tolerance, delta;
    int inputs, max_epochs, digit_to_train;
    
    float evaluate(float*);
    void train(std::vector<cv::Mat>, std::vector<int>);
};

Perceptron::Perceptron(int inputs, int max_epochs, int digit_to_train, float tolerance, float delta)
{
    w.resize(inputs);
    for(int i=0; i < w.size(); i++)
        w[i] = 0;
    theta = 0;
    this->max_epochs = max_epochs;
    this->digit_to_train = digit_to_train;
    this->tolerance = tolerance;
    this->delta = delta;
}

Perceptron::~Perceptron(){}

float Perceptron::evaluate(float *data)
{
    float output = 0;
    /*
     * TODO:
     * Calculate the output of the perceptron using a sigmoid function as an activation function.
     * Assume that the number of inputs in '*data' corresponds to the number of weights 'w.size()'
     * You can assume you are receiving the correct number of data.
     */
    for (int i = 0; i < w.size(); i++){
        output += w[i]*data[i];
    }
    return (1.0f/(1.0f + exp(-output + theta)));
}

void Perceptron::train(std::vector<cv::Mat> training_images, std::vector<int>training_labels){
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
    int epoch = 0;
    float gradient_mag;
    std::vector<float> gradient;
    std::vector<float> desired_output;
    gradient.resize(w.size());
    desired_output.resize(training_labels.size());
    for(int i = 0; i < training_labels.size(); ++i){
        desired_output[i] = training_labels[i] == this->digit_to_train ? 1.0f : 0.0f;
    }
    
    do{ 
        gradient_mag = 0;
        for(int i = 0; i < training_images.size(); ++i){
           
            float estimated_y = this->evaluate((float*)training_images[i].data);
            float grad = 0;
            for(int j = 0; j < training_images.size(); ++j){
                grad += (estimated_y - desired_output[j])*(estimated_y*(1-estimated_y));
            }
            //grad/=training_images.size();
            //std::cout<< gradient_mag << std::endl;
            for(int j = 0; j < w.size(); j++){
                w[j] -= this->delta * grad * (float)training_images[i].data[j];
                gradient_mag+=grad * (float)training_images[i].data[j];
            }
            theta -= this->delta * grad;
        }
        epoch++;
        std::cout<< epoch << std::endl;
        std::cout<< gradient_mag << std::endl;
         
    }while(abs(gradient_mag) > this->tolerance && epoch < this->max_epochs);
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
    Perceptron p(28*28, max_epochs, digit_to_train, 2000.0f, delta);
    
    std::cout << "Training for digit: " << digit_to_train << std::endl;
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
    p.train(training_images, training_labels);

    //Once the perceptron is trained, we proceed to test it.
    //We choose a random image of the testing set. The image is displayed and the output of the
    //Perceptron is calculated.
    char cmd = 0;
    random_numbers::RandomNumberGenerator rng;
    int idx = 0;
    while(ros::ok() && (cmd = cv::waitKey(15)) != 27)
    {
        
        
        if(cmd == 13)
        {
            std::cout<< "holi" << std::endl;
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
