#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <tiny_dnn/tiny_dnn.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;

#include <cstdlib>
#include <string>
#include <cstring>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;
// convert image to vec_t
void convert_image(const std::string& imagefilename,
                   double scale,
                   int w,
                   int h,
                   std::vector<vec_t>& data)
{
    auto img = cv::imread(imagefilename, cv::IMREAD_GRAYSCALE);
    if (img.data == nullptr) return; // cannot open, or it's not an image

    cv::Mat_<uint8_t> resized;
    cv::resize(img, resized, cv::Size(w, h));
    vec_t d;

    std::transform(resized.begin(), resized.end(), std::back_inserter(d),
                   [=](uint8_t c) { return c * scale; });
    data.push_back(d);
}

// convert all images found in directory to vec_t
void convert_images(const std::string& directory,
                    double scale,
                    int w,
                    int h,
                    std::vector<vec_t>& data)
{
    path dpath(directory);

    BOOST_FOREACH(const path& p, 
                  std::make_pair(directory_iterator(dpath), directory_iterator())) {
        if (is_directory(p)) continue;
        convert_image(p.string(), scale, w, h, data);
    }
}

static void construct_net(network<sequential>& nn) {
    // connection table [Y.Lecun, 1998 Table.1]
#define O true
#define X false
    static const bool tbl[] = {
        O, X, X, X, O, O, O, X, X, O, O, O, O, X, O, O,
        O, O, X, X, X, O, O, O, X, X, O, O, O, O, X, O,
        O, O, O, X, X, X, O, O, O, X, X, O, X, O, O, O,
        X, O, O, O, X, X, O, O, O, O, X, X, O, X, O, O,
        X, X, O, O, O, X, X, O, O, O, O, X, O, O, X, O,
        X, X, X, O, O, O, X, X, O, O, O, O, X, O, O, O
    };
#undef O
#undef X

    // by default will use backend_t::tiny_dnn unless you compiled
    // with -DUSE_AVX=ON and your device supports AVX intrinsics
    core::backend_t backend_type = core::default_engine();

    // construct nets
    //
    // C : convolution
    // S : sub-sampling
    // F : fully connected
    nn << convolutional_layer<tan_h>(32, 32, 5, 1, 6,  // C1, 1@32x32-in, 6@28x28-out
            padding::valid, true, 1, 1, backend_type)
       << average_pooling_layer<tan_h>(28, 28, 6, 2)   // S2, 6@28x28-in, 6@14x14-out
       << convolutional_layer<tan_h>(14, 14, 5, 6, 16, // C3, 6@14x14-in, 16@10x10-in
            connection_table(tbl, 6, 16),
            padding::valid, true, 1, 1, backend_type)
       << average_pooling_layer<tan_h>(10, 10, 16, 2)  // S4, 16@10x10-in, 16@5x5-out
       << convolutional_layer<tan_h>(5, 5, 5, 16, 120, // C5, 16@5x5-in, 120@1x1-out
            padding::valid, true, 1, 1, backend_type)
       << fully_connected_layer<tan_h>(120, 10,        // F6, 120-in, 10-out
            true, backend_type)
    ;
}

static void train_lenet(const std::string& data_dir_path) {
    // specify loss-function and learning strategy
    network<sequential> nn;
    adagrad optimizer;

    construct_net(nn);

    std::cout << "load models..." << std::endl;

    // load MNIST dataset
    std::vector<label_t> train_labels, test_labels;
    std::vector<vec_t> train_images, test_images;

    parse_mnist_labels(data_dir_path + "/train-labels.idx1-ubyte",
                       &train_labels);
    parse_mnist_images(data_dir_path + "/train-images.idx3-ubyte",
                       &train_images, -1.0, 1.0, 2, 2);
    parse_mnist_labels(data_dir_path + "/t10k-labels.idx1-ubyte",
                       &test_labels);
    parse_mnist_images(data_dir_path + "/t10k-images.idx3-ubyte",
                       &test_images, -1.0, 1.0, 2, 2);

    std::cout << "start training" << std::endl;

    progress_display disp(static_cast<unsigned long>(train_images.size()));
    timer t;
    int minibatch_size = 10;
    int num_epochs = 30;

    optimizer.alpha *= static_cast<tiny_dnn::float_t>(std::sqrt(minibatch_size));

    // create callback
    auto on_enumerate_epoch = [&](){
        std::cout << t.elapsed() << "s elapsed." << std::endl;
        tiny_dnn::result res = nn.test(test_images, test_labels);
        std::cout << res.num_success << "/" << res.num_total << std::endl;

        disp.restart(static_cast<unsigned long>(train_images.size()));
        t.restart();
    };

    auto on_enumerate_minibatch = [&](){
        disp += minibatch_size;
    };

    // training
    nn.train<mse>(optimizer, train_images, train_labels, minibatch_size, num_epochs,
             on_enumerate_minibatch, on_enumerate_epoch);

    std::cout << "end training." << std::endl;

    // test and show results
    nn.test(test_images, test_labels).print_detail(std::cout);

    // save network model & trained weights
    nn.save("LeNet-model");
}

int main(int argc, char **argv) {
  ROS_INFO("Initializing test ...");
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
//  while(n.ok())
  {
    if (argc != 2) {
        std::cerr << "Usage : " << argv[0]
                  << " path_to_data (example:../data)" << std::endl;
    }
    else
    {
      train_lenet(argv[1]);
    }
//    ros::spinOnce();
//    loop_rate.sleep();
  }
}
/*
int main(int argc, char** argv)
{
  ROS_INFO("Initializing test ...");
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

 

  ros::spinOnce();
  ros::loop_rate.sleep();
}
*/
