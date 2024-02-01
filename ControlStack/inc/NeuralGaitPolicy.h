// #include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include "Types.h"
// Header for onnxruntime
#include <onnxruntime_cxx_api.h>

using namespace Hopper_t;

// /**
//  * @brief Compute the product over all the elements of a vector
//  * @tparam T
//  * @param v: input vector
//  * @return the product
//  */
// template <typename T>
// size_t vectorProduct(const std::vector<T>& v) {
//   return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
// }

class NeuralGaitPolicy {
 public:
  /**
   * @brief Constructor
   * @param modelFilepath: path to the .onnx file
   */
  NeuralGaitPolicy(const std::string& modelFilepath);

  /**
   * @brief Perform inference on a single image
   * @param imageFilepath: path to the image
   * @return the index of the predicted class
   */
  vector_2t Inference(vector_t states);

 private:
  // ORT Environment
  std::shared_ptr<Ort::Env> mEnv;

  // Session
  std::shared_ptr<Ort::Session> mSession;

  // Inputs
  char* mInputName;
  std::vector<int64_t> mInputDims;

  // Outputs
  char* mOutputName;
  std::vector<int64_t> mOutputDims;
};

// #endif  // IMAGE_CLASSIFIER_H_
