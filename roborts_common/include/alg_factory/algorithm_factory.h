/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_COMMON_ALGORITHM_FACTORY_H
#define ROBORTS_COMMON_ALGORITHM_FACTORY_H
#include <memory>
#include <stdio.h>
#include <iostream>
#include <functional>
#include <unordered_map>

#include "bind_this.h"

namespace roborts_common{
/**
 * @brief In order to conveniently using the sepecific algorithm in each function module
 *        such as mapping, localization, planning, it is nessessary to register algorithm
 *        class to the algorithm container in AlgorithmFactory class. The example is at
 *        the end of this program.
 * @tparam AlgorithmBase The base class of algorithm.
 * @tparam Args Parameter pack
 */
template<class AlgorithmBase, typename... Args>
class AlgorithmFactory
{
 public:
  using AlgorithmHash = std::unordered_map<std::string, std::function<std::unique_ptr<AlgorithmBase>(Args...)>>;

  /**
   * @brief The only way to get AlgorithmHash.
   * @return Ref of AlgorithmHash
   */
  static AlgorithmHash& GetAlgorithmHash(){
    static AlgorithmHash algorithm_hash;
    return algorithm_hash;
  }

  /**
   * @brief Register specific algorithm to the algorithm container called AlgorithmHash.
   * @tparam ParamType Type emplate of parameters
   * @param algorithm_name Algorithm name, which must be the same with the variable defined in ".prototxt" file.
   * @param args Parameter pack
   * @return Return true if register successfully, false otherwith.
   */
  template <typename ParamType>
  static bool Register(const std::string algorithm_name,
                       ParamType&& args){
    AlgorithmHash& algorithm_hash = GetAlgorithmHash();
    auto factory_iter = algorithm_hash.find(algorithm_name);
    if(factory_iter == algorithm_hash.end()) {
      algorithm_hash.emplace(std::make_pair(algorithm_name, std::forward<ParamType>(args)));
      std::cout << algorithm_name << " registered successfully!" << std::endl;
    }
    else
      std::cout << algorithm_name << " has been registered!" << std::endl;
    return true;
  }

  /**
   * @brief Unregister algorithm from the algorithm container.
   * @param algorithm_name Algorithm name, which must be the same with the variable defined in ".prototxt" file.
   * @return Return true if register successfully, false otherwith.
   */
  static bool UnRegister(const std::string algorithm_name){
    AlgorithmHash& algorithm_hash = GetAlgorithmHash();
    auto factory_iter = algorithm_hash.find(algorithm_name);
    if(factory_iter != algorithm_hash.end()){
      algorithm_hash.erase(algorithm_name);
      return true;
    }
    else{
      std::cout << "Failed to unregister algorithm, it is a unregistered alrorithm."
                << algorithm_name << std::endl;
      return false;
    }
  }

  /**
   * @brief Create an algorithm class that has been registered before.
   * @param algorithm_name Algorithm name, which must be the same with the variable defined in ".prototxt" file.
   * @param args Parameter pack
   * @return The base class unique_ptr that point to Algorithm corresponding to the algorithm name.
   */
  static std::unique_ptr<AlgorithmBase> CreateAlgorithm(const std::string algorithm_name,
                                                        Args... args) {
    AlgorithmHash& algorithm_hash = GetAlgorithmHash();

    auto factory_iter = algorithm_hash.find(algorithm_name);
    if(factory_iter == algorithm_hash.end()){
      std::cout << "Can't creat algorithm " << algorithm_name <<
               ", because you haven't register it!" << std::endl;
      return nullptr;
    }
    else{
      return (factory_iter->second)(std::forward<Args>(args)...);
    }
  }

 private:
  AlgorithmFactory(){}
};

template<typename AlgorithmBase, typename Algorithm, typename... Args>
class AlgorithmRegister{
 public:
  /**
   * @brief Constructor function of AlgorithmRegister class.
   * @param algorithm_name Algorithm name, which must be the same with the variable defined in ".prototxt" file.
   */
  explicit AlgorithmRegister(std::string algorithm_name) {
    Register(algorithm_name, make_int_sequence<sizeof...(Args)>{});
  }
  /**
   * @brief Create an unique_ptr pointer of Algorithm corresponding to the algorithm name.
   * @param data Parameter pack
   * @return The base class unique_ptr that point to Algorithm corresponding to the algorithm name.
   */
  static std::unique_ptr<Algorithm> create(Args&&... data)
  {
    return std::make_unique<Algorithm>(std::forward<Args>(data)...);
  }

 private:
  /**
   * @brief Using std::bind to create a std::function, than pass it to Register function befined in AlgorithmnFactory
   *        class
   * @tparam Is Parameter pack
   * @param algorithm_name Algorithm name, which must be the same with the variable defined in ".prototxt" file.
   */
  template <int... Is>
  void Register(std::string algorithm_name, int_sequence<Is...>) {
    auto function = std::bind(&AlgorithmRegister<AlgorithmBase, Algorithm, Args...>::create,
                              std::placeholder_template<Is>{}...);
    AlgorithmFactory<AlgorithmBase, Args...>::Register(algorithm_name, function);
  }
};
/**
 * @brief It is convenient to register specific algorithm using this Macro.
 * @example
 * class BaseClass {
 *   public:
 *    BaseClass(int num){
 *      num_ = num;
 *    }
 *    virtual void Excute() = 0;
 *   public:
 *    int num_;
 *  };
 *
 *  class A: public BaseClass {
 *   public:
 *    A(int num): BaseClass(num) {}
 *    void Excute(){
 *      std::cout << "A, you are so beautiful!" << std::endl;
 *    }
 *  };
 *  REGISTER_ALGORITHM(BaseClass, "a", A, int);
 *
 *  class B: public BaseClass {
 *   public:
 *    B(int num): BaseClass(num){}
 *    void Excute() {
 *      std::cout << "B, you are so beautiful!" << std::endl;
 *    }
 *  };
 *  REGISTER_ALGORITHM(BaseClass, "b", B, int);
 *
 *  int main() {
 *    auto base_class = AlgorithmFactory<BaseClass, int>::CreateAlgorithm("a", 3);
 *    base_class->Excute();
 *  }
 */
#define NAME(name) register_##name##_algorithm
#define REGISTER_ALGORITHM(AlgorithmBase, AlgorithmName, Algorithm, ...)                          \
        AlgorithmRegister<AlgorithmBase, Algorithm, ##__VA_ARGS__> NAME(Algorithm)(AlgorithmName)
} //namespace roborts_common

#endif // ROBORTS_COMMON_ALGORITHM_FACTORY_H