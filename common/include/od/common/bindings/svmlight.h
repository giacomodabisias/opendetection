/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


 Original code under the following Apache License 2.0:
 Changes - None

 ----------
  This software is licensed under the Apache License 2.0 (http://www.apache.org/licenses/LICENSE-2.0.html) Please find the Apache license 2.0 statement in the respective file alongside this software.

Copyright 2015 Jan Hendriks

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
 ----------
 */

/**
 * @file:   svmlight.h
 * @author: Jan Hendriks (dahoc3150 [at] gmail.com)
 * @date:   Created on 11. Mai 2011
 * @brief:  Wrapper interface for SVMlight, 
 * @see http://www.cs.cornell.edu/people/tj/svm_light/ for SVMlight details and terms of use
 * 
 */
#pragma once
#include <iostream>
#include <vector>
#include <string>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace svmlight {
    extern "C" {
        #include "svm_common.h"
        #include "svm_learn.h"
    }
}
#endif 


class SVMlight {
private:

    svmlight::DOC ** docs_; // training examples
    long totwords_, totdoc_; // support vector stuff
    double * target_;
    double * alpha_in_;
    svmlight::KERNEL_CACHE * kernel_cache_;
    svmlight::MODEL * model_; // SVM model

    SVMlight();
    virtual ~SVMlight();

public:

    svmlight::LEARN_PARM learn_parm_;
    svmlight::KERNEL_PARM kernel_parm_;

    static SVMlight * getInstance();

    void saveModelToFile(const std::string model_file_name);

    void loadModelFromFile(const std::string model_file_name);

    // read in a problem (in svmlight format)
    void read_problem(char * filename);

    // Calls the actual machine learning algorithm
    void train();

    /**
     * Generates a single detecting feature vector (vec1) from the trained support vectors, for use e.g. with the HOG algorithm
     * vec1 = sum_1_n (alpha_y*x_i). (vec1 is a 1 x n column vector. n = feature vector length)
     * @param single_detector_vector resulting single detector vector for use in openCV HOG
     */
    void getSingleDetectingVector(std::vector<float> & single_detector_vector);
    
    /**
     * Return model detection threshold / bias
     * @return detection threshold / bias
     */
    float getThreshold() const;
    
    std::string getSVMName() const;
};

