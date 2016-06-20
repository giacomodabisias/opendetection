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

#include "od/common/bindings/ODSvmlight.h"

SVMlight::SVMlight(){
    // Init variables
    alpha_in_ = NULL;
    kernel_cache_ = NULL; // Cache not needed with linear kernel
    model_ = static_cast<svmlight::MODEL *>(svmlight::my_malloc(sizeof(svmlight::MODEL)));
    // Init parameters
    svmlight::verbosity = 1; // Show some messages -v 1
    learn_parm_.alphafile[0] = ' '; // NULL; // Important, otherwise files with strange/invalid names appear in the working directory
    //        learn_parm_->alphafile = NULL; // Important, otherwise files with strange/invalid names appear in the working directory
    learn_parm_.biased_hyperplane = 1;
    learn_parm_.sharedslack = 0; // 1
    learn_parm_.skip_final_opt_check = 0;
    learn_parm_.svm_maxqpsize = 10;
    learn_parm_.svm_newvarsinqp = 0;
    learn_parm_.svm_iter_to_shrink = 2; // 2 is for linear;
    learn_parm_.kernel_cache_size = 40;
    learn_parm_.maxiter = 100000;
    learn_parm_.svm_costratio = 1.0;
    learn_parm_.svm_costratio_unlab = 1.0;
    learn_parm_.svm_unlabbound = 1E-5;
    learn_parm_.eps = 0.1;
    learn_parm_.transduction_posratio = -1.0;
    learn_parm_.epsilon_crit = 0.001;
    learn_parm_.epsilon_a = 1E-15;
    learn_parm_.compute_loo = 0;
    learn_parm_.rho = 1.0;
    learn_parm_.xa_depth = 0;
    // The HOG paper uses a soft classifier (C = 0.01), set to 0.0 to get the default calculation
    learn_parm_.svm_c = 0.01; // -c 0.01
    learn_parm_.type = REGRESSION;
    learn_parm_.remove_inconsistent = 0; // -i 0 - Important
    kernel_parm_.rbf_gamma = 1.0;
    kernel_parm_.coef_lin = 1;
    kernel_parm_.coef_const = 1;
    kernel_parm_.kernel_type = LINEAR; // -t 0
    kernel_parm_.poly_degree = 3;
}

SVMlight::~SVMlight() {
    // Cleanup area
    // Free the memory used for the cache
    if(kernel_cache_)
        svmlight::kernel_cache_cleanup(kernel_cache_);
    free(alpha_in_);
    free_model(model_, 0);
    for(long i = 0; i < totdoc_; ++i)
        free_example(docs_[i], 1);
    free(docs_);
    free(target_);
}

void SVMlight::saveModelToFile(const std::string model_file_name) {
    write_model(const_cast<char*>(model_file_name.c_str()), model_);
}

void SVMlight::loadModelFromFile(const std::string model_file_name) {
    model_ = svmlight::read_model(const_cast<char*>(model_file_name.c_str()));
}

void SVMlight::read_problem(char * file_name) {
    read_documents(file_name, &docs_, &target_, &totwords_, &totdoc_);
}

void SVMlight::train() {
    svm_learn_regression(docs_, target_, totdoc_, totwords_, &learn_parm_, &kernel_parm_, &kernel_cache_, model_);
}

void SVMlight::getSingleDetectingVector(std::vector<float> & single_detector_vector) {
    // Now we use the trained svm to retrieve the single detector vector
    svmlight::DOC ** supveclist = model_->supvec;
    std::cout << "Calculating single descriptor vector out of support vectors (may take some time)" << std::endl;
    // Retrieve single detecting vector (v1) from returned ones by calculating vec1 = sum_1_n (alpha_y*x_i). (vec1 is a n x1 column vector. n = feature vector length)
    single_detector_vector.clear();
    single_detector_vector.resize(model_->totwords, 0.);
    std::cout << "Resulting vector size " <<  single_detector_vector.size() << std::endl;
    
    // Walk over every support vector
    for(long ssv = 1; ssv < model_->sv_num; ++ssv) { // Don't know what's inside model->supvec[0] ?!
        // Get a single support vector
        svmlight::DOC * singleSupportVector = supveclist[ssv]; // Get next support vector
        svmlight::SVECTOR * singleSupportVectorValues = singleSupportVector->fvec;
        svmlight::WORD singleSupportVectorComponent;
        // Walk through components of the support vector and populate our detector vector
        for(long singleFeature = 0; singleFeature < model_->totwords; ++singleFeature) {
            singleSupportVectorComponent = singleSupportVectorValues->words[singleFeature];
            single_detector_vector.at(singleSupportVectorComponent.wnum-1) += (singleSupportVectorComponent.weight * model_->alpha[ssv]);
        }
    }
}

float SVMlight::getThreshold() const {
    return model_->b;
}

std::string SVMlight::getSVMName() const {
    return std::string("SVMlight");
}


/// Singleton
SVMlight * SVMlight::getInstance() {
    static SVMlight theInstance;
    return &theInstance;
}
