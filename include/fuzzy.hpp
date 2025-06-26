#ifndef FUZZY_HPP
#define FUZZY_HPP

#define FL_USE_FLOAT
#include <cmath>
#include <fl/Headers.h>
#include "pendulum.hpp"
#include <fstream>
#include <iostream>
#include <memory>
using namespace fl;

class FuzzyController {
public:
    FuzzyController() {
        try {
            MyFile.open("fuzzy_log.txt");
            fEngine = loadEngine("fEngine.fll");
           
            // Log successful initialization
            MyFile << "Fuzzy controllers initialized successfully" << std::endl;
            MyFile << "====================================" << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Error initializing fuzzy controller: " << e.what() << std::endl;
            throw; // Re-throw to let the calling code handle it
        }
    }

    std::tuple<float, float, float> compute(PendulumOnCart& pendulum) {
        try {
            float error = 0.0f;
            float errord = 0.0f;
            float error_i = 0.0f;
            std::tie(error, errord, error_i) = pendulum.pd_get_error();
			
           
           

            error = std::clamp(error, -3.1416f, 3.1416f);
            errord = std::clamp(errord, -5.0f, 5.0f);
            
            MyFile << "Input - error: " << error << " errord: " << errord << std::endl;
            
           
            fEngine->getInputVariable("thetaErr")->setValue(error);
            fEngine->getInputVariable("thetaDotErr")->setValue(errord);

           

            
            fEngine->process();
           

            // Get output values
            float kp = fEngine->getOutputVariable("Kp")->getValue();
            float ki = fEngine->getOutputVariable("Ki")->getValue();
            float kd = fEngine->getOutputVariable("Kd")->getValue();

            
            if (std::isnan(kp) || std::isnan(ki) || std::isnan(kd)) {
                MyFile << "WARNING: NaN detected in fuzzy output! Using fallback values." << std::endl;
                
                kp = std::isnan(kp) ? 0.2f : kp;
                ki = std::isnan(ki) ? 0.0f : ki;
                kd = std::isnan(kd) ? 0.1f : kd;
            }
            
            
            
           

            MyFile << "Raw outputs - kp: " << kp << " kd: " << kd << " ki: " << ki << std::endl;
          
            MyFile << "------------------------------------" << std::endl;

            return std::make_tuple(kp/8, kd / 6, ki * 5.8);
        }
        catch (const std::exception& e) {
            MyFile << "ERROR in compute(): " << e.what() << std::endl;
            return std::make_tuple(0.2f, 0.1f, 0.0f); 
        }
    }

    ~FuzzyController() {
        if (fEngine) {
            delete fEngine;
            fEngine = nullptr;
        }
       
       
        if (MyFile.is_open()) {
            MyFile << "Fuzzy controller shutdown" << std::endl;
            MyFile.close();
        }
    }

private:
    std::ofstream MyFile;
    Engine* fEngine = nullptr;
    
    
    Engine* loadEngine(const std::string& filename) {
        try {
            std::cout << "Loading fuzzy engine from: " << filename << std::endl;
            Engine* engine = FllImporter().fromFile(filename);
            
            if (!engine) {
                throw std::runtime_error("Failed to create engine from " + filename);
            }
            
            if (!engine->isReady()) {
                std::cerr << "[ERROR] Engine '" << filename << "' not ready" << std::endl;
                throw std::runtime_error("Engine not ready: " + filename);
            }
            
            return engine;
        }
        catch (const std::exception& e) {
            std::cerr << "Exception loading engine from " << filename << ": " << e.what() << std::endl;
            throw;
        }
    }
};

#endif // FUZZY_HPP
