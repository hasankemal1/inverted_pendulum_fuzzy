#include <iostream>
#include <SFML/Window.hpp>
#include <math.h>
#include <SFML/Graphics.hpp>
#include <nlopt.hpp>
#include "pendulum.hpp"
#include "imgui.h" // Necessary for ImGui::*
#include "imgui-SFML.h" // For ImGui::SFML::* functions and SFML-specific overloads
#include <fstream>
#include "fuzzy.hpp"
#include "implot.h" // Include Implot for plotting
static bool showSecondWindow = false;
static bool showtWindow = false;
static std::vector<float> errorBuffer;
static const int   maxSamples = 600000;  // tutmak istediðiniz son örnek sayýsý
static const float dt = 1.0f / 60.0f;

void renderPhaseDiagram(const PendulumOnCart& pendulum);
constexpr unsigned int WINDOW_WIDTH = 1900;
constexpr unsigned int WINDOW_HEIGHT = 800;
constexpr char WINDOW_TITLE[] = "Test";
constexpr unsigned int FRAME_RATE_LIMIT = 60;

constexpr float MIN_SPEED = 10.0f; // Minimum speed for the simulation
constexpr float MAX_SPEED = 240.0f; // Maximum speed for the simulation
// Function to handle events
void handleEvents(sf::RenderWindow& window) {
    while (const auto event = window.pollEvent()) {
        ImGui::SFML::ProcessEvent(window, *event);
        if (event->is<sf::Event::Closed>()) {
            window.close();
        }
    }
}



void renderImGuiControls(PendulumOnCart& pendulum, float& t, float& fixedTimeStep, float& force, bool& wasActiveLastFrame, bool& paused, bool& pausef) {
    ImGui::Begin("Pendulum Controls");

    if (ImGui::Button("Reset")) {
        t = 0.0f;
        pendulum.con1 = 0.0f;
        pendulum.reset();
        pendulum.speed = 60.0f; // Reset speed to default value
        fixedTimeStep = sf::seconds(1.0f / pendulum.speed).asSeconds();
        std::cout << "speed: " << pendulum.speed << std::endl;
        
    }

    if (ImGui::Button("Speed Up")) {
        if (pendulum.speed > MIN_SPEED) {
            pendulum.speed -= 10.0f;
            std::cout << "speed: " << pendulum.speed  << std::endl;
            fixedTimeStep = sf::seconds(1.0f / pendulum.speed ).asSeconds();
        } else {
            ImGui::Text("Error: max speed reached");
        }
    }

    if (ImGui::Button("Slow Down")) {
        if (pendulum.speed < MAX_SPEED) {
            pendulum.speed += 10.0f;
            std::cout << "speed: " << pendulum.speed << std::endl;
            fixedTimeStep = sf::seconds(1.0f / pendulum.speed).asSeconds();
        } else {
            ImGui::Text("Error: min speed reached");
        }
    }

    if (ImGui::Button("Pause")) {
        paused = !paused;
    }

    if (ImGui::Button(pausef ? "Stop Fuzzy" : "Start Fuzzy")) {
        if (!paused) {
            pausef = !pausef; 
        }
    }
    if (ImGui::Button("phase")) {
		showSecondWindow = !showSecondWindow; 
        
    }if (ImGui::Button("output")) {
        showtWindow = !showtWindow;

    }
    ImGui::SliderFloat("Force", &force, -80.0f, 80.0f);
    bool isActiveNow = ImGui::IsItemActive();
    if (wasActiveLastFrame && !isActiveNow) {
        force = 0.0f; // Reset force when the slider is released
    }
    wasActiveLastFrame = isActiveNow;

    ImGui::SliderFloat("Friction Force", &pendulum.con1, 0.0f, 0.01f);

    ImGui::End();
}


// Function to update game logic
void updateGameLogic(PendulumOnCart& pendulum, float& t, float fixedTimeStep, float force) {
    pendulum.rk4(t, fixedTimeStep, force);
    t += fixedTimeStep;
}
void renderPhaseDiagram(const PendulumOnCart& pendulum) {
    std::vector<float> x_values, y_values;
    x_values.reserve(pendulum.phasePoints.size());
    y_values.reserve(pendulum.phasePoints.size());

    if (!pendulum.phasePoints.empty()) {
        for (const auto& vertex : pendulum.phasePoints) {
            x_values.push_back(vertex.position.x);
            y_values.push_back(vertex.position.y);
        }

        if (ImGui::Begin("Phase Diagram")) {
            if (ImPlot::BeginPlot("Phase Diagram")) {
                ImPlot::PlotLine("Phase Points", x_values.data(), y_values.data(), x_values.size());
                ImPlot::EndPlot();
            }
            ImGui::End();
        }
    }
    else {
        std::cout << "No phase points to render." << std::endl;
    }
}

float clamp(float value, float min, float max) {  
    return (value < min) ? min : (value > max) ? max : value;  
}  
   
void drawoutput(const PendulumOnCart& pendulum) {
   
    float error = std::get<0>(pendulum.pd_get_error());
    errorBuffer.push_back(error);
    if (errorBuffer.size() > maxSamples)
        errorBuffer.erase(errorBuffer.begin());

   
    static std::vector<float> times;
    times.resize(errorBuffer.size());
    for (int i = 0; i < (int)times.size(); ++i)
        times[i] = i * dt;
    float x_max = times.empty() ? 0.0f : times.back();

   
    ImGui::SetNextWindowSize(ImVec2(700, 700), ImGuiCond_Always);
    ImPlot::SetNextAxesLimits(0.0f, x_max, -5, 5, ImGuiCond_Always);
    ImGui::Begin("Hata Plot Penceresi");

    
    if (ImPlot::BeginPlot("Hata Sinyali e(t)", ImVec2(600, 600))) {
       

        
        ImPlot::PlotLine("e(t)", times.data(), errorBuffer.data(), (int)errorBuffer.size());
        ImPlot::EndPlot();
    }

    ImGui::End();
}

float compute_force(PendulumOnCart& pendulum, FuzzyController &fuzzy) {  
    
    float kp=0.0f;
    float kd=0.0f;
    float ki=0.0f;
    float error, error_dot, error_i;  
    std::tie(error, error_dot, error_i) = pendulum.pd_get_error();  
    std::tie(kp, kd, ki) = fuzzy.compute(pendulum);  
    
    float x_f = pendulum.pendulumpid();  
   
    float force1 = (kp * error + kd * error_dot + ki * error_i)*0.55-x_f*0.21 ;  

    
    return clamp(force1, -33.f, 33.f);  
}   
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


int main() {
    // Create the SFML window
    sf::RenderWindow window(sf::VideoMode({ WINDOW_WIDTH, WINDOW_HEIGHT }), WINDOW_TITLE);
    window.setFramerateLimit(FRAME_RATE_LIMIT);

    std::ofstream myfile = std::ofstream("output.txt");
    if (!myfile.is_open()) {
        std::cerr << "Error opening file for writing." << std::endl;
        return -1;
    }

    
    if (!ImGui::SFML::Init(window)) {
        return -1;
    }

  
    PendulumOnCart pendulum(250.f, 0.550f, 0.10f, 0.6f, 0.001f, {850.0f, 0.0f, PI, 0.0f});
    sf::Clock clock;
    float t = 0.0f;
    float fixedTimeStep = sf::seconds(1.0f / 60.0f).asSeconds();
    float force = 0.0f;
    bool wasActiveLastFrame = false;

    bool paused = false;
    bool pausef = false;

    FuzzyController fuzzy; 
    ImPlot::CreateContext();
    
    while (window.isOpen()) {
        
        handleEvents(window);
        ImGui::SFML::Update(window, clock.restart());
        renderImGuiControls(pendulum, t, fixedTimeStep, force, wasActiveLastFrame, paused, pausef);
        if (!paused) {

            if (pausef) {


                force = compute_force(pendulum, fuzzy);
            }


            float error, error_dot, error_i;
            std::tie(error, error_dot, error_i) = pendulum.pd_get_error(); 
            if (myfile.is_open()) {
                myfile << error << "    " << error_dot << "   " << error_i << std::endl; // Write the error values to the file
            }
            else {
                std::cerr << "Error opening file for writing." << std::endl;
            }

            updateGameLogic(pendulum, t, fixedTimeStep, force);

            if (showSecondWindow) {
                renderPhaseDiagram(pendulum);
            }
            if (showtWindow) {
                drawoutput(pendulum);
            }
        }
            window.clear(sf::Color::White);
            window.draw(pendulum);
            ImGui::SFML::Render(window);
            window.display();
        
    }

    // Close the file after the loop
    if (myfile.is_open()) {
        myfile.close(); // Close the file properly
    }
    ImPlot::DestroyContext();
    ImGui::SFML::Shutdown();

    return 0;
}



