#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>
#include <array>
#include <tuple>
#include <fstream>
#include <iomanip>

#include <iostream>
constexpr float PI = 3.141592653589f;
constexpr float GRAVITY = 9.81f;
constexpr float CART_WIDTH = 100.0f;
constexpr float CART_HEIGHT = 40.0f;
constexpr float PENDULUM_RADIUS = 10.0f;
constexpr float ROD_WIDTH = 2.0f;
constexpr float BASE_HEIGHT = 400.0f;
constexpr float INITIAL_POSITION = 400.0f;
constexpr float threshold = 0.000001f;

class PendulumOnCart : public sf::Drawable, public sf::Transformable {
public:

    PendulumOnCart(float length, float cartMass, float pendulumMass, float friction, float pendulumFriction, const std::array<float, 4>& initialState)
        : L(length), m_c(cartMass), m_p(pendulumMass), mu(friction), quadraticFriction(pendulumFriction), y(initialState), dydt{ 0, 0, 0, 0 } {
        // Initialize cart shape
        cart.setSize(sf::Vector2f(CART_WIDTH, CART_HEIGHT));



        cart.setOrigin(sf::Vector2f(CART_WIDTH / 2.0f, CART_HEIGHT / 2.0f));
        cart.setFillColor(sf::Color::Magenta);

        // Initialize pendulum shape
        pendulum.setRadius(PENDULUM_RADIUS);
        pendulum.setOrigin(sf::Vector2f(PENDULUM_RADIUS, PENDULUM_RADIUS));
        pendulum.setFillColor(sf::Color::Red);

        // Initialize rod shape
        rod.setSize(sf::Vector2f(ROD_WIDTH, L));
        rod.setOrigin(sf::Vector2f(ROD_WIDTH / 2.0f, L));
        rod.setFillColor(sf::Color::Black);
    }

    float ki = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float speed = 60.0f;
    float con1 = 0.0f;
    mutable std::vector<sf::Vertex> phasePoints;
    mutable std::vector<sf::Vertex> outputp;
    void reset() {
        y = { INITIAL_POSITION, 0.0f, 0.0f, 0.0f };
        error_i = 0.0f;
        error_k = 0.0f;
        errorx = 0.0f;
        position_error_i = 0.0f;
        last_time = 0.0f;
    }

    void equations(float F) {
        
        float x_dot = y[1];
        float theta = y[2];
        float theta_dot = y[3];
        
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);
        float denominator = m_c + m_p * (1.0f - cosTheta * cosTheta);

        dydt[0] = x_dot;
        dydt[1] = (F + m_p * l_mm * theta_dot * theta_dot * sinTheta - m_p * GRAVITY * sinTheta * cosTheta
            - mu * x_dot - quadraticFriction * x_dot * std::abs(x_dot)) / denominator;
        dydt[2] = theta_dot;
        dydt[3] = (GRAVITY * sinTheta - cosTheta * (dydt[1] + m_p * l_mm * theta_dot * theta_dot * sinTheta)
            - quadraticFriction * theta_dot * std::abs(theta_dot)) / (l_mm * (4.0f / 3.0f - m_p * cosTheta * cosTheta / (m_c + m_p)));
    }

    void rk4(float t, float h, float F) {
        std::array<float, 4> k1, k2, k3, k4, y_temp;

        // k1
        equations(F);
        for (int i = 0; i < 4; i++) k1[i] = h * dydt[i];

        // k2
        for (int i = 0; i < 4; i++) y_temp[i] = y[i] + 0.5f * k1[i];
        equations_temp(y_temp, F);
        for (int i = 0; i < 4; i++) k2[i] = h * dydt[i];

        // k3
        for (int i = 0; i < 4; i++) y_temp[i] = y[i] + 0.5f * k2[i];
        equations_temp(y_temp, F);
        for (int i = 0; i < 4; i++) k3[i] = h * dydt[i];

        // k4
        for (int i = 0; i < 4; i++) y_temp[i] = y[i] + k3[i];
        equations_temp(y_temp, F);
        for (int i = 0; i < 4; i++) k4[i] = h * dydt[i];

        // Update y
        for (int i = 0; i < 4; i++)
            y[i] += (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]) / 6.0f;

        
        last_time = t;
        sf::Vector2f point(y[2], y[3]);
        point.x *= 100.0f;
        point.y *= -100.0f;
        sf::Vertex vertex{{500 + point.x, 300 + point.y}};
        phasePoints.emplace_back(vertex);
        if (phasePoints.size() > 1000) phasePoints.erase(phasePoints.begin());





    }


    float getPosition() const {
        error_ii += ((y[0] - 850.0f) / 850.0f);
        return error_ii;
    }

    std::tuple<float, float, float> pd_get_error() const {
        
        float theta = atan2(sin(y[2]), cos(y[2]));
        
        float errorl = -theta;        
        float errordot = -y[3];         
        
        const float MAX_INTEGRAL = 16;

        if (abs(errorl) <= 0.09)
        {
            const float MAX_INTEGRAL =1.5;
        } 
        else
        {
            const float MAX_INTEGRAL = 17;
        }
        error_i += errorl /60.0f;     

        
        if (error_i > MAX_INTEGRAL) error_i = MAX_INTEGRAL;
        else if (error_i < -MAX_INTEGRAL) error_i = -MAX_INTEGRAL;
       
        
        return std::make_tuple(errorl, errordot, error_i); 
    }

    float pendulumpid() {
        
        float error = y[0] - 850.0f;
        float errord = y[1];

       
        const float MAX_POS_INTEGRAL = 1000.0f;
        position_error_i += error * 1.f/speed;  

        if (position_error_i > MAX_POS_INTEGRAL)
            position_error_i = MAX_POS_INTEGRAL;
        else if (position_error_i < -MAX_POS_INTEGRAL)
            position_error_i = -MAX_POS_INTEGRAL;

        float kp_pos = 1.9f;
        float ki_pos = 0.0f;
        float kd_pos = 0.0f;

        
      
            return (kp_pos * error + ki_pos * position_error_i + kd_pos * errord);
        
    }

    ~PendulumOnCart() {
        
    }

private:
    float L;       
    float l_mm = L ; 
    float m_c;                   // Mass of the cart  
    float m_p;                   // Mass of the pendulum  
    float mu;                    // Friction coefficient  
    float quadraticFriction;     // Quadratic friction coefficient
    mutable float error_i = 0.0f;
    mutable float error_k = 0.0f; // Sýnýf üyesi error_k
    mutable float errorx = 0.0f;
    mutable float error_ii = 0.0f;
    mutable float position_error_i = 0.0f;
    mutable float lastd = 0.0f;
    float last_time = 0.0f;
    std::array<float, 4> y;      // State variables [x, x', theta, theta']  
    std::array<float, 4> dydt;   // Derivatives [x', x'', theta', theta'']
  

    mutable sf::RectangleShape cart;
    mutable sf::CircleShape pendulum;
    mutable sf::RectangleShape rod;

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
       
        cart.setPosition(sf::Vector2f(y[0], BASE_HEIGHT));
        rod.setPosition(sf::Vector2f(cart.getPosition().x, BASE_HEIGHT - 20.0f)); 
        rod.setRotation(sf::degrees(y[2] * 180.0f / PI)); 
        pendulum.setPosition(sf::Vector2f( 
            rod.getPosition().x + L * std::sin(y[2]),
            rod.getPosition().y - L * std::cos(y[2])
        ));

       
        target.draw(cart, states);
        target.draw(rod, states);
        target.draw(pendulum, states);
       
    }

    void equations_temp(const std::array<float, 4>& y_input, float F) {
        float x = y_input[0];
        float x_dot = y_input[1];
        float theta = y_input[2];
        float theta_dot = y_input[3];

        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);
        float denominator = m_c + m_p * (1.0f - cosTheta * cosTheta);

        dydt[0] = x_dot;
        dydt[1] = (F + m_p * l_mm * theta_dot * theta_dot * sinTheta - m_p * GRAVITY * sinTheta * cosTheta
            - mu * x_dot - quadraticFriction * x_dot * std::abs(x_dot)) / denominator;
        dydt[2] = theta_dot;
        dydt[3] = (GRAVITY * sinTheta - cosTheta * (dydt[1] + m_p * l_mm * theta_dot * theta_dot * sinTheta)
            - quadraticFriction * theta_dot * std::abs(theta_dot)) / (l_mm * (4.0f / 3.0f - cosTheta * cosTheta));
    }
};