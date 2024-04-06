#include <iostream>
#include <vector>
#include <cmath>

//Constants
const double G = 6.67430e-11;  //Grav constant

//Vector class for representing positions and velocities
class Vector2D {
public:
    double x, y;

    Vector2D(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}

    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }

    Vector2D operator*(double scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }

    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }
};

//Body representing a particle(object) in the simulation
class Body {
public:
    double mass;
    Vector2D position, velocity;

    Body(double mass_, Vector2D position_, Vector2D velocity_)
        : mass(mass_), position(position_), velocity(velocity_) {}

    void updatePosition(double dt) {
        position = position + velocity * dt;
    }

    void updateVelocity(const Vector2D& force, double dt) {
        Vector2D acceleration = force * (1.0 / mass);
        velocity = velocity + acceleration * dt;
    }
};

//Grav force calculation
Vector2D calculateGravitationalForce(const Body& body1, const Body& body2) {
    Vector2D displacement = body2.position - body1.position;
    double distance = displacement.magnitude();
    double force = G * body1.mass * body2.mass / (distance * distance);
    return displacement * (force / distance);
}

int main() {
    //Create two bodies
    Body body1(1.0e12, Vector2D(0.0, 0.0), Vector2D(0.0, 1000.0));
    Body body2(5.0e12, Vector2D(1.0e9, 0.0), Vector2D(0.0, 0.0));

    //Simulation loop
    double dt = 86400.0;  //Time step (1 day)
    std::vector<Vector2D> positions1, positions2;

    for (int i = 0; i < 365; i++) {
        //Get grav force
        Vector2D force = calculateGravitationalForce(body1, body2);

        //Update velocities
        body1.updateVelocity(-force, dt);
        body2.updateVelocity(force, dt);

        //Update positions
        body1.updatePosition(dt);
        body2.updatePosition(dt);

        //Store positions for visualization
        positions1.push_back(body1.position);
        positions2.push_back(body2.position);
    }

    //Print the final positions
    std::cout << "Final position of body1: (" << body1.position.x << ", " << body1.position.y << ")" << std::endl;
    std::cout << "Final position of body2: (" << body2.position.x << ", " << body2.position.y << ")" << std::endl;

    return 0;
}