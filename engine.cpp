#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>

// Gravitational constant and collision restitution coefficient
const double G = 6.67430e-11;
const double RESTITUTION = 1.0;  // perfectly elastic collisions

// 2D vector with basic arithmetic operations
class Vector2D {
public:
    double x, y;

    Vector2D(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}

    Vector2D operator+(const Vector2D& other) const { return {x + other.x, y + other.y}; }
    Vector2D operator-(const Vector2D& other) const { return {x - other.x, y - other.y}; }
    Vector2D operator-() const { return {-x, -y}; }
    Vector2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
    Vector2D operator/(double scalar) const { return {x / scalar, y / scalar}; }

    Vector2D& operator+=(const Vector2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2D& operator-=(const Vector2D& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    double dot(const Vector2D& other) const { return x * other.x + y * other.y; }
    double magnitude() const { return std::sqrt(x * x + y * y); }
    Vector2D normalized() const {
        double m = magnitude();
        if (m == 0.0) return {0.0, 0.0};
        return (*this) / m;
    }
};

// Represents a particle in the simulation
class Body {
public:
    double mass;
    double radius;
    Vector2D position;
    Vector2D velocity;
    Vector2D force;

    Body(double m, double r, Vector2D p, Vector2D v)
        : mass(m), radius(r), position(p), velocity(v), force(0.0, 0.0) {}

    void resetForce() { force = {0.0, 0.0}; }
    void addForce(const Vector2D& f) { force += f; }

    void update(double dt) {
        Vector2D acceleration = force / mass;
        velocity += acceleration * dt;
        position += velocity * dt;
    }
};

// Simulation environment handling n-body gravity and collisions
class Simulation {
    double dt;  // time step in seconds
    std::vector<Body> bodies;

    // Gravitational force from a on b
    Vector2D gravitationalForce(const Body& a, const Body& b) const {
        Vector2D displacement = b.position - a.position;
        double distance = displacement.magnitude();
        if (distance == 0.0) return {0.0, 0.0};
        double forceMag = G * a.mass * b.mass / (distance * distance);
        return displacement.normalized() * forceMag;
    }

    void computeForces() {
        for (auto& b : bodies) b.resetForce();

        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                Vector2D f = gravitationalForce(bodies[i], bodies[j]);
                bodies[i].addForce(f);
                bodies[j].addForce(-f);
            }
        }
    }

    // Simple impulse-based elastic collision handling
    void handleCollisions() {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                Vector2D displacement = bodies[j].position - bodies[i].position;
                double distance = displacement.magnitude();
                double minDist = bodies[i].radius + bodies[j].radius;

                if (distance < minDist && distance > 0.0) {
                    Vector2D normal = displacement.normalized();
                    double relativeVel = (bodies[i].velocity - bodies[j].velocity).dot(normal);
                    if (relativeVel > 0) continue;  // bodies separating

                    double impulse = -(1 + RESTITUTION) * relativeVel /
                                      (1 / bodies[i].mass + 1 / bodies[j].mass);

                    bodies[i].velocity += normal * (impulse / bodies[i].mass);
                    bodies[j].velocity -= normal * (impulse / bodies[j].mass);

                    // Separate overlapping bodies
                    double overlap = minDist - distance;
                    bodies[i].position -= normal * (overlap * (bodies[j].mass / (bodies[i].mass + bodies[j].mass)));
                    bodies[j].position += normal * (overlap * (bodies[i].mass / (bodies[i].mass + bodies[j].mass)));
                }
            }
        }
    }

public:
    explicit Simulation(double dt_) : dt(dt_) {}

    void addBody(const Body& b) { bodies.push_back(b); }

    void step() {
        computeForces();
        for (auto& b : bodies) b.update(dt);
        handleCollisions();
    }

    void run(int steps) {
        for (int i = 0; i < steps; ++i) {
            step();
        }
    }

    void printState() const {
        std::cout << std::fixed << std::setprecision(2);
        for (size_t i = 0; i < bodies.size(); ++i) {
            std::cout << "Body " << i << ": (" << bodies[i].position.x
                      << ", " << bodies[i].position.y << ")\n";
        }
    }

    double totalKineticEnergy() const {
        double e = 0.0;
        for (const auto& b : bodies) {
            e += 0.5 * b.mass * b.velocity.dot(b.velocity);
        }
        return e;
    }

    double totalPotentialEnergy() const {
        double e = 0.0;
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                double distance = (bodies[j].position - bodies[i].position).magnitude();
                if (distance > 0.0)
                    e -= G * bodies[i].mass * bodies[j].mass / distance;
            }
        }
        return e;
    }

    void printEnergy() const {
        std::cout << "Total kinetic energy: " << totalKineticEnergy() << "\n";
        std::cout << "Total potential energy: " << totalPotentialEnergy() << "\n";
    }
};

int main() {
    // Simulation with three bodies: Earth, Moon and a satellite
    Simulation sim(3600.0);  // 1-hour time step

    // Mass (kg), radius (m), position (m), velocity (m/s)
    sim.addBody(Body(5.972e24, 6.371e6, {0.0, 0.0}, {0.0, 0.0}));                // Earth
    sim.addBody(Body(7.348e22, 1.737e6, {384.4e6, 0.0}, {0.0, 1022.0}));        // Moon
    sim.addBody(Body(1000.0, 1.0, {384.4e6 + 10000.0, 0.0}, {0.0, 1522.0}));    // Satellite

    sim.run(24 * 10);  // simulate 10 days
    sim.printState();
    sim.printEnergy();

    return 0;
}

