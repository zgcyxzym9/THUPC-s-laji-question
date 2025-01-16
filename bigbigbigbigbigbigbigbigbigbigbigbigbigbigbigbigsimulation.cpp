#include <iostream>

struct Vector
{
    double x;
    double y;
    double z;
};

class Drone
{
public:
    Drone(int id, int faction, Vector pos_, Vector direction_, Vector lift_,
          double pull_rate_, double push_rate_, double roll_rate_, double max_speed_, double lateral_scan_range_, double vertical_scan_range_)
    {
        is_alive_ = true;
    }

private:
    int id_;
    int faction_;
    Vector pos_;
    Vector direction_;
    Vector lift_;
    double pull_rate_, push_rate_, roll_rate_, max_speed_, lateral_scan_range_, vertical_scan_range_;
    bool is_alive_;
};

class Missile
{
public:
private:
};

int main()
{
    int n, T;
    std::cin >> n >> T;

    return 0;
}