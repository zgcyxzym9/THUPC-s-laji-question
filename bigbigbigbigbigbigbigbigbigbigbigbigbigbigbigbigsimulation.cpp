#include <iostream>
#include <vector>

struct Vector
{
    double x;
    double y;
    double z;
};

Vector operator+(const Vector a, const Vector b)
{
    Vector ans;
    ans.x = a.x + b.x;
    ans.y = a.y + b.y;
    ans.z = a.z + b.z;
    return ans;
}

Vector operator-(const Vector a, const Vector b)
{
    Vector ans;
    ans.x = a.x - b.x;
    ans.y = a.y - b.y;
    ans.z = a.z - b.z;
    return ans;
}

double operator*(const Vector a, const Vector b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

class Missile
{
public:
    Missile(int id, int faction, double yaw_rate, double max_speed, double safe_dist, double explode_dist, double max_lock_angle, double nav_time)
    {
        target_ = 0;
        is_launched_ = false;
        is_active_ = false;
        yaw_rate_ = yaw_rate;
        id_ = id;
        faction_ = faction;
        max_speed_ = max_speed;
        safe_dist_ = safe_dist;
        explode_dist_ = explode_dist;
        max_lock_angle_ = max_lock_angle;
        nav_time_ = nav_time;
    }
    Missile();

private:
    int id_;
    int faction_;
    int target_;
    Vector pos_;
    Vector direction_;
    double yaw_rate_, max_speed_, safe_dist_, explode_dist_, max_lock_angle_, nav_time_;
    bool is_launched_;
    bool is_active_;
};

class Drone
{
public:
    Drone(int id, int faction, Vector pos, Vector direction, Vector lift,
          double pull_rate, double push_rate, double roll_rate, double max_speed, double lateral_scan_range, double vertical_scan_range,
          double myr, double mms, double msd, double med, double mmla, double mnt) : missile_(id, faction, myr, mms, msd, med, mmla, mnt)
    {
        is_alive_ = true;
        target_ = 0;
        id_ = id;
        faction_ = faction;
        pos_ = pos;
        direction_ = direction;
        lift_ = lift;
        pull_rate_ = pull_rate;
        push_rate_ = push_rate;
        roll_rate_ = roll_rate;
        max_speed_ = max_speed;
        lateral_scan_range_ = lateral_scan_range;
        vertical_scan_range_ = vertical_scan_range;
    }
    Drone();
    void LockTarget(std::vector<Drone> &DroneList); // 选定目标

private:
    int id_;
    int faction_;
    int target_;
    Vector pos_;
    Vector direction_;
    Vector lift_;
    double pull_rate_, push_rate_, roll_rate_, max_speed_, lateral_scan_range_, vertical_scan_range_;
    bool is_alive_;
    Missile missile_;
};

void Drone::LockTarget(std::vector<Drone> &DroneList) // 选定目标
{
    /*
    若视野内无敌方阵营无人机，则无选定目标
    否则若上一时刻的目标仍位于视野内，则仍选定该目标
    否则若有敌机在雷达扫描范围内，则选取距离最近的，距离相同取编号最小
    否则选取取 min{|rx − Lx|, |rx + Lx|} + min{|ry − Hy|, |ry + Hy|} 最小的，相同则取编号最小
    */

    int drone_num = DroneList.size();
}

int main()
{
    int n, T;
    std::cin >> n >> T;
    std::vector<Drone> DroneList;

    for (int i = 1; i <= 2 * n; i++)
    {
        Vector pos, dir, lift;
        double pullr, pushr, rr, ms, lsr, vsr;
        std::cin >> pos.x >> pos.y >> pos.z >> dir.x >> dir.y >> dir.z >> lift.x >> lift.y >> lift.z >> pullr >> pushr >> rr >> ms >> lsr >> vsr;
        double myr, mms, msd, med, mmla, mnt;
        std::cin >> myr >> mms >> msd >> med >> mmla >> mnt;
        DroneList.push_back(Drone(i, int((i - 1) / n), pos, dir, lift, pullr, pushr, rr, ms, lsr, vsr, myr, mms, msd, med, mmla, mnt));
    }

    return 0;
}