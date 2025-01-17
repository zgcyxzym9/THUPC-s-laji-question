#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

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
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector operator*(const double a, const Vector b)
{
    Vector ans;
    ans.x = a * b.x;
    ans.y = a * b.y;
    ans.z = a * b.z;
    return ans;
}

Vector OuterProduct(const Vector a, const Vector b)
{
    Vector ans;
    ans.x = a.y * b.z - a.z * b.y;
    ans.y = a.z * b.x - a.x * b.z;
    ans.z = a.x * b.y - a.y * b.x;
    return ans;
}

Vector ProjectionToVector(const Vector a, const Vector b)
{
    Vector ans;
    ans = (a * b) / (b * b) * b;
    return ans;
}

double Norm(const Vector a)
{
    return sqrt(a * a);
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

    int GetId();
    int GetFaction();
    Vector GetPos();
    bool IsAlive();
    bool IsInSight(Drone target);
    bool IsInScan(Drone target);
    double GetDist(Drone target);
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

int Drone::GetId()
{
    return id_;
}

int Drone::GetFaction()
{
    return faction_;
}

Vector Drone::GetPos()
{
    return pos_;
}

bool Drone::IsAlive()
{
    return is_alive_;
}

bool Drone::IsInSight(Drone target)
{
    Vector target_pos = target.GetPos();
    return direction_ * (target_pos - pos_) > 0;
}

bool Drone::IsInScan(Drone target)
{
    if (!IsInSight(target))
        return false;

    Vector relative_position = target.GetPos() - pos_;
    Vector l;
    l = OuterProduct(lift_, direction_);
    if (Norm(ProjectionToVector(relative_position, l)) <= lateral_scan_range_ && Norm(ProjectionToVector(relative_position, lift_)) <= vertical_scan_range_)
        return true;
    return false;
}

double Drone::GetDist(Drone target)
{
    return Norm(pos_ - target.GetPos());
}

void Drone::LockTarget(std::vector<Drone> &DroneList) // 选定目标
{
    /*
    若视野内无敌方阵营无人机，则无选定目标
    否则若上一时刻的目标仍位于视野内，则仍选定该目标
    否则若有敌机在雷达扫描范围内，则选取距离最近的，距离相同取编号最小
    否则选取取 min{|rx − Lx|, |rx + Lx|} + min{|ry − Hy|, |ry + Hy|} 最小的，相同则取编号最小

    在实现时我们把第二步检验放至第一步之前，以减少运算次数

    注意，在这里我们利用 target_ - 1 作为目标在DroneList中的下标，可能会存在鲁棒性问题，
    但是暂时没有更好的方式（除了暴力搜索和提前储存目标信息，都会造成复杂度增加），并且我们不会改变DroneList，我们也会尽量避开这种行为
    */

    if (target_ && IsInSight(DroneList[target_ - 1]))
        return;
    else
        target_ = 0;

    int drone_num = DroneList.size();
    bool flag = true;
    for (int i = 0; i < drone_num; i++)
    {
        if (!DroneList[i].IsAlive())
            continue;
        if (DroneList[i].GetFaction() == faction_)
            continue;
        if (IsInSight(DroneList[i]))
        {
            flag = false;
            break;
        }
    }
    if (flag)
    {
        target_ = 0;
        return;
    }

    // 运行到此处时应有 target_ == 0
    for (int i = 0; i < drone_num; i++)
    {
        if (!DroneList[i].IsAlive())
            continue;
        if (DroneList[i].GetFaction() == faction_)
            continue;
        if (!IsInSight(DroneList[i]))
            continue;
        if (IsInScan(DroneList[i]))
        {
            if (!target_)
                target_ = DroneList[i].id_;
            else if (GetDist(DroneList[i]) < GetDist(DroneList[target_ - 1]))
                target_ = DroneList[i].id_;
        }
    }
    if (target_)
        return;

    double tmp = 10000000; // 记录当前min...最小的是多少
    for (int i = 0; i < drone_num; i++)
    {
        if (!DroneList[i].IsAlive())
            continue;
        if (DroneList[i].GetFaction() == faction_)
            continue;

        double rx = Norm(ProjectionToVector(DroneList[i].GetPos() - pos_, OuterProduct(lift_, direction_)));
        double ry = Norm(ProjectionToVector(DroneList[i].GetPos() - pos_, lift_));
        if (std::min(abs(rx - lateral_scan_range_), abs(rx + lateral_scan_range_)) + std::min(abs(ry - vertical_scan_range_), abs(ry + vertical_scan_range_)) < tmp)
            target_ = DroneList[i].GetId();
    }

    return;
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