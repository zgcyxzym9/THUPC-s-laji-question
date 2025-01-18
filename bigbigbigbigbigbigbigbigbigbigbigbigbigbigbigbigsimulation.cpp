#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#define PI 3.141592653589793

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

bool operator==(const Vector a, const Vector b)
{
    return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}

bool operator!=(const Vector a, const Vector b)
{
    return !(a == b);
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

double GetAngle(const Vector a, const Vector b)
{
    return acos((a * b) / (Norm(a) * Norm(b)));
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
    bool IsValidMove(Vector target_pos);
    void GetDestination(std::vector<Drone> &DroneList);

private:
    int id_;
    int faction_;
    int target_;
    Vector pos_;
    Vector direction_;
    Vector lift_;
    Vector dest_pos_;
    Vector dest_direction_;
    Vector dest_lift_;
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

bool Drone::IsValidMove(Vector target_pos) // 判断目前到target_pos是否是合法运动
{
    /*
    注意：滚转率，俯仰率均不超过π/2
    如果目标方向和当前方向共线：特判同向/反向
    否则计算左手向和平面法线的夹角，选择更近的方向滚转；

    计算俯仰所需时间，升力线不可能旋转超过π/2，因此目标/当前方向叉乘位于左侧则拉杆，否则推杆

    计算移动所需时间，较为简单
    */

    if (target_pos == pos_)
        return true;

    double total_time = 0;
    Vector target_direction = target_pos - pos_;
    Vector zero_vector = {0, 0, 0};
    if (OuterProduct(target_direction, direction_) == zero_vector)
    {
        if (target_direction * direction_ < 0)
            return false;
    }
    else // 计算改变航向所需时间
    {
        Vector orthogonal_dir = OuterProduct(target_direction, direction_);
        double roll_angle = GetAngle(OuterProduct(lift_, direction_), orthogonal_dir);
        roll_angle = std::min(roll_angle, PI - roll_angle);
        total_time += roll_angle / roll_rate_;

        if (orthogonal_dir * OuterProduct(lift_, direction_) >= 0)
            total_time += GetAngle(target_direction, direction_) / pull_rate_;
        else
            total_time += GetAngle(target_direction, direction_) / push_rate_;
    }

    total_time += Norm(target_pos - pos_) / max_speed_;
    return total_time <= 1;
}

void Drone::GetDestination(std::vector<Drone> &DroneList)
{
    if (target_ == 0)
    {
        dest_pos_ = pos_;
        dest_direction_ = lift_;
        dest_lift_ = {-pos_.x, -pos_.y, -pos_.z};
        return;
    }

    Drone cur_best = *this;
    for (int i = -int(max_speed_); i <= int(max_speed_); i++)
        for (int j = -int(max_speed_); j <= int(max_speed_); j++)
            for (int k = -int(max_speed_); k <= int(max_speed_); k++)
            {
                Vector dest = {pos_.x + i, pos_.y + j, pos_.z + k};
                if (IsValidMove(dest))
                {
                    Drone dest_drone = *this;
                    dest_drone.pos_ = {pos_.x + i, pos_.y + j, pos_.z + k};
                    dest_drone.direction_ = dest - pos_;
                    /*
                    计算升力线方向：升力线与左手向和前向垂直，且左手向一定和前向垂直，所以可以用叉乘算出升力线方向；
                    如果该升力线方向和先前的升力线方向成钝角，那么将其反向。左手向即原方向和现方向的外积，但这里我们并不考虑其正反。
                    需要特判方向不变。
                    */
                    if (GetAngle(dest_drone.direction_, direction_) == 0)
                        dest_drone.lift_ = lift_;
                    else
                    {
                        Vector left = OuterProduct(dest_drone.direction_, direction_);
                        dest_drone.lift_ = OuterProduct(left, dest_drone.direction_);
                        if (dest_drone.lift_ * lift_ < 0)
                            dest_drone.lift_ = {-dest_drone.lift_.x, -dest_drone.lift_.y, -dest_drone.lift_.z};
                    }

                    if (dest_drone.IsInSight(DroneList[target_ - 1])) // 1.1
                    {
                        if (Norm(DroneList[target_ - 1].pos_ - dest_drone.pos_) < Norm(DroneList[target_ - 1].pos_ - cur_best.pos_))
                        {
                            cur_best.pos_ = dest;
                            cur_best.direction_ = dest_drone.direction_;
                            cur_best.lift_ = dest_drone.lift_;
                        }
                        else if (Norm(DroneList[target_ - 1].pos_ - dest_drone.pos_) == Norm(DroneList[target_ - 1].pos_ - cur_best.pos_)) // 若有多个这样的位置
                        {
                            double rx = Norm(ProjectionToVector(DroneList[target_ - 1].GetPos() - dest_drone.pos_, OuterProduct(dest_drone.lift_, dest_drone.direction_)));
                            double ry = Norm(ProjectionToVector(DroneList[target_ - 1].GetPos() - dest_drone.pos_, dest_drone.lift_));
                            double brx = Norm(ProjectionToVector(DroneList[target_ - 1].GetPos() - cur_best.pos_, OuterProduct(cur_best.lift_, cur_best.direction_)));
                            double bry = Norm(ProjectionToVector(DroneList[target_ - 1].GetPos() - cur_best.pos_, cur_best.lift_));
                            if (dest_drone.IsInScan(DroneList[target_ - 1]))
                            {
                                if (!cur_best.IsInScan(DroneList[target_ - 1])) // 1.1.1
                                {
                                    cur_best.pos_ = dest;
                                    cur_best.direction_ = dest_drone.direction_;
                                    cur_best.lift_ = dest_drone.lift_;
                                }
                                else // 1.1.1.1
                                {
                                    if (sqrt(pow(rx, 2) + pow(ry, 2)) < sqrt(pow(brx, 2) + pow(bry, 2)))
                                    {
                                        cur_best.pos_ = dest;
                                        cur_best.direction_ = dest_drone.direction_;
                                        cur_best.lift_ = dest_drone.lift_;
                                    }
                                }
                            }
                            else if (!cur_best.IsInScan(DroneList[target_ - 1]))
                            {
                                if (std::min(abs(rx - lateral_scan_range_), abs(rx + lateral_scan_range_)) + std::min(abs(ry - vertical_scan_range_), abs(ry + vertical_scan_range_)) < std::min(abs(brx - lateral_scan_range_), abs(brx + lateral_scan_range_)) + std::min(abs(bry - vertical_scan_range_), abs(bry + vertical_scan_range_)))
                                {
                                    cur_best.pos_ = dest;
                                    cur_best.direction_ = dest_drone.direction_;
                                    cur_best.lift_ = dest_drone.lift_;
                                }
                            }
                        }
                    }
                    else // 1.2
                    {
                        if (!cur_best.IsInSight(DroneList[target_ - 1]) && Norm(dest - pos_ - max_speed_ * direction_) < Norm(cur_best.pos_ - pos_ - max_speed_ * direction_)) // 不确定：Vm*d中的d是之前还是之后？此处认为是之前
                        {
                            cur_best.pos_ = dest;
                            cur_best.direction_ = dest_drone.direction_;
                            cur_best.lift_ = dest_drone.lift_;
                        }
                    }
                } // IsValidMove
            }

    dest_pos_ = cur_best.pos_;
    dest_direction_ = cur_best.direction_;
    dest_lift_ = cur_best.lift_;
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