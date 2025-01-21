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

Vector operator/(const Vector a, const double b)
{
    return {a.x / b, a.y / b, a.z / b};
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

Vector ProjectionToVector(const Vector a, const Vector b) // 从a到b投影
{
    Vector ans;
    ans = ((a * b) / (b * b)) * b;
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

double GetDistToSegment(const Vector endpoint_a, const Vector endpoint_b, const Vector dot)
{
    Vector tmp_segment = dot - endpoint_a;
    Vector foot = dot + tmp_segment - ProjectionToVector(tmp_segment, endpoint_b - endpoint_a);
    if ((endpoint_a - foot) * (endpoint_b - foot) > 0)
        return std::min(Norm(dot - endpoint_a), Norm(dot - endpoint_b));
    else
        return Norm(foot - dot);
}

struct Event
{
    int destroyed_;
    int destroyer_;
    int method_;
};

bool SortEvent(Event a, Event b)
{
    if (a.method_ != b.method_)
        return a.method_ < b.method_;
    if (a.destroyed_ != b.destroyed_)
        return a.destroyed_ < b.destroyed_;
    return a.destroyer_ < b.destroyer_;
}

class Drone;

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
        time_elapsed_ = 0;
        max_speed_ = max_speed;
        safe_dist_ = safe_dist;
        explode_dist_ = explode_dist;
        max_lock_angle_ = max_lock_angle;
        nav_time_ = nav_time;
    }
    Missile();
    bool DestIsInScan(Drone target);
    bool IsInScan(Drone target);
    bool IsValidMove(Vector target_pos);
    void MoveMissile(Vector dest, std::vector<Drone> &DroneList, std::vector<Event> &EventLog);
    void GetDestination(std::vector<Drone> &DroneList, std::vector<Event> &EventLog);
    void ClearMissile();

private:
    int id_;
    int faction_;
    int target_;
    int time_elapsed_;
    Vector pos_;
    Vector direction_;
    double yaw_rate_, max_speed_, safe_dist_, explode_dist_, max_lock_angle_, nav_time_;
    bool is_launched_;
    bool is_active_;
    bool is_locked_;
    bool is_detonated_;
    friend class Drone;
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
    Vector GetDestPos();
    bool IsAlive();
    bool IsInSight(Drone target);
    bool IsInScan(Drone target);
    double GetDist(Drone target);
    void LockTarget(std::vector<Drone> &DroneList); // 选定目标
    bool IsValidMove(Vector target_pos);
    void GetDestination(std::vector<Drone> &DroneList);
    void FireMissile(Vector target_pos);
    void CheckFireMissile(std::vector<Drone> &DroneList);
    void Destroyed();
    void ClearMissile();
    void MoveMissile(std::vector<Drone> &DroneList, std::vector<Event> &EventLog);
    void MoveDrone(std::vector<Drone> &DroneList, std::vector<Event> &EventLog);
    void UpdateMissileLock(std::vector<Drone> &DroneList);
    void ValidateMissile(std::vector<Drone> &DroneList);
    void ActiveMissile();

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

Vector Drone::GetDestPos()
{
    return dest_pos_;
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
    /*if(id_ == 19 && target_ == 1)
    {
        std::cout<<id_<<" rel pos: "<<relative_position.x<<" "<<relative_position.y<<" "<<relative_position.z<<std::endl;
        std::cout<<"rx = "<<Norm(ProjectionToVector(relative_position, l))<<" ry = "<<Norm(ProjectionToVector(relative_position, lift_))<<std::endl;
        std::cout<<lift_.x<<" "<<lift_.y<<" "<<lift_.z<<" "<<direction_.x<<" "<<direction_.y<<" "<<direction_.z<<std::endl;
        std::cout<<l.x<<" "<<l.y<<" "<<l.z<<std::endl;
        std::cout<<std::endl;
    }*/
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

    //if(id_ == 1)
    //    std::cout<<"target: "<<target_<<std::endl;
    if (target_ && DroneList[target_ - 1].is_alive_ &&IsInSight(DroneList[target_ - 1]))
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
            //if(id_ == 1)
            //    std::cout<<DroneList[i].id_<<" is in sight"<<std::endl;
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
    //if(id_ == 1)
    //    std::cout<<"target2: "<<target_<<std::endl;
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
            //if(id_ == 1)
            //    std::cout<<"candidate: "<<DroneList[i].id_<<std::endl;
            if (!target_)
                target_ = DroneList[i].id_;
            else if (GetDist(DroneList[i]) < GetDist(DroneList[target_ - 1]))
                target_ = DroneList[i].id_;
        }
    }
    if (target_)
        return;
    
    //if(id_ == 19)
    //    std::cout<<"no candidate"<<std::endl;

    double tmp = 10000000; // 记录当前min...最小的是多少
    for (int i = 0; i < drone_num; i++)
    {
        if (!DroneList[i].IsAlive())
            continue;
        if (DroneList[i].GetFaction() == faction_)
            continue;
        if (!IsInSight(DroneList[i]))
            continue;

        double rx = Norm(ProjectionToVector(DroneList[i].GetPos() - pos_, OuterProduct(lift_, direction_)));
        double ry = Norm(ProjectionToVector(DroneList[i].GetPos() - pos_, lift_));
        //if(id_ == 19)
        //    std::cout<<std::min(abs(rx - lateral_scan_range_), abs(rx + lateral_scan_range_)) + std::min(abs(ry - vertical_scan_range_), abs(ry + vertical_scan_range_))<<std::endl;
        if (std::min(abs(rx - lateral_scan_range_), abs(rx + lateral_scan_range_)) + std::min(abs(ry - vertical_scan_range_), abs(ry + vertical_scan_range_)) < tmp)
        {
            target_ = DroneList[i].GetId();
            tmp = std::min(abs(rx - lateral_scan_range_), abs(rx + lateral_scan_range_)) + std::min(abs(ry - vertical_scan_range_), abs(ry + vertical_scan_range_));
        }
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
        return false;

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
    //if(id_ == 1 && total_time <= 1)
    //    std::cout<<total_time<<std::endl;
    return total_time <= 1;
}

void Drone::GetDestination(std::vector<Drone> &DroneList)
{
    /*if(id_ == 1)
    {
        std::cout<<"getting destination for "<<id_<<", target_ is "<<target_<<std::endl;
        std::cout<<"initial position "<<pos_.x<<" "<<pos_.y<<" "<<pos_.z<<" direction "<<direction_.x<<" "<<direction_.y<<" "<<direction_.z<<std::endl;
    }*/
    if (target_ == 0)
    {
        //if(id_ == 1)
        //    std::cout<<"doing cobra"<<std::endl;
        dest_pos_ = pos_;
        dest_direction_ = lift_;
        dest_lift_ = {-direction_.x, -direction_.y, -direction_.z};
        return;
    }

    Drone cur_best = *this;
    for (int i = -int(max_speed_); i <= int(max_speed_); i++)
        for (int j = -int(max_speed_); j <= int(max_speed_); j++)
            for (int k = -int(max_speed_); k <= int(max_speed_); k++)
            {
                if(!i && !j && !k)
                    continue;
                Vector dest = {pos_.x + i, pos_.y + j, pos_.z + k};
                //if(id_ == 1)
                //    std::cout<<pos_.x+i<<" "<<pos_.y+j<<" "<<pos_.z+k<<std::endl;
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
                    if (std::abs(GetAngle(dest_drone.direction_, direction_)) <= 1e-7)
                        dest_drone.lift_ = lift_;
                    else
                    {
                        Vector left = OuterProduct(dest_drone.direction_, direction_);
                        dest_drone.lift_ = OuterProduct(left, dest_drone.direction_);
                        if (dest_drone.lift_ * lift_ < 0)
                            dest_drone.lift_ = {-dest_drone.lift_.x, -dest_drone.lift_.y, -dest_drone.lift_.z};
                    }

                    //if(id_ == 19)
                    //    std::cout<<dest.x<<" "<<dest.y<<" "<<dest.z<<" is valid move, rel dist = "<<Norm(dest - DroneList[target_ - 1].pos_)<<", is in sight = "<<dest_drone.IsInSight(DroneList[target_ - 1])<<std::endl;

                    if(cur_best.pos_ == pos_)
                    {
                        cur_best.pos_ = dest;
                        cur_best.direction_ = dest_drone.direction_;
                        cur_best.lift_ = dest_drone.lift_;
                    }

                    if (dest_drone.IsInSight(DroneList[target_ - 1])) // 1.1
                    {
                        if(!cur_best.IsInSight(DroneList[target_ - 1]))
                        {
                            cur_best.pos_ = dest;
                            cur_best.direction_ = dest_drone.direction_;
                            cur_best.lift_ = dest_drone.lift_;
                            continue;
                        }
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

void Drone::FireMissile(Vector target_pos)
{
    //if(id_ == 19 || id_ == 1)
    //    std::cout<<id_<<" fired"<<std::endl;
    missile_.is_launched_ = true;
    missile_.is_active_ = false;
    missile_.pos_ = pos_;
    missile_.direction_ = (target_pos - pos_) / Norm(target_pos - pos_);
    missile_.target_ = target_;
    missile_.is_locked_ = true;
    missile_.is_detonated_ = false;
    missile_.time_elapsed_ = 0;
}

void Drone::CheckFireMissile(std::vector<Drone> &DroneList)
{
    //if(id_ == 19)
    //    std::cout<<"target of "<<id_<<" is "<<target_<<", missile launched: "<<missile_.is_launched_<<", is in scan: "<<IsInScan(DroneList[target_ - 1])<<", is in sight: "<<IsInSight(DroneList[target_ - 1])<<std::endl;
    if (target_ && !missile_.is_launched_ && IsInScan(DroneList[target_ - 1]))
        FireMissile(DroneList[target_ - 1].pos_);
    return;
}

bool Missile::DestIsInScan(Drone target)
{
    Vector target_dest = target.GetDestPos();
    return direction_ * (target_dest - pos_) > 0 && GetAngle(direction_, target_dest - pos_) <= max_lock_angle_;
}

bool Missile::IsInScan(Drone target)
{
    Vector target_pos = target.GetPos();
    return direction_ * (target_pos - pos_) > 0 && GetAngle(direction_, target_pos - pos_) <= max_lock_angle_;
}

bool Missile::IsValidMove(Vector target_pos)
{
    if (target_pos == pos_)
        return false;

    Vector target_direction = target_pos - pos_;
    double total_time = GetAngle(target_direction, direction_) / yaw_rate_ + Norm(target_pos - pos_) / max_speed_;
    return total_time <= 1;
}

void Missile::ClearMissile()
{
    is_launched_ = false;
    is_active_ = false;
    is_detonated_ = false;
}

void Missile::MoveMissile(Vector dest, std::vector<Drone> &DroneList, std::vector<Event> &EventLog)
{
    /*
    需要完成的功能：
        若已激活则摧毁无人机：若激活，在唯一过程中与该导弹的距离不大于空爆距离；位移结束后与某无人机位置重合（无人机位移之前）；
        移动导弹位置；
        若可以空爆则空爆；
        若未激活在移动后检测位置

    摧毁无人机方式：在eventlist里面添加，结束后统一摧毁
    */

    if (is_active_)
    {
        for (int i = 0; i < DroneList.size(); i++)
        {
            if (!DroneList[i].IsAlive())
                continue;
            if (GetDistToSegment(pos_, dest, DroneList[i].GetPos()) < explode_dist_)
            {
                is_detonated_ = true;
                EventLog.push_back({DroneList[i].GetId(), id_, 1});
            }
        }
    }

    direction_ = dest - pos_;
    pos_ = dest;

    if (!is_active_)
    {
        for (int i = 0; i < DroneList.size(); i++)  //collision check
        {
            if (!DroneList[i].IsAlive())
                continue;
            if (pos_ == DroneList[i].GetPos())
            {
                is_detonated_ = true;
                EventLog.push_back({DroneList[i].GetId(), id_, 1});
            }
        }
    }
    //if(id_ == 1)
    //    std::cout<<"missile of "<<id_<<" moved to "<<dest.x<<" "<<dest.y<<" "<<dest.z<<std::endl;
    return;
}

void Missile::GetDestination(std::vector<Drone> &DroneList, std::vector<Event> &EventLog)
{
    Missile cur_best = *this;
    //std::cout<<"initial value for "<<id_<<" : "<<cur_best.pos_.x<<" "<<cur_best.pos_.y<<" "<<cur_best.pos_.z<<" "<<cur_best.direction_.x<<" "<<cur_best.direction_.y<<" "<<cur_best.direction_.z<<std::endl;
    if (is_locked_)
    {
        Vector target_dest = DroneList[target_ - 1].GetDestPos();
        if (IsValidMove(target_dest) && target_dest != pos_)
        {
            MoveMissile(target_dest, DroneList, EventLog);
            return;
        }
        else
        {
            for (int i = -int(max_speed_); i <= int(max_speed_); i++)
                for (int j = -int(max_speed_); j <= int(max_speed_); j++)
                    for (int k = -int(max_speed_); k <= int(max_speed_); k++)
                    {
                        if(!i && !j && !k)
                            continue;
                        Vector dest = {pos_.x + i, pos_.y + j, pos_.z + k};
                        Missile dest_missile = *this;
                        dest_missile.pos_ = dest;
                        dest_missile.direction_ = dest - pos_;
                        if(!IsValidMove(dest))
                            continue;

                        //std::cout<<"there is a valid for "<<id_<<std::endl;
                        if(cur_best.pos_ == pos_)
                        {
                            cur_best.pos_ = dest;
                            cur_best.direction_ = dest - pos_;
                        }

                        if (dest_missile.DestIsInScan(DroneList[target_ - 1])) // 否则导弹会移到能使敌机位移后的位置处于锁定范围内的位置
                        {
                            if (!cur_best.DestIsInScan(DroneList[target_ - 1]))
                            {
                                cur_best.pos_ = dest;
                                cur_best.direction_ = dest - pos_;
                            }
                            else if (Norm(dest - target_dest) < Norm(cur_best.pos_ - target_dest)) //|q-q'|最小
                            {
                                cur_best.pos_ = dest;
                                cur_best.direction_ = dest - pos_;
                            }
                            else if (Norm(dest - target_dest) == Norm(cur_best.pos_ - target_dest))
                            {
                                if (GetAngle(dest_missile.direction_, target_dest - dest) < GetAngle(cur_best.direction_, target_dest - cur_best.pos_)) // 锁定角最小
                                {
                                    cur_best.pos_ = dest;
                                    cur_best.direction_ = dest - pos_;
                                }
                            }
                        }
                        else if (!cur_best.DestIsInScan(DroneList[target_ - 1]) && Norm(dest - pos_ - max_speed_ * direction_) < Norm(cur_best.pos_ - pos_ - max_speed_ * direction_)) // 若不存在这样的位置
                        {
                            cur_best.pos_ = dest;
                            cur_best.direction_ = dest - pos_;
                        }
                    }
        }
    }   //is_locked_
    else // 若导弹脱锁
    {
        for (int i = -int(max_speed_); i <= int(max_speed_); i++)
            for (int j = -int(max_speed_); j <= int(max_speed_); j++)
                for (int k = -int(max_speed_); k <= int(max_speed_); k++)
                {
                    Vector dest = {pos_.x + i, pos_.y + j, pos_.z + k};
                    if(dest == pos_)
                        continue;
                    if(!IsValidMove(dest))
                        continue;
                    if(cur_best.pos_ == pos_)
                    {
                        cur_best.pos_ = dest;
                        cur_best.direction_ = dest - pos_;
                    }
                    if (Norm(dest - pos_ - max_speed_ * direction_) < Norm(cur_best.pos_ - pos_ - max_speed_ * direction_))
                    {
                        cur_best.pos_ = dest;
                        cur_best.direction_ = dest - pos_;
                    }
                }
    }

    MoveMissile(cur_best.pos_, DroneList, EventLog);
    return;
}

void ProcessEvent(std::vector<Drone> &DroneList, std::vector<Event> &EventLog)
{
    for (int i = 0; i < EventLog.size(); i++)
    {
        DroneList[EventLog[i].destroyed_ - 1].Destroyed();
        if (EventLog[i].method_ == 1 || EventLog[i].method_ == 2)
            DroneList[EventLog[i].destroyer_ - 1].ClearMissile();
    }
    return;
}

void Drone::Destroyed()
{
    is_alive_ = false;
}

void Drone::ClearMissile()
{
    missile_.ClearMissile();
}

void Drone::MoveDrone(std::vector<Drone> &DroneList, std::vector<Event> &EventLog)
{
    for (int i = 0; i < DroneList.size(); i++)
    {
        if (DroneList[i].missile_.is_active_ == false && DroneList[i].missile_.is_launched_ == true)
        {
            if (dest_pos_ == DroneList[i].missile_.pos_)
            {
                EventLog.push_back({id_, DroneList[i].GetId(), 2});
                DroneList[i].missile_.is_detonated_ = true;
            }
            continue;
        }
        if(DroneList[i].missile_.is_active_ == true && DroneList[i].missile_.is_launched_ == true)
            if (GetDistToSegment(pos_, dest_pos_, DroneList[i].missile_.pos_) < DroneList[i].missile_.explode_dist_)
            {
                EventLog.push_back({id_, DroneList[i].GetId(), 2});
                DroneList[i].missile_.is_detonated_ = true;
            }
    }
    //if(id_ == 19 || id_ == 1)
    //    std::cout<<id_<<" moved to "<<dest_pos_.x<<" "<<dest_pos_.y<<" "<<dest_pos_.z<<", dest_direction_ is "<<dest_direction_.x<<" "<<dest_direction_.y<<" "<<dest_direction_.z<<", dest_lift_ is  "<<dest_lift_.x<<" "<<dest_lift_.y<<" "<<dest_lift_.z<<std::endl;
    pos_ = dest_pos_;
    direction_ = dest_direction_;
    lift_ = dest_lift_;
    return;
}

void Drone::MoveMissile(std::vector<Drone> &DroneList, std::vector<Event> &EventLog)
{
    if(!missile_.is_launched_)
        return;
    missile_.GetDestination(DroneList, EventLog);
}

void CheckCollision(std::vector<Drone> &DroneList, std::vector<Event> &EventLog)
{
    for (int i = 0; i < DroneList.size(); i++)
    {
        if (!DroneList[i].IsAlive())
            continue;

        for (int j = 0; j < DroneList.size(); j++)
        {
            if(!DroneList[j].IsAlive())
                continue;
            if (i == j)
                continue;
            if (DroneList[i].GetPos() == DroneList[j].GetPos())
            {
                EventLog.push_back({DroneList[i].GetId(), DroneList[j].GetId(), 3});
                DroneList[i].Destroyed();
                DroneList[j].Destroyed();
            }
        }
    }
    return;
}

void Drone::UpdateMissileLock(std::vector<Drone> &DroneList)
{
    if (!missile_.is_launched_)
        return;
    if (!DroneList[target_ - 1].IsAlive() || !missile_.IsInScan(DroneList[target_ - 1]))
        missile_.is_locked_ = false;
    return;
}

void Drone::ValidateMissile(std::vector<Drone> &DroneList)
{
    if (!missile_.is_launched_)
        return;

    if (missile_.time_elapsed_ >= missile_.nav_time_)
    {
        missile_.ClearMissile();
        return;
    }

    if (!missile_.is_locked_ && missile_.is_active_)
    {
        missile_.ClearMissile();
        return;
    }

    if(missile_.is_detonated_)
    {
        missile_.ClearMissile();
        return;
    }

    missile_.time_elapsed_++;
}

void Drone::ActiveMissile()
{
    if(!missile_.is_launched_)
        return;
    if (!is_alive_ || Norm(missile_.pos_ - pos_) > missile_.safe_dist_)
        missile_.is_active_ = true;
    return;
}

int main()
{
    freopen("bigsimulation.in", "rb", stdin);
    freopen("bigsimulation.out", "wb", stdout);
    int n, T;
    std::cin >> n >> T;
    std::vector<Drone> DroneList;
    std::vector<Event> EventLog;

    for (int i = 1; i <= 2 * n; i++)
    {
        Vector pos, dir, lift;
        double pullr, pushr, rr, ms, lsr, vsr;
        std::cin >> pos.x >> pos.y >> pos.z >> dir.x >> dir.y >> dir.z >> lift.x >> lift.y >> lift.z >> pullr >> pushr >> rr >> ms >> lsr >> vsr;
        double myr, mms, msd, med, mmla, mnt;
        std::cin >> myr >> mms >> msd >> med >> mmla >> mnt;
        DroneList.push_back(Drone(i, int((i - 1) / n), pos, dir, lift, pullr, pushr, rr, ms, lsr, vsr, myr, mms, msd, med, mmla, mnt));
    }

    for (int i = 1; i <= T; i++)
    {
        for (int j = 0; j < 2 * n; j++) // subtask 1
        {
            if (DroneList[j].IsAlive())
            {
                DroneList[j].LockTarget(DroneList);
                DroneList[j].GetDestination(DroneList);
            }
        }

        for (int j = 0; j < 2 * n; j++) // subtask 2
        {
            if (DroneList[j].IsAlive())
                DroneList[j].CheckFireMissile(DroneList);
        }

        for (int j = 0; j < 2 * n; j++) // subtask 3
        {
            DroneList[j].MoveMissile(DroneList, EventLog);
        }

        ProcessEvent(DroneList, EventLog); // subtask 4

        for (int j = 0; j < 2 * n; j++) // subtask 5
        {
            if (DroneList[j].IsAlive())
                DroneList[j].MoveDrone(DroneList, EventLog);
        }
        for (int j = 0; j < 2 * n; j++)
        {
            DroneList[j].UpdateMissileLock(DroneList);
        }

        ProcessEvent(DroneList, EventLog); // subtask 6

        CheckCollision(DroneList, EventLog); // subtask 7

        for (int j = 0; j < 2 * n; j++) // subtask 8
        {
            DroneList[j].ValidateMissile(DroneList);
        }

        for (int j = 0; j < 2 * n; j++) // subtask 9
        {
            DroneList[j].ActiveMissile();
        }

        // output
        std::sort(EventLog.begin(), EventLog.end(), SortEvent);
        int cnt1 = 0, cnt2 = 0, cnt3 = 0;
        for (int i = 0; i < EventLog.size(); i++)
        {
            if (EventLog[i].method_ == 1 && (i == 0 || EventLog[i].destroyed_ != EventLog[i-1].destroyed_))
                cnt1++;
            else if (EventLog[i].method_ == 2 && (i == 0 || EventLog[i].destroyed_ != EventLog[i-1].destroyed_))
                cnt2++;
            else if (EventLog[i].method_ == 3 && (i == 0 || EventLog[i].destroyed_ != EventLog[i-1].destroyed_))
                cnt3++;
        }
        std::cout << cnt1 << " " << cnt2 << " " << cnt3 << std::endl;

        int idx = 0;
        int tmp_id = 2 * n + 1;
        std::vector<int> tmp_list;
        tmp_list.empty();
        while (idx < EventLog.size())
        {

            if (EventLog[idx].destroyed_ != tmp_id)
            {
                if (!tmp_list.empty())
                {
                    if(EventLog[idx - 1].method_ == 3)
                        std::cout << tmp_list.size() + 1 << " " << tmp_id << " ";
                    else
                        std::cout << tmp_id << " " << tmp_list.size() << " ";
                    for (int i = 0; i < tmp_list.size(); i++)
                        std::cout << tmp_list[i] << " ";
                    std::cout << std::endl;
                }
                tmp_id = EventLog[idx].destroyed_;
                tmp_list.clear();
            }

            tmp_list.push_back(EventLog[idx].destroyer_);
            idx++;
        }
        if (!tmp_list.empty())
        {
            if(EventLog[idx - 1].method_ == 3)
                std::cout << tmp_list.size() + 1 << " " << tmp_id << " ";
            else
                std::cout << tmp_id << " " << tmp_list.size() << " ";
            for (int i = 0; i < tmp_list.size(); i++)
                std::cout << tmp_list[i] << " ";
            std::cout << std::endl;
        }
        EventLog.clear();
        //std::cout<<"end of tick"<<std::endl<<std::endl;
    }

    return 0;
}