#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

struct Vector
{
	double x,y,z;
};

Vector operator - (const Vector &a, const Vector &b)
{
	Vector ans;
	ans.x=a.x-b.x;
	ans.y=a.y-b.y;
	ans.z=a.z-b.z;
	return ans;
}

int operator * (const Vector &a, const Vector &b)
{
	return a.x*b.x+a.y*b.y+a.z*b.z;
}

Vector operator ^ (const Vector &a, const Vector &b)
{
	Vector ans;
	ans.x = a.y * b.z - a.z * b.y;
	ans.y = a.z * b.x - a.x * b.z;
	ans.z = a.x * b.y - a.y * b.x;
	return ans;
}

struct MissileData
{
	Vector Pos;
	Vector Heading;
	double RollRate, Vm, SafeDist, ExplodeDist, LockAngle, MaxTime;
	int Origin;	//谁射的 
};

struct DroneData	//储存一架无人机的所有参数 
{
	Vector Pos;
	Vector Heading, Lift;
	double PitchUpRate, PitchDownRate, RollRate, Vm, LateralScanRange, VerticalScanRange;
	MissileData Missile;
	int Squad, MissileFlying;	//Squad 0或1; MissileFlying判断是否有发射的导弹 
	int Target, Alive;
}Drone[1005];

vector<int> ActiveDroneID;

vector<int> ActiveMissile;

int n,T;

double GetVectorLen(Vector a)
{
	return sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
};

double GetVectorAngle(Vector a, Vector b)
{
	return acos(a*b/(GetVectorLen(a)*GetVectorLen(b)));
};

double GetPosLen(Vector a, Vector b)
{
	double Diffx,Diffy,Diffz;
	Diffx = a.x-b.x;
	Diffy = a.y-b.y;
	Diffz = a.z-b.z;
	return sqrt(Diffx*Diffx+Diffy*Diffy+Diffz*Diffz); 
} 

void CobraManeuver(int CurID)
{
	Vector TempHeading;
	Vector TempLift;
	TempHeading = Drone[CurID].Heading;
	TempLift = Drone[CurID].Lift;
	Drone[CurID].Heading = TempLift;
	TempHeading.x *= -1;
	TempHeading.y *= -1;
	TempHeading.z *= -1;
	Drone[CurID].Lift = TempHeading;
	return; 
}

bool IsInSight(DroneData a, DroneData b)		//判断是否可以看见
{
	if(a.Heading * (b.Pos - a.Pos) > 0)
		return true;
	return false;
}

bool IsSearchable(DroneData a, DroneData b)
{
	if(!IsInSight(a, b))
		return false;
	
	Vector DronePos, TargetPos, Heading, Lift, Left;
	DronePos = a.Pos;
	TargetPos = b.Pos;
	Heading = a.Heading;
	Lift = a.Lift;
	Left = Heading ^ Lift;
	if(Heading.z == 0)
	{
		swap(Heading.x, Heading.z);
		swap(Lift.x, Lift.z);
		swap(Left.x, Left.z);
		if(Heading.z == 0)
		{
			swap(Heading.y, Heading.z);
			swap(Lift.y, Lift.z);
			swap(Left.y, Left.z);
		}
	}
	if(Heading.z * (Lift.x - Lift.z / Heading.z) == 0)
	{
		swap(Heading.x, Heading.y);
		swap(Lift.x, Lift.y);
		swap(Left.x, Left.y);
	}

	double DiffX, DiffY;

	DiffX = (TargetPos.y - DronePos.y - (Lift.y * (TargetPos.x - DronePos.x - Heading.x * (TargetPos.z - DronePos.z) / Heading.z) / (Lift.x - Lift.z / Heading.z)) - Heading.y * (TargetPos.z - DronePos.z) / Heading.z + (Heading.y * Lift.z * (TargetPos.x - DronePos.x - Heading.x * (TargetPos.z - DronePos.z) / Heading.z) / (Lift.x - Lift.z / Heading.z))) / (Left.y - (Lift.y * (Left.x - Left.z / Heading.z) / (Lift.x - Lift.z / Heading.z)) - Heading.y * Left.z / Heading.z + (Heading.y * Lift.z * (Left.x - Left.z / Heading.z))/(Heading.z * (Lift.x - Lift.z / Heading.z)));

}

void DroneSelTarget()	//无人机锁定目标 对应步骤1前半部分 
{
	int ActiveDroneNum = ActiveDroneID.size();
	for(int i=0;i<ActiveDroneNum;i++)
	{
		int CurID = ActiveDroneID[i];
		//先判断上一次的无人机是不是还能锁定住
		int LastTarget = Drone[CurID].Target;
		if(LastTarget && Drone[LastTarget].Alive && Drone[CurID].Heading * (Drone[CurID].Pos - Drone[LastTarget].Pos) > 0)
			continue;
		
		Drone[CurID].Target = 0;

		for(int j=0;j<ActiveDroneNum;j++)	//先看雷达扫的 
		{
			//判断是不是对面的
			int TgtID = ActiveDroneID[j];
			if(Drone[CurID].Squad == Drone[TgtID].Squad)
				continue;
			
			//然后判断雷达能不能扫到
			if(IsSearchable(Drone[CurID], Drone[j]))
		}
		
		if(Drone[CurID].Target == 0)
		{
			
		}
	}
	return;
}

void DroneSelDest()		//无人机选定飞行策略 对应步骤1后半部分 
{
	int ActiveDroneNum = ActiveDroneID.size();
	for(int i=0;i<ActiveDroneNum;i++)
	{
		int CurID = ActiveDroneID[i];
		if(Drone[CurID].Target)
		{
			
		}
		else	//若没有目标 
		{
			CobraManeuver(CurID);
		}
	}
	return;
}

int main()
{
	cin>>n>>T;
	for(int i=1;i<=n;i++)
	{
		Drone[i].Squad = 0;
		cin>>Drone[i].Pos.x>>Drone[i].Pos.y>>Drone[i].Pos.z;
		cin>>Drone[i].Heading.x>>Drone[i].Heading.y>>Drone[i].Heading.z;
		cin>>Drone[i].Lift.x>>Drone[i].Lift.y>>Drone[i].Lift.z;
		cin>>Drone[i].PitchUpRate>>Drone[i].PitchDownRate>>Drone[i].RollRate>>Drone[i].Vm>>Drone[i].LateralScanRange>>Drone[i].VerticalScanRange;
		cin>>Drone[i].Missile.RollRate>>Drone[i].Missile.Vm>>Drone[i].Missile.SafeDist>>Drone[i].Missile.ExplodeDist>>Drone[i].Missile.LockAngle>>Drone[i].Missile.MaxTime;
		Drone[i].Alive = 1;
		ActiveDroneID.push_back(i);
	}
	for(int i=1;i<=n;i++)
	{
		Drone[i+n].Squad = 1;
		cin>>Drone[i+n].Pos.x>>Drone[i+n].Pos.y>>Drone[i+n].Pos.z;
		cin>>Drone[i+n].Heading.x>>Drone[i+n].Heading.y>>Drone[i+n].Heading.z;
		cin>>Drone[i+n].Lift.x>>Drone[i+n].Lift.y>>Drone[i+n].Lift.z;
		cin>>Drone[i+n].PitchUpRate>>Drone[i+n].PitchDownRate>>Drone[i+n].RollRate>>Drone[i+n].Vm>>Drone[i+n].LateralScanRange>>Drone[i+n].VerticalScanRange;
		cin>>Drone[i+n].Missile.RollRate>>Drone[i+n].Missile.Vm>>Drone[i+n].Missile.SafeDist>>Drone[i+n].Missile.ExplodeDist>>Drone[i+n].Missile.LockAngle>>Drone[i+n].Missile.MaxTime;
		Drone[i+n].Alive = 1;
		ActiveDroneID.push_back(i+n);
	}
	
	//下面开始模拟
	for(int i=1;i<=T;i++)
	{
		DroneSelTarget();
		DroneSelDest();
		DroneFireMissile();
		MissileCalc();
	} 
	return 0;
}
