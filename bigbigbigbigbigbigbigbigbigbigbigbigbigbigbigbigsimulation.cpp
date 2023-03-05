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

bool IsInSight(DroneData a,DroneData b)
{
	if(a.Heading * (b.Pos - a.Pos) > 0)
		return true;
	return false;
}

//bool IsSearchable(DroneData a, DroneData b)
//{
//	if(IsInSight(a,b) && )
//}

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

void DroneSelTarget()
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
			
			
		}
		
		if(Drone[CurID].Target == 0)
		{
			
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
	} 
	return 0;
}
