#pragma once

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <string>
#include <string.h>

#include "Robot.h"

#define TIME_RES_ERROR (5e-10) // 允許計算時間數值誤差的解析度，單位(s)

namespace Rbt
{
	class Traj_Base
	{
	public:
		std::vector<double> Time; // Time, 時間, 單位(s)
		VectorXdList Pos;         // Position, 位置, 單位為使用者自訂
		VectorXdList Vel;         // Velocity, 速度, 單位為使用者自訂
		VectorXdList Acc;         // Acceleration, 加速度, 單位為使用者自訂

	public:
		// 建構子
		Traj_Base();
		Traj_Base(unsigned int SizeConut); // 預先保留記憶體空間

		// 解構子
		~Traj_Base();
		
		void Reserved(unsigned int Reserved_Size);
		void Push_back(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel, const Eigen::VectorXd& acc, const double& time);
		void Clear();
	};

	class Traj_Data : public Traj_Base
	{
	public:
		// 經過點的時間、位置、速度、加速度
		std::vector<double> Via_Time; // Time, 時間, 單位(s)
		VectorXdList Via_Pos;         // Position, 位置, 單位為使用者自訂
		VectorXdList Via_Vel;         // Velocity, 速度, 單位為使用者自訂
		VectorXdList Via_Acc;         // Acceleration, 加速度, 單位為使用者自訂
		double TimeInterval;          // 時間間隔(預設為0), 單位(s)

	public:
		// 建構子
		Traj_Data(double _TimeInterval = 0);
		Traj_Data(unsigned int Via_SizeConut, unsigned int SizeConut, double _TimeInterval = 0); // 預先保留記憶體空間

		// 解構子
		~Traj_Data();
		
		
		void Via_Reserved(unsigned int Via_SizeConut);
		void Via_Push_back(const Eigen::VectorXd& via_pos, const Eigen::VectorXd& via_vel, const Eigen::VectorXd& via_acc, const double& via_time);
		void Via_Clear();
		bool Via_Check(); // 檢查Via_Time, Via_Pos, Via_Vel, Via_Acc大小；且 Via_Time 是否為TimeInterval的正整數倍，成功回傳true 否則回傳 false
	};

	// 讀取外部 Traj 檔案
	class Traj_InOut
	{
	private:
		bool  IsError;      // 判斷讀寫檔是否有錯
		char  ErrCode[512]; // 記錄 Last Error code
		FILE* f_ptr;        // 紀錄檔案結構指標

		std::vector<int> Seg_Num;  // 點到點之間要切的段數
		
		bool Read_File(unsigned int DOF);     // 正確回傳true，錯誤回傳false
		void Ignore_OneLine();                // 讀到註解 ==> 忽略一行
		bool Get_SegNum(int value, int Line); // 獲得 Seg_Num 的值，並檢查是否有錯(正確回傳true、反之false)
		
	public:
		Traj_Data Data; // 記錄 Via_Traj, 以及內插的 Traj

		// 建構子
		Traj_InOut();

		// 解構子
		~Traj_InOut();

		// 傳入路徑+檔名與需讀取的DOF，正確回傳true，錯誤回傳false (利用Get_LastError()可得到錯誤資訊)
		bool Load_File(const char* FileName, unsigned int DOF);

		// 將所有 Via point 線性內差
		bool Linear_Planning();						// 儲存在Data.Pos
		bool Linear_Planning(VectorXdList& q_Traj); // 儲存在外部 q_Traj

		// 獲得錯誤資訊
		const char* Get_LastError();
	};
}

namespace Rbt
{
	// 解五階多項式係數 S(t) = [1; t; t^2; t^3; t^4; t^5].transpose() * [a0; a1; a2; a3; a4; a5]
	// 輸出：Dest_Parameter[] Array Size: 6，係數由階數小到大冪次對應排序 a0~a5
	// 傳入：初始P0 [位置 速度 加速度 時間]，結束P1 [位置 速度 加速度 時間]
	// 成功回傳true，反之false
	bool Solve_Quintic(double* Dest_Parameter, double P0, double P0_d, double P0_dd, double T0, double P1, double P1_d, double P1_dd,  double T1);

	// 五階多項式軌跡規劃，成功回傳true，反之false
	bool Quintic_Planning(Traj_Data& Dest_Traj);
}