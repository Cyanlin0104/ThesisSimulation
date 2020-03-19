#pragma once

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <string>
#include <string.h>

#include "Robot.h"

#define TIME_RES_ERROR (5e-10) // ���\�p��ɶ��ƭȻ~�t���ѪR�סA���(s)

namespace Rbt
{
	class Traj_Base
	{
	public:
		std::vector<double> Time; // Time, �ɶ�, ���(s)
		VectorXdList Pos;         // Position, ��m, ��쬰�ϥΪ̦ۭq
		VectorXdList Vel;         // Velocity, �t��, ��쬰�ϥΪ̦ۭq
		VectorXdList Acc;         // Acceleration, �[�t��, ��쬰�ϥΪ̦ۭq

	public:
		// �غc�l
		Traj_Base();
		Traj_Base(unsigned int SizeConut); // �w���O�d�O����Ŷ�

		// �Ѻc�l
		~Traj_Base();
		
		void Reserved(unsigned int Reserved_Size);
		void Push_back(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel, const Eigen::VectorXd& acc, const double& time);
		void Clear();
	};

	class Traj_Data : public Traj_Base
	{
	public:
		// �g�L�I���ɶ��B��m�B�t�סB�[�t��
		std::vector<double> Via_Time; // Time, �ɶ�, ���(s)
		VectorXdList Via_Pos;         // Position, ��m, ��쬰�ϥΪ̦ۭq
		VectorXdList Via_Vel;         // Velocity, �t��, ��쬰�ϥΪ̦ۭq
		VectorXdList Via_Acc;         // Acceleration, �[�t��, ��쬰�ϥΪ̦ۭq
		double TimeInterval;          // �ɶ����j(�w�]��0), ���(s)

	public:
		// �غc�l
		Traj_Data(double _TimeInterval = 0);
		Traj_Data(unsigned int Via_SizeConut, unsigned int SizeConut, double _TimeInterval = 0); // �w���O�d�O����Ŷ�

		// �Ѻc�l
		~Traj_Data();
		
		
		void Via_Reserved(unsigned int Via_SizeConut);
		void Via_Push_back(const Eigen::VectorXd& via_pos, const Eigen::VectorXd& via_vel, const Eigen::VectorXd& via_acc, const double& via_time);
		void Via_Clear();
		bool Via_Check(); // �ˬdVia_Time, Via_Pos, Via_Vel, Via_Acc�j�p�F�B Via_Time �O�_��TimeInterval������ƭ��A���\�^��true �_�h�^�� false
	};

	// Ū���~�� Traj �ɮ�
	class Traj_InOut
	{
	private:
		bool  IsError;      // �P�_Ū�g�ɬO�_����
		char  ErrCode[512]; // �O�� Last Error code
		FILE* f_ptr;        // �����ɮ׵��c����

		std::vector<int> Seg_Num;  // �I���I�����n�����q��
		
		bool Read_File(unsigned int DOF);     // ���T�^��true�A���~�^��false
		void Ignore_OneLine();                // Ū����� ==> �����@��
		bool Get_SegNum(int value, int Line); // ��o Seg_Num ���ȡA���ˬd�O�_����(���T�^��true�B�Ϥ�false)
		
	public:
		Traj_Data Data; // �O�� Via_Traj, �H�Τ����� Traj

		// �غc�l
		Traj_InOut();

		// �Ѻc�l
		~Traj_InOut();

		// �ǤJ���|+�ɦW�P��Ū����DOF�A���T�^��true�A���~�^��false (�Q��Get_LastError()�i�o����~��T)
		bool Load_File(const char* FileName, unsigned int DOF);

		// �N�Ҧ� Via point �u�ʤ��t
		bool Linear_Planning();						// �x�s�bData.Pos
		bool Linear_Planning(VectorXdList& q_Traj); // �x�s�b�~�� q_Traj

		// ��o���~��T
		const char* Get_LastError();
	};
}

namespace Rbt
{
	// �Ѥ����h�����Y�� S(t) = [1; t; t^2; t^3; t^4; t^5].transpose() * [a0; a1; a2; a3; a4; a5]
	// ��X�GDest_Parameter[] Array Size: 6�A�Y�ƥѶ��Ƥp��j���������Ƨ� a0~a5
	// �ǤJ�G��lP0 [��m �t�� �[�t�� �ɶ�]�A����P1 [��m �t�� �[�t�� �ɶ�]
	// ���\�^��true�A�Ϥ�false
	bool Solve_Quintic(double* Dest_Parameter, double P0, double P0_d, double P0_dd, double T0, double P1, double P1_d, double P1_dd,  double T1);

	// �����h�����y��W���A���\�^��true�A�Ϥ�false
	bool Quintic_Planning(Traj_Data& Dest_Traj);
}