#include "Traj_Planning.h"

using namespace Rbt;

#pragma region Traj_Base ��@

Traj_Base::Traj_Base()
{
}

Traj_Base::Traj_Base(unsigned int SizeConut)
{
	Time.reserve(SizeConut);
	Pos.reserve(SizeConut);
	Vel.reserve(SizeConut);
	Acc.reserve(SizeConut);
}

Traj_Base::~Traj_Base()
{

}

void Traj_Base::Reserved(unsigned int Reserved_Size)
{
	Time.reserve(Reserved_Size);
	Pos.reserve(Reserved_Size);
	Vel.reserve(Reserved_Size);
	Acc.reserve(Reserved_Size);
}

void Traj_Base::Push_back(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel, const Eigen::VectorXd& acc, const double& time)
{
	Time.push_back(time);
	Pos.push_back(pos);
	Vel.push_back(vel);
	Acc.push_back(acc);
}

void Traj_Base::Clear()
{
	Time.clear();
	Pos.clear();
	Vel.clear();
	Acc.clear();
}

#pragma endregion

#pragma region Traj_Data ��@

Traj_Data::Traj_Data(double _TimeInterval /*=0*/)
	      :Traj_Base()
{
	TimeInterval = _TimeInterval;
}

Traj_Data::Traj_Data(unsigned int Via_SizeConut, unsigned int SizeConut, double _TimeInterval /*=0*/)
	      :Traj_Base(SizeConut)
{
	Via_Time.reserve(Via_SizeConut);
	Via_Pos.reserve(Via_SizeConut);
	Via_Vel.reserve(Via_SizeConut);
	Via_Acc.reserve(Via_SizeConut);
	
	TimeInterval = _TimeInterval;
}

Traj_Data::~Traj_Data()
{

}

void Traj_Data::Via_Reserved(unsigned int Via_SizeConut)
{
	Via_Time.reserve(Via_SizeConut);
	Via_Pos.reserve(Via_SizeConut);
	Via_Vel.reserve(Via_SizeConut);
	Via_Acc.reserve(Via_SizeConut);
}

void Traj_Data::Via_Push_back(const Eigen::VectorXd& via_pos, const Eigen::VectorXd& via_vel, const Eigen::VectorXd& via_acc, const double& via_time)
{	
	Via_Time.push_back(via_time);
	Via_Pos.push_back(via_pos);
	Via_Vel.push_back(via_vel);
	Via_Acc.push_back(via_acc);
}

void Traj_Data::Via_Clear()
{
	Via_Time.clear();
	Via_Pos.clear();
	Via_Vel.clear();
	Via_Acc.clear();

	TimeInterval = 0;
}

bool Traj_Data::Via_Check()
{
	const unsigned int Size = Via_Time.size();
	
	// �ܤֻݨ�ӳq�L�I
	if(Size < 2)
		return false;

	// std::vector::size() �ҭn�ۦP
	if( !(Via_Pos.size() == Size && Via_Vel.size() == Size && Via_Acc.size() == Size && TimeInterval > TIME_RES_ERROR) )
		return false;

	// DOF = Eigen::VectorXd::rows() �ҭn�ۦP �B DOF >= 1
	const unsigned int DOF = Via_Pos[0].rows();
	
	if(DOF == 0)
		return false;

	for(int i=0; i < Size; i++)
	{
		if( !(Via_Pos[i].rows() == DOF && Via_Vel[i].rows() == DOF && Via_Acc[i].rows() == DOF) )
			return false;
	}

	// �ˬd TimeInterval �O�_�� Via_Time[i+1]-Via_Time[i] ������ƭ�
	for(int i=0; i < (Size-1); i++)
	{
		double N = (Via_Time[i+1]-Via_Time[i]) / TimeInterval;
		int num = int(N); 
		
		if(abs(N-double(num)-1) < TIME_RES_ERROR)
			num += 1; // ex: a.99999... ==> a+1

		if( !(num >= 1 && abs(Via_Time[i+1]-Via_Time[i]-double(num)*TimeInterval) < TIME_RES_ERROR) )
			return false;
	}

	return true;
}

#pragma endregion

#pragma region Traj_InOut ��@

Traj_InOut::Traj_InOut()
{
	IsError = false;
	f_ptr   = NULL;
	memset(ErrCode, '\0', sizeof(ErrCode));

	// �w���t�m�Ŷ�
	Seg_Num.reserve(20);
	
	// �ثe�u�|�Ψ� Position Ū��
	Data.Via_Pos.reserve(20);
	Data.Pos.reserve(2000);
}

Traj_InOut::~Traj_InOut()
{

}

bool Traj_InOut::Read_File(unsigned int DOF)
{
	// Traj�ɮ׮榡
	// �Y�X�{ '#' �N������ ==> ���L�᭱��椺�e
	// �y��O�� Seg_Num �P Joint Value �զ���
	// Seg_Num:     �P�e�q Via point ���t�h���_ [ �Ĥ@�� Via Point �� Seg_Num �ȥ�����0(�e���S��Via Point)�A��l�����j�󵥩�1(�ܤ֤@�q) ]
	// Joint Value: Robot joint space ���ȡA�ϥΪ̷|�M�w�nŪ��DOF�ӼƭȡA��������0�W�L�h����!!

	Eigen::VectorXd Via_Value(DOF);
	std::string Str;  // �Ȧs�r��
	char temp = '\0'; // �Ȧs�r��
	int Line  = 0;    // �����ثe�ѪR�ĴX��
	
	Str.reserve(64);

	// �}�l�ѪR
	while ( true )
	{
NEXT_LINE:
		Line++; // �p�ƥثe��ĴX��
		
		#pragma region ��� Seg_Num ���
		
		while (true)
		{
			temp = fgetc(f_ptr);
			
			// �J����ѡB�ťաBtab�Benter �n�i���r����(�r����פ���0)
			if(temp == '#' || temp == ' ' || temp == '\t' || temp == '\n' || temp == EOF)
			{
				if(Str.size() == 0)
				{
					if(temp == '#')
					{	
						Ignore_OneLine();  goto NEXT_LINE;
					}
					else if(temp == '\n')
						goto NEXT_LINE;
					else if(temp == EOF)
						return true; // ����Ū��
				}
				else // Str.size() != 0
				{
					if( !Get_SegNum(atoi(Str.c_str()), Line) )
						return false;
					Str.clear();
					break;
				}
			}
			else
				Str += temp;
		}
		
		// �p�G�O'#', Enter, EOF�N�������� joint value = 0
		if( temp == '#')
		{	
			Data.Via_Pos.push_back(Eigen::VectorXd::Zero(DOF));
			Ignore_OneLine();
			continue;
		}
		else if(temp == '\n')
		{
			Data.Via_Pos.push_back(Eigen::VectorXd::Zero(DOF));
			continue;
		}
		else if(temp == EOF)
		{	
			Data.Via_Pos.push_back(Eigen::VectorXd::Zero(DOF));
			return true; // ����Ū��
		}

		#pragma endregion

		#pragma region ���U�� joint value �ƭ�

		int Effective_Count = 0; // �����ثeŪ����h�֭Ӧ��Ī�joint value (0 <= Effective_Count <= DOF)
		Via_Value.setZero();	 // �����k0
		
		while (true)
		{
			temp = fgetc(f_ptr);

			// �J����ѡB�ťաBtab �n�i���r����
			if( temp == '#' || temp == ' ' || temp == '\t' || temp == '\n' || temp == EOF )
			{
				if(Str.size() != 0)
				{
					Via_Value(Effective_Count) = atof(Str.c_str());
					Effective_Count++;
					Str.clear();

					if(Effective_Count == DOF) // �w�g�񺡡A�᭱�i����
					{
						Data.Via_Pos.push_back(Via_Value);
						
						if(temp != '\n')
							Ignore_OneLine();
						break;
					}
				}

				if(temp == '#')
				{
					Data.Via_Pos.push_back(Via_Value);
					Ignore_OneLine();
					break;
				}
				else if(temp == '\n')
				{
					Data.Via_Pos.push_back(Via_Value);
					break;
				}
			}
			else
				Str += temp;
		}

		#pragma endregion
	}

	return true;
}

void Traj_InOut::Ignore_OneLine()
{
	char temp = '\0';
	
	while (true)
	{
		temp = fgetc(f_ptr);
		if( temp == '\n' || temp == EOF)
			break;
	}
}

bool Traj_InOut::Get_SegNum(int value, int Line)
{
	// Seg_Num: �P�e�q Via point ���t�h���_ [ �Ĥ@�� Via Point �� Seg_Num �ȥ�����0(�e���S��Via Point)�A��l�����j�󵥩�1(�ܤ֤@�q) ]
	
	if(Seg_Num.size() == 0) //�Ĥ@�ӭȶǶi��
	{	
		if(value == 0)
			Seg_Num.push_back(value);
		else
		{
			sprintf(ErrCode, "Error: Line %d, ��1�� SegValue ������0 !!", Line);
			return false;
		}
	}
	else
	{
		if(value >= 1)
			Seg_Num.push_back(value);
		else
		{
			sprintf(ErrCode, "Error: Line %d, ��%d�� SegValue �����j�󵥩�1 !!", Line, Seg_Num.size()+1);
			return false;
		}
	}

	return true;
}

bool Traj_InOut::Load_File(const char* FileName, unsigned int DOF)
{
	// ���~�ˬd
	if(DOF == 0)
	{
		sprintf(ErrCode, "Error: ���w DOF ���i��0 !!");
		IsError = true;
		return false;
	}

	// �}���ɮ�
	f_ptr = fopen(FileName, "r");
	
	if(f_ptr == NULL)
	{
		sprintf(ErrCode, "Error: %s �}�_���~!!", FileName);
		IsError = true;
		return false;
	}

	// ��l�ƨö}�l�ѪR�~���ɮ�
	Data.Via_Clear();
	Data.Clear();
	Seg_Num.clear();

	bool Result = Read_File(DOF);
	
	fclose(f_ptr); // �����ɮ�
	f_ptr = NULL;

	if( Result == false )
	{
		IsError = true;
		return false;
	}
	
	IsError = false;
	return true;
}

bool Traj_InOut::Linear_Planning()
{
	if(IsError)
		return false;
	
	if( Seg_Num.size() == 0 )
	{	
		IsError = true;
		sprintf(ErrCode, "Error: Traj_Data Empty");
		return false;
	}

	// �M�ť��e���
	Data.Pos.clear();

	// �w���t�m�Ŷ��ö}�l���t
	int sum = 0;
	for(int i=0, n=Seg_Num.size(); i < n; i++)
		sum += Seg_Num[i];

	Data.Pos.reserve(sum+1);

	Eigen::VectorXd dq;

	for(int i=0, n=Seg_Num.size(); i < n; i++)
	{
		if(i == 0)
			Data.Pos.push_back(Data.Via_Pos[i]);
		else
		{
			dq = (Data.Via_Pos[i] - Data.Via_Pos[i-1]) / double(Seg_Num[i]);
			for(int j=1, k=Seg_Num[i]; j <= k; j++)
				Data.Pos.push_back( Data.Via_Pos[i-1] + double(j)*dq );
		}
	}

	return true;
}

bool Traj_InOut::Linear_Planning(VectorXdList& q_Traj)
{
	if(IsError)
		return false;
	
	if( Seg_Num.size() == 0 )
	{	
		IsError = true;
		sprintf(ErrCode, "Error: Traj_Data Empty");
		return false;
	}

	// �M�ť��e���y��
	q_Traj.clear();

	// �w���t�m�Ŷ��ö}�l���t
	int sum = 0;
	for(int i=0, n=Seg_Num.size(); i < n; i++)
		sum += Seg_Num[i];

	q_Traj.reserve(sum+1);

	Eigen::VectorXd dq;

	for(int i=0, n=Seg_Num.size(); i < n; i++)
	{
		if(i == 0)
			q_Traj.push_back(Data.Via_Pos[i]);
		else
		{
			dq = (Data.Via_Pos[i] - Data.Via_Pos[i-1]) / double(Seg_Num[i]);
			for(int j=1, k=Seg_Num[i]; j <= k; j++)
				q_Traj.push_back( Data.Via_Pos[i-1] + double(j)*dq );
		}
	}

	return true;
}

const char* Traj_InOut::Get_LastError()
{
	if(IsError)
		return ErrCode;
	else
		return NULL;
}

#pragma endregion

#pragma region Trajectory Planning

bool Rbt::Solve_Quintic(double* Dest_Parameter, double P0, double P0_d, double P0_dd, double T0, double P1, double P1_d, double P1_dd,  double T1)
{
	// �Ѥ����h�����Y�� S(t) = [1; t; t^2; t^3; t^4; t^5].transpose() * [a0; a1; a2; a3; a4; a5]
	// ��X�GDest_Parameter[] Array Size: 6�A�Y�ƥѶ��Ƥp��j���������Ƨ� a0~a5
	// �ǤJ�G��lP0 [��m �t�� �[�t�� �ɶ�]�A����P1 [��m �t�� �[�t�� �ɶ�]
	// ���\�^��true�A�Ϥ�false

	// Ax = b ==> x = A.inv() * b;
	static Eigen::MatrixXd A(6, 6);
	static Eigen::VectorXd b(6);
	static double* A_data = A.data(); // Column Major
	static double* b_data = b.data(); // Column Major

	if(Dest_Parameter == NULL || abs(T1-T0) < TIME_RES_ERROR)
		return false;

	// b = [P0; P1; P0_d; P1_d; P0_dd; P1_dd]
	b_data[0] = P0;    b_data[1] = P1;    // ��m����
	b_data[2] = P0_d;  b_data[3] = P1_d;  // �t�ױ���
	b_data[4] = P0_dd; b_data[5] = P1_dd; // �[�t�ױ���

	// [1 t t^2 t^3 t^4 t^5]
	for(int i=0; i < 6; i++)
	{
		A_data[6*i]   = pow(T0, i); 
		A_data[6*i+1] = pow(T1, i);
	}

	// [0 1 2*t 3*t^2 4*t^3 5*t^4]
	A_data[2] = 0; A_data[8] = 1;
	A_data[3] = 0; A_data[9] = 1;
	for(int i=2; i < 6; i++)
	{
		A_data[6*i+2] = double(i) * pow(T0, i-1); 
		A_data[6*i+3] = double(i) * pow(T1, i-1);
	}

	// [0 0 2 6*t 12*t^2 20*t^3]
	A_data[4] = 0; A_data[10] = 0; A_data[16] = 2;
	A_data[5] = 0; A_data[11] = 0; A_data[17] = 2;
	for(int i=3; i < 6; i++)
	{
		A_data[6*i+4] = double(i) * double(i-1) * pow(T0, i-2); 
		A_data[6*i+5] = double(i) * double(i-1) * pow(T1, i-2);
	}


	// �ѫY�� x = A.inv() * b
	b = A.inverse() * b;
	memcpy(Dest_Parameter, b_data, 6*sizeof(double));

	return true;
}

bool Rbt::Quintic_Planning(Traj_Data& Dest_Traj)
{
	// �M�ť��e�����
	Dest_Traj.Clear();

	// �ˬd Via Point ��Ʈ榡
	if(Dest_Traj.Via_Check() == false)
		return false;

	const unsigned int Size = Dest_Traj.Via_Time.size();
	const unsigned int DOF  = Dest_Traj.Via_Pos[0].rows();

	Eigen::MatrixXd Para(6, DOF); // ��Y�ơAColumn Major
	Eigen::MatrixXd Time(6, 3);   // ��ɶ��AColumn Major
	Eigen::MatrixXd Temp(DOF, 3); // �񵲪G�AColumn Major

	// �w����J�`��
	Time(0, 0) = 1;
	Time(0, 1) = 0; Time(1, 1) = 1;
	Time(0, 2) = 0; Time(1, 2) = 0; Time(2, 2) = 2;

	// �w���t�m�O����Ŷ�
	int Total_Seg = 0;
	for(int i=0; i < (Size-1); i++)
		Total_Seg += int( (Dest_Traj.Via_Time[i+1]-Dest_Traj.Via_Time[i]) / Dest_Traj.TimeInterval );
	Dest_Traj.Reserved(Total_Seg + 2);

	// ��J��l��m�B�t�סB�[�t�סB�ɶ�
	Dest_Traj.Push_back(Dest_Traj.Via_Pos[0], Dest_Traj.Via_Vel[0], Dest_Traj.Via_Acc[0], Dest_Traj.Via_Time[0]);

	// �}�l�i�� Quintic_Planning �����p��
	for(int index = 0; index < (Size-1); index++)
	{
		// �p��Y�� [a0; a1; a2; a3; a4; a5]
		for(int i=0; i < DOF; i++)
		{
			Solve_Quintic(Para.col(i).data(), Dest_Traj.Via_Pos[index](i),   Dest_Traj.Via_Vel[index](i),   Dest_Traj.Via_Acc[index](i),   Dest_Traj.Via_Time[index],
				                              Dest_Traj.Via_Pos[index+1](i), Dest_Traj.Via_Vel[index+1](i), Dest_Traj.Via_Acc[index+1](i), Dest_Traj.Via_Time[index+1]);
		}

		// �����i����m Para = Para.transpose();
		Para.transposeInPlace();
		
		double seg_temp = (Dest_Traj.Via_Time[index+1]-Dest_Traj.Via_Time[index]) / Dest_Traj.TimeInterval;
		int seg_num = int(seg_temp);
		
		if(abs(seg_temp-double(seg_num)-1) < TIME_RES_ERROR)
			seg_num += 1; // ex: a.99999... ==> a+1

		for(int seg=1; seg <= seg_num; seg++)
		{
			double t = Dest_Traj.Via_Time[index] + double(seg) * Dest_Traj.TimeInterval;
			
			// [1; t; t^2; t^3; t^4; t^5]
			for(int i=1; i < 6; i++)
				Time(i, 0) = pow(t, i);

			// [0; 1; 2*t; 3*t^2; 4*t^3; 5*t^4]
			for(int i=2; i < 6; i++)
				Time(i, 1) = double(i) * pow(t, i-1);
			
			// [0; 0; 2; 6*t; 12*t^2; 20*t^3]
			for(int i=3; i < 6; i++)
				Time(i, 2) = double(i) * double(i-1) * pow(t, i-2);

			// [DOF,3] = [DOF,6] * [6,3]
			Temp = Para * Time;
			
			// ��J��m�B�t�סB�[�t�סB�ɶ�
			Dest_Traj.Push_back(Temp.col(0), Temp.col(1), Temp.col(2), t);
		}

		// ��m�^��
		Para.transposeInPlace();
	}

	return true;
}

#pragma endregion