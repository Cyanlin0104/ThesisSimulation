#pragma once

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <vector>
#include <map>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/StdVector>

// rad �P deg ����
#define Deg2Rad(theta) ((theta)*M_PI/180.0) 
#define Rad2Deg(theta) ((theta)*180.0/M_PI)

// ���O�[�t�ױ`�� (m/s^2)
#define Gravity 9.80665

// Frame Mode
#define ACTIVE_FRAME  0
#define PASSIVE_FRAME 1
#define CONST_FRAME   2


// Eigen Vector �X�R for Robot library
namespace Eigen
{
	// Column Major
	typedef Matrix<   int, 6, 1>    Vector6i;
	typedef Matrix< float, 6, 1>    Vector6f;
	typedef Matrix<double, 6, 1>    Vector6d;
}

namespace Rbt
{
	typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Matrix4dList;
	typedef std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> MatrixXdList;
	
	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector3dList;
	typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> Vector6dList;
	typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> VectorXdList;
}

// Robot Kinematics Class
namespace Rbt
{
	class DHFrame;
	class EndEffector;
	class Robot;
	
	class DHFrame
	{
	protected:
		double a, alpha, d, theta, cmd;  // DH ��l�Ѽ�[a alpha d theta]�A cmd �� Robot_q
		double ratio;                    // �Q��Frame�P����Frame��������ʤ�
		double joint_min, joint_max;
		
		Eigen::Matrix4d TFMat;      // �ΨӦs��۹��Base Frmae��Transformation Matrix
		
		Eigen::Matrix4d ConstFrame; // �ΨӦs�� Const Frame
		Eigen::Matrix4d Front_T;    // �ΨӦs�� pseudo frame �� �D��frame ������ const T
		Eigen::Matrix4d back_T;     // �ΨӦs�� pseudo frame �� �D��frame ������ const T

		DHFrame *Parent_DHNode;              // ���V���`�I
		std::vector<DHFrame*> Child_DHNode;  // ���V�Ҧ��l�`�I
		DHFrame *DependActive_DHNode;        // �Y�O�Q��Frame�A�h���V�̪ۨ��D�ʸ`�I�F�Ϥ��Y�O�D��Frame�h���V�ۤv
		
		int DependActive_ID;   // �Y�O�Q��Frame�A�h�����̪ۨ��D�ʸ`�IID�F�Ϥ��Y�O�D��Frame�h��Self_ID
		
		int qList_Index;   // �����D��Frame�b qList ������m

		void (DHFrame::*UpdateMat_ptr)(const Eigen::Matrix4d&);
		inline void UpdateMat(double New_theta);          // ��s TFMat (��e�@�b)
		void UpdateMat1(const Eigen::Matrix4d& Parent_T); // TFMat = Parent_T * TFMat
		void UpdateMat2(const Eigen::Matrix4d& Parent_T); // TFMat = Parent_T * Front_T * TFMat
		void UpdateMat3(const Eigen::Matrix4d& Parent_T); // TFMat = Parent_T * TFMat * back_T
		void UpdateMat4(const Eigen::Matrix4d& Parent_T); // TFMat = Parent_T * Front_T * TFMat * back_T
		void UpdateMat5(const Eigen::Matrix4d& Parent_T); // TFMat = Parent_T * Const Frame

		bool HaveJointLimit; // �O�_���]�w joint_limit�A�w�]��false
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// �غc�l
		// �إ� TransFormation Matrix �ѼƳ]�w�A���G[a, alpha, d, theta] = (m, rad, m, rad)
		DHFrame();

		DHFrame(int _Parent_ID, int _Self_ID, double _a, double _alpha, double _d, double _theta);
		DHFrame(int _Parent_ID, int _Self_ID, double _a, double _alpha, double _d, double _theta, Eigen::Matrix4d *_Front_T, Eigen::Matrix4d *_back_T); // �Y�S�� Front_T �� back_T �h�ǤJNULL

		DHFrame(int _Parent_ID, int _Self_ID, int _DependActive_ID, double _a, double _alpha, double _d, double _theta, double _ratio);
		DHFrame(int _Parent_ID, int _Self_ID, int _DependActive_ID, double _a, double _alpha, double _d, double _theta, double _ratio, Eigen::Matrix4d *_Front_T, Eigen::Matrix4d *_back_T);

		DHFrame(int _Parent_ID, int _Self_ID, const Eigen::Matrix4d& _ConstFrame);

		// �Ѻc�l
		~DHFrame();

		// �]�w�D��Frame�� Joint ����d��A���rad
		void Set_JointLimit(double _joint_min, double _joint_max);

		void Set_Parent_DHNode(DHFrame *_Parent_DHNode);
		DHFrame* Get_Parent_DHNode() const;

		void Set_DependActive_DHNode(DHFrame *_DependActive_DHNode);
		DHFrame* Get_DependActive_DHNode() const;

		void Add_Child_DHNode(DHFrame *_Child_DHNode);
		void Clear_Child_DHNode();
		const std::vector<DHFrame*>& Get_Child_DHNode() const;

		const Eigen::Matrix4d& Get_TFMat() const; // return TFMat

		// ���� Frame Mode
		const int FrameMode;         // Frame_Mode: 0:Active Frame, 1:Passive Frame, 2:Const Frame
		const int Parent_ID;
		const int Self_ID;
		int Get_DependActive_ID() const;


		// �p�G�OActive Frame�٭n�A�]�w Robot_q ����m
		void Set_qList_Index(int index);
		int Get_qList_Index() const;

		// ��o Kinematic �����Ѽ�
		double Get_DH_a() const;
		double Get_DH_alpha() const;
		double Get_DH_d() const;
		double Get_DH_Offset_theta() const;
		double Get_Ratio() const;

		// �L�XFrame��������T
		virtual void Get_FrameInfo();

		// �]�w���ਤ�שR�O
		virtual void Set_Command(double New_Cmd);  // ��s�D��Frame��cmd
		virtual void Set_Command();                // ��s�Q��Frame��cmd
		virtual double Get_Command() const;

		// �ˬd joint limit�A�Y�W�X�|��� Cmd
		void Check_JointLimit(double& New_Cmd);

		// �H���j���覡�h����FK��s�A�ðO���bFrame_List
		void Get_FK(const Eigen::Matrix4d& Parent_T);


		// ���Ĳv���Ҷq�A EndEffector �i�H�����s�� DHFrame �p�������A�٥h�禡�I�s���t��
		friend class EndEffector;
	};
	
	class EndEffector
	{
	private:
		DHFrame* EndFrame;
		std::vector<DHFrame*> Path; // �O���q�ڸ`�I�쥽�ݸ`�I�����|
		Eigen::VectorXd EEP;        // �x�sEndEffector��Position�MOrientation

		void Search_Path();

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// �Ѻc�l
		EndEffector();
		EndEffector(DHFrame* _EndFrame);
		EndEffector(DHFrame* _EndFrame, const std::vector<DHFrame*>& _Path);

		// �Ѻc�l
		~EndEffector();

		void Set_EndFrame(DHFrame* _EndFrame);
		DHFrame* Get_EndFrame() const;

		void Print_Path();
		const std::vector<DHFrame*>& Get_Path();
		const Eigen::VectorXd& Get_EEP();           // �]�tPosition�MOrientation
		const Eigen::Matrix4d& Get_EETFMat() const; // Transformation Matrix
	};

	class Robot
	{
	protected:
		unsigned int Active_DOF;  // ����Robot�D��Frame���Ӽ�
		unsigned int Passive_DOF; // ����Robot�Q��Frame���Ӽ�
		unsigned int EE_SIZE;     // �������h�֭�EndEffector

		// �Ҧ��D��Frame������q�A����FrameID���j�p�A�Ѥp��j�ƧǡA��l�Ҭ�0
		Eigen::VectorXd Robot_q;

		// �x�s Frame* �P EndEffector*
		std::vector<DHFrame*>     ActiveFrame_Store;
		std::vector<DHFrame*>     PassiveFrame_Store;
		std::vector<DHFrame*>     ConstFrame_Store;
		std::vector<EndEffector*> EndEffector_Store;

		std::map<int, DHFrame*> Frame_Tree;   // �̷�Self_ID�Ѥp��j�̧ǰO���Ҧ��� DHFrame Node
		
		DHFrame*        Root_DHFrame; // DHFrame ���ڸ`�I
		Eigen::Matrix4d Base_TFMat;   // �H�@�ɮy�Ъ��� Robot Base �� Transformation Matrix�A�w�]�� Identity

		// �H���j���覡�h����EE��s�A�ðO���bEndEffector_Store
		void Search_EE(DHFrame* Self_DHNode, std::vector<DHFrame*>& _Path);
		virtual void Frame_Traversal(DHFrame* Self_DHNode);
		
		// �������IK�L�{����dp
		inline void ClampMag(Eigen::VectorXd& dp, bool Is_IncludeOri);
		
		// �x�s�]�w�� EE size �ҹ����� Damp_Value�A�����u�ʤ��t�Υ~�� ==> lambda = Damp^2
		double EE_Value[2], Damp_Value[2]; //�w�]�Ҭ�0

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// �غc�l
		Robot();

		// �Ѻc�l
		~Robot();

		// �[�J�D��Frame�AFrameID���i����
		virtual void AddActiveFrame(DHFrame* ActiveFrame);
		virtual void AddActiveFrame(DHFrame* ActiveFrame, double joint_min, double joint_max);

		// �[�J�Q��Frame�AFrameID���i����
		virtual void AddPassiveFrame(DHFrame* PassiveFrame);
		
		// �[�JConst Frame�AFrameID���i����
		virtual void AddConstFrame(DHFrame* ConstFrame);

		// �N�Ҧ��� DH Frame �إ߳s���A�̷�ID�Ѥp��j�j�M�A�åB�M��EndEffector
		virtual bool Build_Robot();

		// �L�X�ثe�Ҧ�Frame����T
		virtual void Get_RobotInfo();
		
		// �]�wRobot Base TFMat �ç�s FK
		bool Set_BaseTFMat(const Eigen::Matrix4d& Src_BaseTFMat); // ���\�^��True�A�Ϥ�False
		void Get_BaseTFMat(Eigen::Matrix4d& Dest_BaseTFMat)  const;
		const Eigen::Matrix4d& Get_BaseTFMat() const;

		unsigned int DOF_Size() const;
		unsigned int EE_Size() const;

		// �]�w Robot_q �ç�s Transformation Matrix List
		void Set_q(Eigen::VectorXd& Src_q);
		
		// Robot_q �����k0�A�ç�s Transformation Matrix List
		void Reset_q();

		void Get_q(Eigen::VectorXd& Dest_q) const;
		const Eigen::VectorXd& Get_q() const;

		// ��o�x�s�� <DHFrame*>  �P <EndEffector*>
		const std::vector<DHFrame*>&     Get_ActiveFrame() const;
		const std::vector<DHFrame*>&     Get_PassiveFrame() const;
		const std::vector<DHFrame*>&     Get_ConstFrame() const;
		const std::vector<EndEffector*>& Get_EndEffector() const;
		DHFrame* Get_RootDHFrame() const;

		// Pd = J * qd ==> Jacobian �u��Position�A�w��Ҧ���EndEffector
		void Get_JacobianNoOri(Eigen::MatrixXd& J) const;

		// Pd = J * qd ==> Jacobian �]�tPosition�POrientation�A�w��Ҧ���EndEffector
		void Get_Jacobian(Eigen::MatrixXd& J) const;
		
		// Pd = J * qd ==> Jacobian �u��Position�A�w����w��EndEffector
		void Get_JacobianNoOri(Eigen::MatrixXd& J, const std::vector<EndEffector*>& UserDefEE) const;

		// Pd = J * qd ==> Jacobian �]�tPosition�POrientation�A�w����w��EndEffector
		void Get_Jacobian(Eigen::MatrixXd& J, const std::vector<EndEffector*>& UserDefEE) const;


		// �]�w EE_Value[2] = {EE_min, EE_max}, Damp_Value[2] = {Damp_min, Damp_max}
		void Set_DLS_Parameter(double EE_min, double Damp_min, double EE_max, double Damp_max);

		// �q EE_Value[2], Damp_Value[2] ��o���t�� damp
		double Get_DLS_Damp(double EE_Size);

		#pragma region Damped Least Square (DLS) IK
		// �Y�^��true�N�����N���ƥH���p��~�t�]�w�Afalse�h�N���w��F���N����

		// �̧ǥu�����Ҧ�EndEffector���ؼ�[Position] ==> �h�p������� Robot_q �ç�sFK
		bool dlsIK(const Vector3dList& TargetPos);

		// �̧ǵ����Ҧ�EndEffector���ؼ�[Position; Orientation] ==> �h�p������� Robot_q �ç�sFK
		bool dlsIK(const VectorXdList& TargetPosOri);
		bool dlsIK(const Matrix4dList& TargetPosOri);


		// �̧ǵ����Ҧ�EndEffector�C�Ӯɨ誺�ؼ�[Position] ==> �x�s�X�C�Ӯɨ������ Robot_q ����sFK
		void dlsIK(const std::vector<Vector3dList>& EE_Trajectory, VectorXdList& q_Trajectory);

		// �̧ǵ����Ҧ�EndEffector�C�Ӯɨ誺�ؼ�[Position; Orientation] ==> �x�s�X�C�Ӯɨ������ Robot_q ����sFK
		void dlsIK(const std::vector<VectorXdList>& EE_Trajectory, VectorXdList& q_Trajectory);
		void dlsIK(const std::vector<Matrix4dList>& EE_Trajectory, VectorXdList& q_Trajectory);

		// �q Frame_Tree ���M����wSelf_ID������ DHFrame*�A�Y�S���h�^��NULL
		DHFrame* Find_UserDef_Frame(int Self_ID);

		// �̧ǵ������wEndEffector���ؼ�[Position] ==> �h�p������� Robot_q �ç�sFK
		bool dlsIK(const Vector3dList& TargetPos, const std::vector<EndEffector*>& UserDefEE);

		// �̧ǵ������wEndEffector���ؼ�[Position; Orientation] ==> �h�p������� Robot_q �ç�sFK
		bool dlsIK(const VectorXdList& TargetPosOri, const std::vector<EndEffector*>& UserDefEE);
		bool dlsIK(const Matrix4dList& TargetPosOri, const std::vector<EndEffector*>& UserDefEE);

		// �̧ǵ������wEndEffector�C�Ӯɨ誺�ؼ�[Position] ==> �x�s�X�C�Ӯɨ������ Robot_q ����sFK
		void dlsIK(const std::vector<Vector3dList>& EE_Trajectory, VectorXdList& q_Trajectory, const std::vector<EndEffector*>& UserDefEE);

		// �̧ǵ������wEndEffector�C�Ӯɨ誺�ؼ�[Position; Orientation] ==> �x�s�X�C�Ӯɨ������ Robot_q ����sFK
		void dlsIK(const std::vector<VectorXdList>& EE_Trajectory, VectorXdList& q_Trajectory, const std::vector<EndEffector*>& UserDefEE);
		void dlsIK(const std::vector<Matrix4dList>& EE_Trajectory, VectorXdList& q_Trajectory, const std::vector<EndEffector*>& UserDefEE);

		#pragma endregion
	};
}

// Robot Dynamics Class
namespace Rbt
{
	class DynDHFrame;
	class DynRobot;

	class DynDHFrame : public DHFrame
	{
	protected:
		// Robot cmd : q, qd, qdd
		double cmd_d;  // cmd 1���L�� ==> Robot_qd
		double cmd_dd; // cmd 2���L�� ==> Robot_qdd
		
		// Dynamic �Ѽ� (���G�Ҧ��V�q�ҥH�ثe�� Local Frame �Ӫ���)
		double Mass;              // Link ��߽�q ���Gkg
		Eigen::Vector3d MC_P;     // Link ��ߦ�m ���Gm
		Eigen::Matrix3d Inertial; // �b Link ��ߤW����ʺD�q�A�D�ʶb���� Local Frame �y�� ���Gkg-m^2

		// Dynamic �V�q�ҥH�ثe�� Local Frame ����
		Eigen::Vector3d W;      // ���t��          ���G(rad/s)
		Eigen::Vector3d Wd;     // ���[�t��        ���G(rad/s^2)
		Eigen::Vector3d Ac;     // ��ߥ[�t��      ���G(m/s^2)
		Eigen::Vector3d Ae;     // Link ���ݥ[�t�� ���G(m/s^2)
		Eigen::Vector3d Force;  // �O              ���G(N)
		Eigen::Vector3d Torque; // �O�x            ���G(Nm)
		double tau;             // Torque ��v�b����b�W����(��v�b�e�@�b��z) ���G(Nm)
		
		// Frame[i] ����~���O�P�~���O�x(�HLocal Frame����)�A�w�]�Ҭ��s
		Eigen::Vector3d External_Force;  // �O      ���G(N)
		Eigen::Vector3d External_Torque; // �O�x    ���G(Nm)

		// ���]����Frame�� Frame[i]�A�h Frame[i-1] ��Parent Frame
		Eigen::Matrix3d R_Parent;        // R[i, i-1] = R[0, i].transpose * R[0, i-1]
		Eigen::Vector3d r_Parent_MC;     // r[i-1, MCi] ==> Frame[i-1]���I �� ���[i] �V�q
		Eigen::Vector3d r_Parent_Origin; // r[i-1, i] ==> Frame[i-1]���I �� Frame[i]���I �V�q

		void (DynDHFrame::*Forward_Recur_ptr)(const Eigen::Matrix4d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&);
		void Forward_Recur1(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae);  // Active / Passive Frame �ϥ�
		void Forward_Recur2(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae);  // Const Frame �ϥ�

		void (DynDHFrame::*Backward_Recur_ptr)(void);
		void Backward_Recur1(); // Active / Passive Frame �ϥ�
		void Backward_Recur2(); // Const Frame �ϥ�

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// �غc�l
		// �ǤJ DHFrame ����A�H�� Drnamic �Ѽ� [Mass, MC_P[3], Inertial[6]] ���G[kg, m, kg-m^2]
		DynDHFrame();
		DynDHFrame(const DHFrame& _DHFrame, const double _Mass = 0, const double *_MC_P = NULL, const double* _Inertial = NULL);

		// �Ѻc�l
		~DynDHFrame();

		// ���O�[�t�� (�H Base Frame ���ܡA�w�]��0) 
		static Eigen::Vector3d g;

		// �L�XFrame��������T
		void Get_FrameInfo();

		// �]�w Dynamic �����Ѽ�
		void Set_Mass(double New_Mass);
		void Set_MC_Position(const double* New_MC_Position); // MC_P = {MCx, MCy, MCz}
		void Set_MC_Position(const Eigen::Vector3d& New_MC_Position);
		void Set_Inertial(const double* New_Inertial); // Inertial = {Ixx, Ixy, Ixz, Iyy, Iyz, Izz}
		void Set_Inertial(const Eigen::Matrix3d& New_Inertial);

		double Get_Mass() const; 
		const Eigen::Vector3d& Get_MC_Position() const;
		const Eigen::Matrix3d& Get_Inertial() const;

		const Eigen::Vector3d& Get_W()     const;  // ���t��          ���G(rad/s)
		const Eigen::Vector3d& Get_Wd()    const;  // ���[�t��        ���G(rad/s^2)
		const Eigen::Vector3d& Get_Ac()    const;  // ��ߥ[�t��      ���G(m/s^2)
		const Eigen::Vector3d& Get_Ae()    const;  // Link ���ݥ[�t�� ���G(m/s^2)
		const Eigen::Vector3d& Get_Force() const;  // �O              ���G(N)
		const Eigen::Vector3d& Get_Torque()const;  // �O�x            ���G(Nm)
		double Get_tau() const;                    // ���F��X�O�x    ���G(Nm)

		// �]�w�ثeFrame���쪺�~���O�ΤO�x�A(���GLocal Frame ����)
		void Set_External_Value(const Eigen::Vector3d& Ex_Force, const Eigen::Vector3d& Ex_Torque);
		void Set_External_Zero();
		void Set_External_Force(const Eigen::Vector3d& Ex_Force);
		void Set_External_Torque(const Eigen::Vector3d& Ex_Torque);

		void Get_External_Value(Eigen::Vector3d& Dest_Force, Eigen::Vector3d& Dest_Torque) const;
		void Get_External_Force(Eigen::Vector3d& Dest_Force) const;
		const Eigen::Vector3d& Get_External_Force() const;
		void Get_External_Torque(Eigen::Vector3d& Dest_Torque) const;
		const Eigen::Vector3d& Get_External_Torque() const;

		// �]�w���ਤ�שR�O
		void Set_Command(double New_q, double New_qd, double New_qdd);  // ��s�D��Frame�� q, qd, qdd
		void Set_q(double New_q);
		void Set_qd(double New_qd);
		void Set_qdd(double New_qdd);
		
		void Set_Command();                                             // ��s�Q��Frame�� q, qd, qdd
		void Set_q();
		void Set_qd();
		void Set_qdd();

		void Get_Command(double& Dest_q, double& Dest_qd, double& Dest_qdd) const;
		double Get_q() const;
		double Get_qd() const;
		double Get_qdd() const;

		// Dynamic Equation : M(q)*qdd + C(q, qd)*qd + G(q) + Tau_ex = Tau(q, qd, qdd)
		// Inver Dynamic Input (q, qd, qdd) ==> Output Tau
		// Forward Dynamic Input (Tau, Ini_q, Ini_qd) ==> Output qdd
		
		// �H���j���覡�h�P�ɧ��� FK ��s�M Dynamic Torgue �p��A�ðO���b�C�� Frame ��
		// �ǤJ�e�@��Frame �� TFMat, W, Wd, Ae
		void Get_FK_InvDyn(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae);

		// �H���j���覡�h���� Dynamic Torgue �p��A�ðO���b�C�� Frame ��(���G����sFK)
		void Get_InvDyn(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae);
	};

	class DynRobot : public Robot
	{
	protected:
		// �Ҧ��D��Frame������qd, qdd�A����FrameID���j�p�A�Ѥp��j�ƧǡA��l�Ҭ�0
		Eigen::VectorXd Robot_qd;
		Eigen::VectorXd Robot_qdd;

		void Frame_Traversal(DynDHFrame* Self_DHNode);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		// �غc�l
		DynRobot();
		
		// �Ѻc�l
		~DynRobot();

		// �[�J�D��Frame�AFrameID���i����
		void AddActiveFrame(DynDHFrame* ActiveFrame);
		void AddActiveFrame(DynDHFrame* ActiveFrame, double joint_min, double joint_max);

		// �[�J�Q��Frame�AFrameID���i����
		void AddPassiveFrame(DynDHFrame* PassiveFrame);
		
		// �[�JConst Frame�AFrameID���i����
		void AddConstFrame(DynDHFrame* ConstFrame);

		// �N�Ҧ��� DH Frame �إ߳s���A�̷�ID�Ѥp��j�j�M�A�åB�M��EndEffector
		bool Build_DynRobot();

		// �L�X�ثe�Ҧ�Frame����T
		void Get_RobotInfo();

		// �]�w Robot_qd, Robot_qdd �ç�s DynDHFrame List cmd_d, cmd_dd
		void Set_qd(Eigen::VectorXd& Src_qd);
		void Set_qdd(Eigen::VectorXd& Src_qdd);
		
		// Robot_qd, Robot_qdd �����k0�A�ç�s DynDHFrame List cmd_d, cmd_dd
		void Reset_qd();
		void Reset_qdd();

		void Get_qd(Eigen::VectorXd& Dest_qd) const;
		const Eigen::VectorXd& Get_qd() const;

		void Get_qdd(Eigen::VectorXd& Dest_qdd) const;
		const Eigen::VectorXd& Get_qdd() const;

		// �p��Inverse Dynamic�A��J(Robot_q, Robot_dq, Robot_qdd) �p�� Torque
		bool InvDyn(Eigen::VectorXd& Dest_Torque, const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd);
		bool InvDyn(VectorXdList& Dest_Torque, const VectorXdList& q, const VectorXdList& qd, const VectorXdList& qdd);

		// �p�� Forward Dynamic�A��J Torque, ��lq, qd �p�� Robot_qdd
		bool ForDyn(const Eigen::VectorXd& Torque, const Eigen::VectorXd& Ini_q, const Eigen::VectorXd& Ini_qd);

		// �p�� Forward Dynamic�A��J Torque(t), ��lq, qd, �ɶ����jT �p�� Robot_q(t), Robot_qd(t), Robot_qdd(t)
		// q_traj[n] = q(nT), qd_traj[n] = qd(nT), qdd_traj[n] = qdd(nT),
		bool ForDyn(VectorXdList& Dest_q, VectorXdList& Dest_qd, VectorXdList& Dest_qdd, const VectorXdList& Torque, const Eigen::VectorXd& Ini_q, const Eigen::VectorXd& Ini_qd, const double TimeInterval);
	};
}


#pragma region Robot �����ƾǹB��禡

namespace Rbt
{
	// �qStandard DH Model �ন TransFormation Matrix�A���(m,rad,m,rad)
	Eigen::Matrix4d& DH2TFMat(double a, double alpha, double d, double theta);
	void DH2TFMat(Eigen::Matrix4d& Dest_T, double a, double alpha, double d, double theta);


	// �qRotation �ন���� Orientation = theta*k
	Eigen::Vector3d& Get_Orientation(const Eigen::Matrix3d& Src_R);
	void Get_Orientation(Eigen::Vector3d& Dest_Ori, const Eigen::Matrix3d& Src_R);
	void Get_Orientation(double* Dest_Ori, const double* Src_R); // Column Major�A�����T�O Array size ���T

	// �qTransformation Matrix ��o P = [Position, Orientation]
	void Get_PosOri(Eigen::VectorXd& Dest_P, const Eigen::Matrix4d& Src_T);
	void Get_PosOri(double* Dest_P, const double* Src_T); // Column Major�A�����T�O Array size ���T

	// �q P = [Position, Orientation] �ഫ�� Transformation Matrix
	void Get_TFMatrix(Eigen::Matrix4d& Dest_T, const Eigen::VectorXd& Src_P);
	void Get_TFMatrix(double* Dest_T, const double* Src_P); // Column Major�A�����T�O Array size ���T

	// �q O = [Orientation] �ഫ�� Rotation Matrix
	void Get_RMatrix(Eigen::Matrix3d& Dest_R, const Eigen::Vector3d& Src_O);
	void Get_RMatrix(double* Dest_R, const double* Src_O); // Column Major�A�����T�O Array size ���T

	// �p�� Differential Transformation Matrix ==> dp = [Dleta_Pos, Dleta_Ori]
	void Get_DiffTFMat(double* Dest_dp, const double* T_target, const double* T_ref);// Column Major�A�����T�O Array size ���T


	// ��o�u��X, Y, Z�b���઺Rotation Matrix ==> Rot(X, theta), Rot(Y, theta), Rot(Z, theta)
	void Get_RotX(double* Dest_R, double Theta); // Column Major�A�����T�O Array size ���T
	void Get_RotY(double* Dest_R, double Theta); // Column Major�A�����T�O Array size ���T
	void Get_RotZ(double* Dest_R, double Theta); // Column Major�A�����T�O Array size ���T
}

#pragma endregion