#include "stdafx.h"
#include "Robot.h"

#define pi M_PI

using namespace Rbt;

#pragma region DHFrame ��@

DHFrame::DHFrame()
	   :Parent_ID(-1), Self_ID(-1), FrameMode(-1)
{
	printf("DHFrame Error�G��l�S�����w�Ѽ�\n");
	assert(false);
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, double _a, double _alpha, double _d, double _theta)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), FrameMode(ACTIVE_FRAME)
{
	// ���~�ˬd
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error�GParent_ID �P Self_ID �ۦP\n");
		printf("Parent_ID=%d, Self_ID=%d, ACTIVE_FRAME\n", Parent_ID, Self_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f\n", _a, _alpha, _d, _theta);
		assert(false);
	}

	// �Ѽƪ�l��
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = 1;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	Front_T = Eigen::Matrix4d::Zero();
	back_T = Eigen::Matrix4d::Zero();

	Parent_DHNode = NULL;
	DependActive_DHNode = this;
	DependActive_ID = Self_ID;

	qList_Index = -1;

	UpdateMat_ptr = &DHFrame::UpdateMat1;
	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, double _a, double _alpha, double _d, double _theta, Eigen::Matrix4d *_Front_T, Eigen::Matrix4d *_back_T)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), FrameMode(ACTIVE_FRAME)
{
	// ���~�ˬd
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error�GParent_ID �P Self_ID �ۦP\n");
		printf("Parent_ID=%d, Self_ID=%d, ACTIVE_FRAME\n", Parent_ID, Self_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f\n", _a, _alpha, _d, _theta);
		if(_Front_T != NULL)
			std::cout << "Front_T = \n" << *_Front_T << std::endl;
		if(_back_T != NULL)
			std::cout << "back_T = \n" << *_back_T << std::endl;

		assert(false);
	}

	// �Ѽƪ�l��
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = 1;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	
	if(_Front_T != NULL)
		Front_T = (*_Front_T);
	else
		Front_T = Eigen::Matrix4d::Zero();
	
	if(_back_T != NULL)
		back_T = (*_back_T);
	else
		back_T = Eigen::Matrix4d::Zero();

	
	Parent_DHNode = NULL;
	DependActive_DHNode = this;
	DependActive_ID = Self_ID;

	qList_Index = -1;

	if(_Front_T == NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat1;
	else if(_Front_T != NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat2;
	else if(_Front_T == NULL && _back_T != NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat3;
	else  // _Front_T != NULL && _back_T != NULL
		UpdateMat_ptr = &DHFrame::UpdateMat4;

	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, int _DependActive_ID, double _a, double _alpha, double _d, double _theta, double _ratio)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), DependActive_ID(_DependActive_ID), FrameMode(PASSIVE_FRAME)
{
	// ���~�ˬd
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error�GParent_ID �P Self_ID �ۦP\n");
		printf("Parent_ID=%d, Self_ID=%d, DependActive_ID=%d, PASSIVE_FRAME\n", Parent_ID, Self_ID, DependActive_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, ratio=%f\n", _a, _alpha, _d, _theta, _ratio);
		assert(false);
	}

	// �Ѽƪ�l��
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = _ratio;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	Front_T = Eigen::Matrix4d::Zero();
	back_T = Eigen::Matrix4d::Zero();

	Parent_DHNode = NULL;
	DependActive_DHNode = NULL;
	
	qList_Index = -1;

	UpdateMat_ptr = &DHFrame::UpdateMat1;
	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, int _DependActive_ID, double _a, double _alpha, double _d, double _theta, double _ratio, Eigen::Matrix4d *_Front_T, Eigen::Matrix4d *_back_T)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), DependActive_ID(_DependActive_ID), FrameMode(PASSIVE_FRAME)
{
	// ���~�ˬd
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error�GParent_ID �P Self_ID �ۦP\n");
		printf("Parent_ID=%d, Self_ID=%d, DependActive_ID=%d, PASSIVE_FRAME\n", Parent_ID, Self_ID, DependActive_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, ratio=%f\n", _a, _alpha, _d, _theta, _ratio);
		if(_Front_T != NULL)
			std::cout << "Front_T = \n" << *_Front_T << std::endl;
		if(_back_T != NULL)
			std::cout << "back_T = \n" << *_back_T << std::endl;

		assert(false);
	}

	// �Ѽƪ�l��
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = _ratio;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	
	if(_Front_T != NULL)
		Front_T = (*_Front_T);
	else
		Front_T = Eigen::Matrix4d::Zero();
	
	if(_back_T != NULL)
		back_T = (*_back_T);
	else
		back_T = Eigen::Matrix4d::Zero();

	
	Parent_DHNode = NULL;
	DependActive_DHNode = NULL;

	qList_Index = -1;

	if(_Front_T == NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat1;
	else if(_Front_T != NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat2;
	else if(_Front_T == NULL && _back_T != NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat3;
	else  // _Front_T != NULL && _back_T != NULL
		UpdateMat_ptr = &DHFrame::UpdateMat4;

	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, const Eigen::Matrix4d& _ConstFrame)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), FrameMode(CONST_FRAME)
{
	// ���~�ˬd
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error�GParent_ID �P Self_ID �ۦP\n");
		printf("Parent_ID=%d, Self_ID=%d, CONST_FRAME\n", Parent_ID, Self_ID);
		std::cout << "ConstFrame = \n" << _ConstFrame << std::endl;
		assert(false);
	}

	// �Ѽƪ�l��
	a = 0;		alpha = 0;		d = 0;		theta = 0;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = 0;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = _ConstFrame;
	Front_T = Eigen::Matrix4d::Zero();
	back_T = Eigen::Matrix4d::Zero();

	Parent_DHNode = NULL;
	DependActive_DHNode = this;
	DependActive_ID = Self_ID;

	qList_Index = -1;

	UpdateMat_ptr = &DHFrame::UpdateMat5;
	HaveJointLimit = false;
}

DHFrame::~DHFrame()
{
	UpdateMat_ptr = NULL;
}

void DHFrame::Set_JointLimit(double _joint_min, double _joint_max)
{
	if(FrameMode == ACTIVE_FRAME)
	{
		joint_min = _joint_min;
		joint_max = _joint_max;
		HaveJointLimit = true;
	}
}

void DHFrame::Set_Parent_DHNode(DHFrame *_Parent_DHNode)
{
	Parent_DHNode = _Parent_DHNode;
}

DHFrame* DHFrame::Get_Parent_DHNode() const
{
	return Parent_DHNode;
}

void DHFrame::Set_DependActive_DHNode(DHFrame *_DependActive_DHNode)
{
	if(FrameMode == PASSIVE_FRAME)
	{
		if(_DependActive_DHNode->FrameMode == ACTIVE_FRAME)
			DependActive_DHNode = _DependActive_DHNode;
		else
		{
			printf("DHFrame Error�GSelf_ID=%d �̥ۨD��Frame���w���~\n", Self_ID);
			assert(false);
		}
	}
}

DHFrame* DHFrame::Get_DependActive_DHNode() const
{
	return DependActive_DHNode;
}

// �ھ�ChildID�Ѥp�ƨ�j
bool Sort_SelfID(const DHFrame* A, const DHFrame* B)
{
	return (A->Self_ID < B->Self_ID); 
}

void DHFrame::Add_Child_DHNode(DHFrame *_Child_DHNode)
{
	//���G Self_ID ���୫��
	Child_DHNode.push_back(_Child_DHNode);
}

void DHFrame::Clear_Child_DHNode()
{
	Child_DHNode.clear();
}

const std::vector<DHFrame*>& DHFrame::Get_Child_DHNode() const
{
	return Child_DHNode;
}

const Eigen::Matrix4d& DHFrame::Get_TFMat() const
{
	return TFMat;
}

int DHFrame::Get_DependActive_ID() const
{
	return DependActive_ID;
}

void DHFrame::Set_qList_Index(int index)
{
	qList_Index = index;
}

int DHFrame::Get_qList_Index() const
{
	return qList_Index;
}

double DHFrame::Get_DH_a() const { return a; }
double DHFrame::Get_DH_alpha() const { return alpha; }
double DHFrame::Get_DH_d() const { return d; }
double DHFrame::Get_DH_Offset_theta() const { return theta; }
double DHFrame::Get_Ratio() const { return ratio; }

void DHFrame::Get_FrameInfo()
{
	printf("FrameInfo�G ");
	
	if(FrameMode == ACTIVE_FRAME)
	{	
		printf("Parent_ID=%d, Self_ID=%d, ACTIVE_FRAME\n", Parent_ID, Self_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, cmd=%f\n", a, alpha, d, theta, cmd);
		
		if(HaveJointLimit)
			printf("joint_min = %f, joint_max = %f\n", joint_min, joint_max);
	}
	else if(FrameMode == PASSIVE_FRAME)
	{	
		printf("Parent_ID=%d, Self_ID=%d, DependActive_ID=%d, PASSIVE_FRAME\n", Parent_ID, Self_ID, DependActive_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, ratio=%f, cmd=%f\n", a, alpha, d, theta, ratio, cmd);
	}
	else // FrameMode == CONST_FRAME
	{	
		printf("Parent_ID=%d, Self_ID=%d, CONST_FRAME\n", Parent_ID, Self_ID);	
	}
	printf("qList_Index = %d\n", qList_Index);

	printf("Child_DHNode�G");
	for(int i=0; i < Child_DHNode.size(); i++)
		printf("%d ", Child_DHNode[i]->Self_ID);
	printf("\n");

	std::cout << "TFMat = \n" << TFMat << std::endl << std::endl;
}

void DHFrame::Set_Command(double New_Cmd)
{
	if(FrameMode == ACTIVE_FRAME)
		cmd = New_Cmd;
}

void DHFrame::Set_Command()
{
	if(FrameMode == PASSIVE_FRAME)
		cmd = ratio * DependActive_DHNode->cmd;
}

double DHFrame::Get_Command() const
{
	return cmd;
}

void DHFrame::Check_JointLimit(double& New_Cmd)
{
	if(HaveJointLimit)
	{
		if(New_Cmd > joint_max)
			New_Cmd = joint_max;
		else if(New_Cmd < joint_min)
			New_Cmd = joint_min;
	}
}


// �H���j���覡�h����FK��s�A�ðO���bFrame_List
void DHFrame::Get_FK(const Eigen::Matrix4d& Parent_T)
{
	// ���o�̷s�� TFMat
	(this->*UpdateMat_ptr)(Parent_T);
	
	for(int i=0, n=Child_DHNode.size(); i < n; i++)
	{
		Child_DHNode[i]->Get_FK(TFMat);
	}
}


inline void DHFrame::UpdateMat(double New_theta)  // ��s TFMat (��e�@�b)
{
	/*******************************
	
	TFMat <<  cos(New_theta), -sin(New_theta)*cos(alpha),  sin(New_theta)*sin(alpha),   a*cos(New_theta),
			  sin(New_theta),  cos(New_theta)*cos(alpha),  -cos(New_theta)*sin(alpha),  a*sin(New_theta),
			  0,			   sin(alpha),                 cos(alpha),                  d,
			  0,			   0,						   0,							1;
	
	*******************************/
	
	double *Mat_data = TFMat.data(); // Column Major

	Mat_data[0] = cos(New_theta);
	Mat_data[1] = sin(New_theta);
	Mat_data[2] = 0;

	Mat_data[4] = -sin(New_theta)*cos(alpha);
	Mat_data[5] = cos(New_theta)*cos(alpha);
	Mat_data[6] = sin(alpha);

	Mat_data[8] = sin(New_theta)*sin(alpha);
	Mat_data[9] = -cos(New_theta)*sin(alpha);
	Mat_data[10] = cos(alpha);

	Mat_data[12] = a*cos(New_theta);
	Mat_data[13] = a*sin(New_theta);
	Mat_data[14] = d;
}

void DHFrame::UpdateMat1(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * TFMat
{
	UpdateMat(theta + cmd);   // ��s TFMat (�۹�e�@�b)
	TFMat = Parent_T * TFMat; // �۹�Root Frame
}

void DHFrame::UpdateMat2(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * Front_T * TFMat
{
	UpdateMat(theta + cmd);             // ��s TFMat (��e�@�b)
	TFMat = Parent_T * Front_T * TFMat; // �۹�Root Frame
}

void DHFrame::UpdateMat3(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * TFMat * back_T
{
	UpdateMat(theta + cmd);            // ��s TFMat (��e�@�b)
	TFMat = Parent_T * TFMat * back_T; // �۹�Root Frame
}

void DHFrame::UpdateMat4(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * Front_T * TFMat * back_T
{
	UpdateMat(theta + cmd);                      // ��s TFMat (��e�@�b)
	TFMat = Parent_T * Front_T * TFMat * back_T; // �۹�Root Frame
}

void DHFrame::UpdateMat5(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * Const Frame
{
	TFMat = Parent_T * ConstFrame;
}


#pragma endregion


#pragma region EndEffector ��@

EndEffector::EndEffector()
{
	EndFrame = NULL;
	EEP.resize(6);
}

EndEffector::EndEffector(DHFrame* _EndFrame)
{
	EndFrame = _EndFrame;
	EEP.resize(6);
	Search_Path();
}

EndEffector::EndEffector(DHFrame* _EndFrame, const std::vector<DHFrame*>& _Path)
{
	EndFrame = _EndFrame;
	Path = _Path;
	EEP.resize(6);
}

EndEffector::~EndEffector()
{

}

void EndEffector::Set_EndFrame(DHFrame* _EndFrame)
{
	EndFrame = _EndFrame;
	Search_Path();
}

DHFrame* EndEffector::Get_EndFrame() const
{
	return EndFrame;
}

void EndEffector::Print_Path()
{
	printf("EndEffector Path�G\n");
	for(int i=0, n=Path.size(); i < n; i++)
		printf("%d ", Path[i]->Self_ID);
	printf("\n");
}

const std::vector<DHFrame*>& EndEffector::Get_Path()
{
	return Path;
}

const Eigen::VectorXd& EndEffector::Get_EEP()
{
	// Get_PosOri(EEP, EndFrame->TFMat);
	Get_PosOri(EEP.data(), EndFrame->TFMat.data());
	return EEP;
}

const Eigen::Matrix4d& EndEffector::Get_EETFMat() const
{
	return EndFrame->TFMat;
}

void EndEffector::Search_Path()
{
	Path.clear();
	
	Path.push_back(EndFrame);

	DHFrame* Parent = EndFrame->Parent_DHNode;

	while (Parent != NULL)
	{
		Path.push_back(Parent);
		Parent = Parent->Parent_DHNode;
	}

	// vector����
	std::reverse(Path.begin(), Path.end());
}

#pragma endregion


#pragma region Robot ��@

Robot::Robot()
{
	Active_DOF  = 0;
	Passive_DOF = 0;
	EE_SIZE     = 0;
	Root_DHFrame = NULL;
	Base_TFMat.setIdentity();

	EE_Value[0]   = 0;	EE_Value[1]   = 0;
	Damp_Value[0] = 0;	Damp_Value[1] = 0;
}

Robot::~Robot()
{
	// ����Ŷ�
	for(int i=0, n=ActiveFrame_Store.size(); i < n ; i++)
		delete ActiveFrame_Store[i];
	
	for(int i=0, n=PassiveFrame_Store.size(); i < n ; i++)
		delete PassiveFrame_Store[i];

	for(int i=0, n=ConstFrame_Store.size(); i < n ; i++)
		delete ConstFrame_Store[i];

	for(int i=0, n=EndEffector_Store.size(); i < n ; i++)
		delete EndEffector_Store[i];
}

void Robot::AddActiveFrame(DHFrame* ActiveFrame)
{
	// �ˬdFrame Mode
	if(ActiveFrame->FrameMode != ACTIVE_FRAME)
	{	
		printf("Robot Error�GAddActiveFrame()�ϥο��~\n");
		assert(false);
	}

	ActiveFrame_Store.push_back(ActiveFrame);
	Active_DOF++;
}

void Robot::AddActiveFrame(DHFrame* ActiveFrame, double joint_min, double joint_max)
{
	// �ˬdFrame Mode
	if(ActiveFrame->FrameMode != ACTIVE_FRAME)
	{	
		printf("Robot Error�GAddActiveFrame()�ϥο��~\n");
		assert(false);
	}
	
	ActiveFrame->Set_JointLimit(joint_min, joint_max);
	ActiveFrame_Store.push_back(ActiveFrame);
	Active_DOF++;
}

void Robot::AddPassiveFrame(DHFrame* PassiveFrame)
{
	// �ˬdFrame Mode
	if(PassiveFrame->FrameMode != PASSIVE_FRAME)
	{	
		printf("Robot Error�GAddPassiveFrame()�ϥο��~\n");
		assert(false);
	}
	
	PassiveFrame_Store.push_back(PassiveFrame);
	Passive_DOF++;
}

void Robot::AddConstFrame(DHFrame* ConstFrame)
{
	// �ˬdFrame Mode
	if(ConstFrame->FrameMode != CONST_FRAME)
	{	
		printf("Robot Error�GAddConstFrame()�ϥο��~\n");
		assert(false);
	}
	
	ConstFrame_Store.push_back(ConstFrame);
}

bool Robot::Build_Robot()
{
	// Robot_q ���Ǩ̷� ActiveFrame �� SelfID �Ѥp��j
	Robot_q = Eigen::VectorXd::Zero(Active_DOF);

	std::sort(ActiveFrame_Store.begin(), ActiveFrame_Store.end(), Sort_SelfID);
	for(int i=0, n=ActiveFrame_Store.size(); i < n; i++)
		ActiveFrame_Store[i]->Set_qList_Index(i);


	// �̷�SelfID�Ѥp��j�ƧǡA SelfID���i����
	Frame_Tree.clear();
	std::map<int, DHFrame*>::iterator Tree_it;
	

	for(int i=0, n=ActiveFrame_Store.size(); i < n; i++)
	{
		// �M���W���]�w
		ActiveFrame_Store[i]->Set_Parent_DHNode(NULL);
		ActiveFrame_Store[i]->Clear_Child_DHNode();

		// �ˬd�O�_������SelfID
		Tree_it = Frame_Tree.find(ActiveFrame_Store[i]->Self_ID);
		
		if(Tree_it == Frame_Tree.end())
			Frame_Tree[ActiveFrame_Store[i]->Self_ID] = ActiveFrame_Store[i];
		else
		{
			printf("Robot Error�GSelfID����\n");
			printf("�b SelfID=%d\n", Tree_it->first);
			assert(false);
		}
	}

	for(int i=0, n=PassiveFrame_Store.size(); i < n; i++)
	{
		// �M���W���]�w
		PassiveFrame_Store[i]->Set_Parent_DHNode(NULL);
		PassiveFrame_Store[i]->Clear_Child_DHNode();

		
		// �ˬd�O�_������SelfID
		Tree_it = Frame_Tree.find(PassiveFrame_Store[i]->Self_ID);
		
		if(Tree_it == Frame_Tree.end())
			Frame_Tree[PassiveFrame_Store[i]->Self_ID] = PassiveFrame_Store[i];
		else
		{
			printf("Robot Error�GSelfID����\n");
			printf("�b SelfID=%d\n", Tree_it->first);
			assert(false);
		}
	}

	for(int i=0, n=ConstFrame_Store.size(); i < n; i++)
	{
		// �M���W���]�w
		ConstFrame_Store[i]->Set_Parent_DHNode(NULL);
		ConstFrame_Store[i]->Clear_Child_DHNode();
		
		// �ˬd�O�_������SelfID
		Tree_it = Frame_Tree.find(ConstFrame_Store[i]->Self_ID);
		
		if(Tree_it == Frame_Tree.end())
			Frame_Tree[ConstFrame_Store[i]->Self_ID] = ConstFrame_Store[i];
		else
		{
			printf("Robot Error�GSelfID����\n");
			printf("�b SelfID=%d\n", Tree_it->first);
			assert(false);
		}
	}

	// �N�Q��Frame�s���̪ۨ��D��Frame
	for(int i=0, n=PassiveFrame_Store.size(); i < n; i++)
	{	
		std::map<int, DHFrame*>::iterator it = Frame_Tree.find(PassiveFrame_Store[i]->Get_DependActive_ID());
		
		if(it == Frame_Tree.end())
		{
			printf("Robot Error�G�̥ۨD��Frame�s�����~ Parent_ID=%d, Self_ID=%d\n", PassiveFrame_Store[i]->Parent_ID, PassiveFrame_Store[i]->Self_ID);
			assert(false);
		}
		else
		{	
			PassiveFrame_Store[i]->Set_DependActive_DHNode(it->second);
			PassiveFrame_Store[i]->Set_qList_Index(it->second->Get_qList_Index());
		}
	}

	// �}�l�إߤ��l�s��
	for(Tree_it = Frame_Tree.begin(); Tree_it != Frame_Tree.end(); Tree_it++)
	{
		if(Tree_it == Frame_Tree.begin())
			Root_DHFrame = Tree_it->second;
		else
		{
			int ParentID = Tree_it->second->Parent_ID;

			std::map<int, DHFrame*>::iterator it = Frame_Tree.find(ParentID);
			
			if(it == Frame_Tree.end())
			{
				printf("Robot Error�G�s�����~�A�䤣����`�I Parent_ID=%d, Self_ID=%d\n", (Tree_it->second)->Parent_ID, (Tree_it->second)->Self_ID);
				assert(false);
			}
			else
			{
				Tree_it->second->Set_Parent_DHNode(it->second);
				it->second->Add_Child_DHNode(Tree_it->second);
			}
		}
	}

	// �M�� EndEffector Path
	std::vector<DHFrame*> _Path;	_Path.reserve(10);
	
	// �M���ª�EndEffector*
	for(int i=0, n=EndEffector_Store.size(); i < n ; i++)
		delete EndEffector_Store[i];
	
	EndEffector_Store.clear(); 

	Search_EE(Root_DHFrame, _Path);

	EE_SIZE = EndEffector_Store.size();

	#pragma region ��l�ƩҦ� Frame::TFMat
			
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
			
	#pragma endregion

	return true;
}

void Robot::Get_RobotInfo()
{
	printf("Robot Frame Info�G\n");
	printf("*****************************************\n");
	Frame_Traversal(Root_DHFrame);
	printf("*****************************************\n");
}

bool Robot::Set_BaseTFMat(const Eigen::Matrix4d& Src_BaseTFMat)
{
	// �ˬd�O�_�� Transformation Matrix
	/*****************************
	Column Major
	
	   [0 4  8 12]   [     |   ]
	T: [1 5  9 13] = [  R  | P ]
       [2 6 10 14]   [-----|---]
   	   [3 7 11 15]   [  0  | 1 ]
	*****************************/
	
	// �z�Q temp = 1 + 0 + 0 + 0 + 1 = 2
	double temp = Src_BaseTFMat.block<3,3>(0,0).determinant() + Src_BaseTFMat(3,0) + Src_BaseTFMat(3,1) + Src_BaseTFMat(3,2) + Src_BaseTFMat(3,3);
	
	if(abs(temp-2.0) > 1e-12)
		return false;

	Base_TFMat = Src_BaseTFMat;

	// ��sFK
	if(Root_DHFrame != NULL)
		Root_DHFrame->Get_FK(Base_TFMat);

	return true;
}

void Robot::Get_BaseTFMat(Eigen::Matrix4d& Dest_BaseTFMat) const
{
	Dest_BaseTFMat = Base_TFMat;
}

const Eigen::Matrix4d& Robot::Get_BaseTFMat() const
{
	return Base_TFMat;
}

unsigned int Robot::DOF_Size() const
{
	return Active_DOF;
}

unsigned int Robot::EE_Size() const
{
	return EE_SIZE;
}

void Robot::Set_q(Eigen::VectorXd& Src_q)
{
	if(Src_q.rows() != Active_DOF)
		return;
	
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		ActiveFrame_Store[i]->Check_JointLimit(Src_q(i));
		ActiveFrame_Store[i]->Set_Command(Src_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);

	Robot_q = Src_q;
}

void Robot::Reset_q()
{
	Robot_q.setZero();
	
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
}

void  Robot::Get_q(Eigen::VectorXd& Dest_q) const
{
	Dest_q = Robot_q;
}

const Eigen::VectorXd& Robot::Get_q() const
{
	return Robot_q;
}

const std::vector<DHFrame*>& Robot::Get_ActiveFrame() const
{
	return ActiveFrame_Store;
}

const std::vector<DHFrame*>& Robot::Get_PassiveFrame() const
{
	return PassiveFrame_Store;
}

const std::vector<DHFrame*>& Robot::Get_ConstFrame() const
{
	return ConstFrame_Store;
}

const std::vector<EndEffector*>& Robot::Get_EndEffector() const
{
	return EndEffector_Store;
}

DHFrame* Robot::Get_RootDHFrame() const
{
	return Root_DHFrame;
}

void Robot::Get_JacobianNoOri(Eigen::MatrixXd& J) const
{
	J = Eigen::MatrixXd::Zero(3*EE_SIZE, Active_DOF);
	DHFrame* Parent = NULL;
	
	for(int i=0; i < EE_SIZE; i++)
	{
		const std::vector<DHFrame*>& Path = EndEffector_Store[i]->Get_Path();
		Eigen::Vector3d EE_Pos = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
		
		for(int j=0, n=Path.size(); j < n; j++)
		{
			// �D�� Frame
			if(Path[j]->FrameMode == ACTIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();
				
				if(Parent == NULL)
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
				else
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
			}
			// �Q�� Frame
			else if(Path[j]->FrameMode == PASSIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();

				if(Parent == NULL)
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
				else
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
			}
		}
	}
}

void Robot::Get_Jacobian(Eigen::MatrixXd& J) const
{
	J = Eigen::MatrixXd::Zero(6*EE_SIZE, Active_DOF);
	DHFrame* Parent = NULL;

	for(int i=0; i < EE_SIZE; i++)
	{
		const std::vector<DHFrame*>& Path = EndEffector_Store[i]->Get_Path();
		Eigen::Vector3d EE_Pos = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
		
		for(int j=0, n=Path.size(); j < n; j++)
		{
			// �D�� Frame
			if(Path[j]->FrameMode == ACTIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();
				
				if(Parent == NULL)
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Base_TFMat.block<3,1>(0,2);											    // z
				}
				else
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Parent->Get_TFMat().block<3,1>(0,2);
				}
			}
			// �Q�� Frame
			else if(Path[j]->FrameMode == PASSIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();

				if(Parent == NULL)
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Path[j]->Get_Ratio() * Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Base_TFMat.block<3,1>(0,2);                                              // z 
				}
				else
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Path[j]->Get_Ratio() * Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Parent->Get_TFMat().block<3,1>(0,2);
				}
			}
		}
	}
}

void Robot::Get_JacobianNoOri(Eigen::MatrixXd& J, const std::vector<EndEffector*>& UserDefEE) const
{
	unsigned int ee_size = UserDefEE.size();
	
	J = Eigen::MatrixXd::Zero(3*ee_size, Active_DOF);
	DHFrame* Parent = NULL;

	for(int i=0; i < ee_size; i++)
	{
		const std::vector<DHFrame*>& Path = UserDefEE[i]->Get_Path();
		Eigen::Vector3d EE_Pos = UserDefEE[i]->Get_EETFMat().block<3,1>(0,3);
		
		for(int j=0, n=Path.size(); j < n; j++)
		{
			// �D�� Frame
			if(Path[j]->FrameMode == ACTIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();
				
				if(Parent == NULL)
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
				else
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
			}
			// �Q�� Frame
			else if(Path[j]->FrameMode == PASSIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();

				if(Parent == NULL)
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
				else
					J.block<3,1>(3*i, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
			}
		}
	}
}

void Robot::Get_Jacobian(Eigen::MatrixXd& J, const std::vector<EndEffector*>& UserDefEE) const
{
	unsigned int ee_size = UserDefEE.size();
	
	J = Eigen::MatrixXd::Zero(6*ee_size, Active_DOF);
	DHFrame* Parent = NULL;

	for(int i=0; i < ee_size; i++)
	{
		const std::vector<DHFrame*>& Path = UserDefEE[i]->Get_Path();
		Eigen::Vector3d EE_Pos = UserDefEE[i]->Get_EETFMat().block<3,1>(0,3);
		
		for(int j=0, n=Path.size(); j < n; j++)
		{
			// �D�� Frame
			if(Path[j]->FrameMode == ACTIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();
				
				if(Parent == NULL)
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Base_TFMat.block<3,1>(0,2);			                                    // z
				}
				else
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Parent->Get_TFMat().block<3,1>(0,2);
				}
			}
			// �Q�� Frame
			else if(Path[j]->FrameMode == PASSIVE_FRAME)
			{
				Parent = Path[j]->Get_Parent_DHNode();

				if(Parent == NULL)
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Path[j]->Get_Ratio() * Base_TFMat.block<3,1>(0,2).cross( EE_Pos - Base_TFMat.block<3,1>(0,3) ); // z cross r
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Base_TFMat.block<3,1>(0,2);                                              // z 
				}
				else
				{	
					J.block<3,1>(6*i, Path[j]->Get_qList_Index())   += Path[j]->Get_Ratio() * Parent->Get_TFMat().block<3,1>(0,2).cross( EE_Pos - Parent->Get_TFMat().block<3,1>(0,3) );
					J.block<3,1>(6*i+3, Path[j]->Get_qList_Index()) += Path[j]->Get_Ratio() * Parent->Get_TFMat().block<3,1>(0,2);
				}
			}
		}
	}
}


void Robot::Set_DLS_Parameter(double EE_min, double Damp_min, double EE_max, double Damp_max)
{
	// �]�w EE_Value[2] = {EE_min, EE_max}, Damp_Value[2] = {Damp_min, Damp_max}
	EE_Value[0] = EE_min;
	EE_Value[1] = EE_max;
	Damp_Value[0] = Damp_min;
	Damp_Value[1] = Damp_max;
}

double Robot::Get_DLS_Damp(double EE_Size)
{
	if(EE_Value[0] == EE_Value[1])
		return 0;
	else
		return ( (Damp_Value[1]-Damp_Value[0]) / (EE_Value[1]-EE_Value[0]) * (EE_Size-EE_Value[0]) + Damp_Value[0] );
}


bool Robot::dlsIK(const Vector3dList& TargetPos)
{
	// �ˬd
	if(TargetPos.size() != EE_SIZE)
	{	
		printf("Robot Error: TargetPos.size() != EE_SIZE\n");
		assert(false);
	}
	
	Eigen::VectorXd targetPos(3*EE_SIZE); // �ؼ�Pos
	Eigen::VectorXd nowPos(3*EE_SIZE);    // �{�bPos
	Eigen::VectorXd dp(3*EE_SIZE);        // delta P
	Eigen::VectorXd dq(Active_DOF);       // delta q
	unsigned int J_rows = 3*EE_SIZE;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(EE_SIZE), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;                 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;                           // �~�tdp.norm()�p�󵥩�0.0001
	
	unsigned int iter = 0;               // ���N����
	

	for(int i=0; i < EE_SIZE; i++)
	{	
		targetPos.block<3,1>(3*i, 0) = TargetPos[i];
		nowPos.block<3,1>(3*i, 0) = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
	}
	
	dp = targetPos - nowPos;

	if(J_rows <= J_cols)// Jacobian�O�D�x�}
	{
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_JacobianNoOri(J);

			ClampMag(dp, false);

			// ��Xdq
			dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;
			
			Robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
				ActiveFrame_Store[i]->Set_Command(Robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < EE_SIZE; i++)
				nowPos.block<3,1>(3*i, 0) = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
			
			dp = targetPos - nowPos;

			iter++;
		}
	}
	else  // Jacobian�O�G�x�}
	{
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_JacobianNoOri(J);

			ClampMag(dp, false);

			// ��Xdq
			dq = ( J.transpose()*J + lambda * Eigen::MatrixXd::Identity(J_cols,J_cols) ).inverse() * J.transpose() * dp;
			
			Robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
				ActiveFrame_Store[i]->Set_Command(Robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < EE_SIZE; i++)
				nowPos.block<3,1>(3*i, 0) = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
			
			dp = targetPos - nowPos;

			iter++;
		}
	} 
	
	//printf("IK iter = %d\n", iter);
	//printf("dp.norm() = %f\n", dp.norm());

	if(iter >= Max_Iter)
		return false;
	else	
		return true;
}

bool Robot::dlsIK(const VectorXdList& TargetPosOri)
{
	// �ˬd
	if(TargetPosOri.size() != EE_SIZE)
	{	
		printf("Robot Error: TargetPosOri.size() != EE_SIZE\n");
		assert(false);
	}
	
	double  *targetTFMat   = new double[16*EE_SIZE]; // �O���ؼ� PosOri ��TFMat�AColumn Major
	double* *nowTFMat_data = new double*[EE_SIZE];   // �O���{�bPosOri data ��m

	Eigen::VectorXd dp(6*EE_SIZE);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*EE_SIZE;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(EE_SIZE), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;                 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;                           // �~�tdp.norm()�p�󵥩�0.0001
	
	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();
	
	for(int i=0; i < EE_SIZE; i++)
	{	
		if(TargetPosOri[i].rows() != 6)
		{
			printf("Robot Error�G TargetPosOri[%d].rows() != 6\n", i);
			delete[] targetTFMat, nowTFMat_data;
			assert(false);
		}
		
		Get_TFMatrix(targetTFMat + 16*i, TargetPosOri[i].data());
		nowTFMat_data[i] = const_cast<double*>(EndEffector_Store[i]->Get_EETFMat().data());
		Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);
	}
	

	if(J_rows <= J_cols)// Jacobian�O�D�x�}
	{
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_Jacobian(J);

			ClampMag(dp, true);

			// ��Xdq
			dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;
			
			Robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
				ActiveFrame_Store[i]->Set_Command(Robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < EE_SIZE; i++)
				Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);

			iter++;
		}
	}
	else  // Jacobian�O�G�x�}
	{
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_Jacobian(J);

			ClampMag(dp, true);

			// ��Xdq
			dq = ( J.transpose()*J + lambda * Eigen::MatrixXd::Identity(J_cols,J_cols) ).inverse() * J.transpose() * dp;
			
			Robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
				ActiveFrame_Store[i]->Set_Command(Robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < EE_SIZE; i++)
				Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);

			iter++;
		}
	} 
	
	//printf("IK iter = %d\n", iter);
	//printf("dp.norm() = %f\n", dp.norm());

	delete[] targetTFMat, nowTFMat_data;

	if(iter >= Max_Iter)
		return false;
	else
		return true;
}

bool Robot::dlsIK(const Matrix4dList& TargetPosOri)
{
	// �ˬd
	if(TargetPosOri.size() != EE_SIZE)
	{	
		printf("Robot Error: TargetPosOri.size() != EE_SIZE\n");
		assert(false);
	}
	
	double* *targetTFMat_data = new double*[EE_SIZE]; // �O���ؼ� PosOri data ��m
	double* *nowTFMat_data    = new double*[EE_SIZE]; // �O���{�b PosOri data ��m

	Eigen::VectorXd dp(6*EE_SIZE);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*EE_SIZE;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(EE_SIZE), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;                 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;                           // �~�tdp.norm()�p�󵥩�0.001

	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();
	
	for(int i=0; i < EE_SIZE; i++)
	{	
		targetTFMat_data[i] = const_cast<double*>(TargetPosOri[i].data());
		nowTFMat_data[i]    = const_cast<double*>(EndEffector_Store[i]->Get_EETFMat().data());
		Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
	}
	

	if(J_rows <= J_cols)// Jacobian�O�D�x�}
	{
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_Jacobian(J);

			ClampMag(dp, true);

			// ��Xdq
			dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

			Robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
				ActiveFrame_Store[i]->Set_Command(Robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < EE_SIZE; i++)
				Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
			
			iter++;
		}
	}
	else  // Jacobian�O�G�x�}
	{
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_Jacobian(J);

			ClampMag(dp, true);

			// ��Xdq
			dq = ( J.transpose()*J + lambda * Eigen::MatrixXd::Identity(J_cols,J_cols) ).inverse() * J.transpose() * dp;
			
			Robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
				ActiveFrame_Store[i]->Set_Command(Robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < EE_SIZE; i++)
				Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);

			iter++;
		}
	} 
	
	//printf("IK iter = %d\n", iter);
	//printf("dp.norm() = %f\n", dp.norm());

	delete[] targetTFMat_data, nowTFMat_data;

	if(iter >= Max_Iter)
		return false;
	else
		return true;
}

void Robot::dlsIK(const std::vector<Vector3dList>& EE_Trajectory, VectorXdList& q_Trajectory)
{
	unsigned int Number = EE_Trajectory.size();
	
	q_Trajectory.clear();         // �M�ťH�e����
	q_Trajectory.reserve(Number); // �w���t�m�Ŷ�

	Eigen::VectorXd robot_q = Robot_q;

	Eigen::VectorXd targetPos(3*EE_SIZE); // �ؼ�Pos
	Eigen::VectorXd nowPos(3*EE_SIZE);    // �{�bPos
	Eigen::VectorXd dp(3*EE_SIZE);        // delta P
	Eigen::VectorXd dq(Active_DOF);       // delta q
	unsigned int J_rows = 3*EE_SIZE;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(EE_SIZE), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;				 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;							 // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����

	// �ˬd
	for(int i=0; i < Number; i++)
	{
		if(EE_Trajectory[i].size() != EE_SIZE)
		{	
			printf("Robot Error: EE_Trajectory[%d].size() != EE_SIZE\n", i);
			assert(false);
		}
	}

	for(int i=0; i < EE_SIZE; i++)
		nowPos.block<3,1>(3*i, 0) = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
	

	if(J_rows <= J_cols)// Jacobian�O�D�x�}
	{
		for(int num = 0; num < Number; num++)
		{
			for(int i=0; i < EE_SIZE; i++)
				targetPos.block<3,1>(3*i, 0) = EE_Trajectory[num][i];
			
			dp = targetPos - nowPos;
			
			while (iter < Max_Iter && dp.norm() > err)
			{
				// �����{�b���A�� Jacobian
				Get_JacobianNoOri(J);

				ClampMag(dp, false);

				// ��Xdq
				dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

				robot_q += dq;
			
				#pragma region �p��FK
			
				// ����s�D��Frame cmd
				for(int i=0; i < Active_DOF; i++)
				{	
					// ����Joint limit�ˬd
					ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
					ActiveFrame_Store[i]->Set_Command(robot_q(i));
				}
	
				// �A��s�Q��Frame cmd
				for(int i=0; i < Passive_DOF; i++)
					PassiveFrame_Store[i]->Set_Command();

				Root_DHFrame->Get_FK(Base_TFMat);
			
				#pragma endregion

				for(int i=0; i < EE_SIZE; i++)
					nowPos.block<3,1>(3*i, 0) = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
			
				dp = targetPos - nowPos;

				iter++;
			}

			iter = 0;
			q_Trajectory.push_back(robot_q);
		}
	}
	else  // Jacobian�O�G�x�}
	{
		for(int num = 0; num < Number; num++)
		{
			for(int i=0; i < EE_SIZE; i++)
				targetPos.block<3,1>(3*i, 0) = EE_Trajectory[num][i];
			
			dp = targetPos - nowPos;

			while (iter < Max_Iter && dp.norm() > err)
			{
				// �����{�b���A�� Jacobian
				Get_JacobianNoOri(J);

				ClampMag(dp, false);

				// ��Xdq
				dq = ( J.transpose()*J + lambda * Eigen::MatrixXd::Identity(J_cols,J_cols) ).inverse() * J.transpose() * dp;
			
				robot_q += dq;
			
				#pragma region �p��FK
			
				// ����s�D��Frame cmd
				for(int i=0; i < Active_DOF; i++)
				{	
					// ����Joint limit�ˬd
					ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
					ActiveFrame_Store[i]->Set_Command(robot_q(i));
				}
	
				// �A��s�Q��Frame cmd
				for(int i=0; i < Passive_DOF; i++)
					PassiveFrame_Store[i]->Set_Command();

				Root_DHFrame->Get_FK(Base_TFMat);
			
				#pragma endregion

				for(int i=0; i < EE_SIZE; i++)
					nowPos.block<3,1>(3*i, 0) = EndEffector_Store[i]->Get_EETFMat().block<3,1>(0,3);
			
				dp = targetPos - nowPos;

				iter++;
			}

			iter = 0;
			q_Trajectory.push_back(robot_q);
		}
	} 
	
	#pragma region �^�_��l���A Robot_q
			
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		// ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
			
	#pragma endregion
}

void Robot::dlsIK(const std::vector<VectorXdList>& EE_Trajectory, VectorXdList& q_Trajectory)
{
	unsigned int Number = EE_Trajectory.size();
	
	q_Trajectory.clear();         // �M�ťH�e����
	q_Trajectory.reserve(Number); // �w���t�m�Ŷ�

	Eigen::VectorXd robot_q = Robot_q;

	double  *targetTFMat   = new double[16*EE_SIZE]; // �O���ؼ� PosOri ��TFMat�AColumn Major
	double* *nowTFMat_data = new double*[EE_SIZE];   // �O���{�bPosOri data ��m
	
	Eigen::VectorXd dp(6*EE_SIZE);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*EE_SIZE;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(EE_SIZE), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;				 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;							 // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();

	// �ˬd
	for(int i=0; i < Number; i++)
	{
		if(EE_Trajectory[i].size() != EE_SIZE)
		{	
			printf("Robot Error: EE_Trajectory[%d].size() != EE_SIZE\n", i);
			delete[] targetTFMat, nowTFMat_data;
			assert(false);
		}

		for(int j=0; j < EE_SIZE; j++)
		{	
			if(EE_Trajectory[i][j].rows() != 6)
			{	
				printf("Robot Error: EE_Trajectory[%d][%d].rows() != 6\n", i, j);
				delete[] targetTFMat, nowTFMat_data;
				assert(false);
			}
		}
	}

	for(int i=0; i < EE_SIZE; i++)
		nowTFMat_data[i] = const_cast<double*>(EndEffector_Store[i]->Get_EETFMat().data());
	

	if(J_rows <= J_cols)// Jacobian�O�D�x�}
	{
		for(int num = 0; num < Number; num++)
		{
			for(int i=0; i < EE_SIZE; i++)
			{	
				Get_TFMatrix(targetTFMat + 16*i, EE_Trajectory[num][i].data());
				Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);
			}
			
			while (iter < Max_Iter && dp.norm() > err)
			{
				// �����{�b���A�� Jacobian
				Get_Jacobian(J);

				ClampMag(dp, true);

				// ��Xdq
				dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

				robot_q += dq;
			
				#pragma region �p��FK
			
				// ����s�D��Frame cmd
				for(int i=0; i < Active_DOF; i++)
				{	
					// ����Joint limit�ˬd
					ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
					ActiveFrame_Store[i]->Set_Command(robot_q(i));
				}
	
				// �A��s�Q��Frame cmd
				for(int i=0; i < Passive_DOF; i++)
					PassiveFrame_Store[i]->Set_Command();

				Root_DHFrame->Get_FK(Base_TFMat);
			
				#pragma endregion

				for(int i=0; i < EE_SIZE; i++)
					Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);

				iter++;
			}

			iter = 0;
			q_Trajectory.push_back(robot_q);
		}
	}
	else  // Jacobian�O�G�x�}
	{
		for(int num = 0; num < Number; num++)
		{
			for(int i=0; i < EE_SIZE; i++)
			{	
				Get_TFMatrix(targetTFMat + 16*i, EE_Trajectory[num][i].data());
				Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);
			}

			while (iter < Max_Iter && dp.norm() > err)
			{
				// �����{�b���A�� Jacobian
				Get_Jacobian(J);

				ClampMag(dp, true);

				// ��Xdq
				dq = ( J.transpose()*J + lambda * Eigen::MatrixXd::Identity(J_cols,J_cols) ).inverse() * J.transpose() * dp;
			
				robot_q += dq;
			
				#pragma region �p��FK
			
				// ����s�D��Frame cmd
				for(int i=0; i < Active_DOF; i++)
				{	
					// ����Joint limit�ˬd
					ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
					ActiveFrame_Store[i]->Set_Command(robot_q(i));
				}
	
				// �A��s�Q��Frame cmd
				for(int i=0; i < Passive_DOF; i++)
					PassiveFrame_Store[i]->Set_Command();

				Root_DHFrame->Get_FK(Base_TFMat);
			
				#pragma endregion

				for(int i=0; i < EE_SIZE; i++)
					Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);

				iter++;
			}

			iter = 0;
			q_Trajectory.push_back(robot_q);
		}
	} 
	
	#pragma region �^�_��l���A Robot_q
			
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		// ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
			
	#pragma endregion

	delete[] targetTFMat, nowTFMat_data;
}

void Robot::dlsIK(const std::vector<Matrix4dList>& EE_Trajectory, VectorXdList& q_Trajectory)
{
	unsigned int Number = EE_Trajectory.size();
	
	q_Trajectory.clear();         // �M�ťH�e����
	q_Trajectory.reserve(Number); // �w���t�m�Ŷ�

	Eigen::VectorXd robot_q = Robot_q;

	double* *targetTFMat_data = new double*[EE_SIZE]; // �O���ؼ� PosOri data ��m
	double* *nowTFMat_data    = new double*[EE_SIZE]; // �O���{�b PosOri data ��m

	Eigen::VectorXd dp(6*EE_SIZE);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*EE_SIZE;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(EE_SIZE), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;				 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;							 // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();

	// �ˬd
	for(int i=0; i < Number; i++)
	{
		if(EE_Trajectory[i].size() != EE_SIZE)
		{	
			printf("Robot Error: EE_Trajectory[%d].size() != EE_SIZE\n", i);
			delete[] targetTFMat_data, nowTFMat_data;
			assert(false);
		}
	}

	for(int i=0; i < EE_SIZE; i++)
		nowTFMat_data[i] = const_cast<double*>(EndEffector_Store[i]->Get_EETFMat().data());
	

	if(J_rows <= J_cols)// Jacobian�O�D�x�}
	{
		for(int num = 0; num < Number; num++)
		{
			for(int i=0; i < EE_SIZE; i++)
			{
				targetTFMat_data[i] = const_cast<double*>(EE_Trajectory[num][i].data());
				Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
			}
			
			while (iter < Max_Iter && dp.norm() > err)
			{
				// �����{�b���A�� Jacobian
				Get_Jacobian(J);

				ClampMag(dp, true);

				// ��Xdq
				dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

				robot_q += dq;
			
				#pragma region �p��FK
			
				// ����s�D��Frame cmd
				for(int i=0; i < Active_DOF; i++)
				{	
					// ����Joint limit�ˬd
					ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
					ActiveFrame_Store[i]->Set_Command(robot_q(i));
				}
	
				// �A��s�Q��Frame cmd
				for(int i=0; i < Passive_DOF; i++)
					PassiveFrame_Store[i]->Set_Command();

				Root_DHFrame->Get_FK(Base_TFMat);
			
				#pragma endregion

				for(int i=0; i < EE_SIZE; i++)
					Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
			
				iter++;
			}

			iter = 0;
			q_Trajectory.push_back(robot_q);
		}
	}
	else  // Jacobian�O�G�x�}
	{
		for(int num = 0; num < Number; num++)
		{
			for(int i=0; i < EE_SIZE; i++)
			{
				targetTFMat_data[i] = const_cast<double*>(EE_Trajectory[num][i].data());
				Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
			}

			while (iter < Max_Iter && dp.norm() > err)
			{
				// �����{�b���A�� Jacobian
				Get_Jacobian(J);

				ClampMag(dp, true);

				// ��Xdq
				dq = ( J.transpose()*J + lambda * Eigen::MatrixXd::Identity(J_cols,J_cols) ).inverse() * J.transpose() * dp;
			
				robot_q += dq;
			
				#pragma region �p��FK
			
				// ����s�D��Frame cmd
				for(int i=0; i < Active_DOF; i++)
				{	
					// ����Joint limit�ˬd
					ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
					ActiveFrame_Store[i]->Set_Command(robot_q(i));
				}
	
				// �A��s�Q��Frame cmd
				for(int i=0; i < Passive_DOF; i++)
					PassiveFrame_Store[i]->Set_Command();

				Root_DHFrame->Get_FK(Base_TFMat);
			
				#pragma endregion

				for(int i=0; i < EE_SIZE; i++)
					Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);

				iter++;
			}

			iter = 0;
			q_Trajectory.push_back(robot_q);
		}
	} 
	
	#pragma region �^�_��l���A Robot_q
			
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		// ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
			
	#pragma endregion

	delete[] targetTFMat_data, nowTFMat_data;
}


DHFrame* Robot::Find_UserDef_Frame(int Self_ID)
{
	std::map<int, DHFrame*>::iterator Tree_it = Frame_Tree.find(Self_ID);

	if(Tree_it == Frame_Tree.end())
		return NULL;
	else
		return (Tree_it->second);
}

bool Robot::dlsIK(const Vector3dList& TargetPos, const std::vector<EndEffector*>& UserDefEE)
{
	unsigned int ee_size = UserDefEE.size();
	
	// �ˬd
	if(TargetPos.size() != ee_size || ee_size == 0)
	{	
		printf("Robot Error: TargetPos.size() = %d, UserDefEE.size() = %d\n", TargetPos.size(), ee_size);
		assert(false);
	}
	
	Eigen::VectorXd targetPos(3*ee_size); // �ؼ�Pos
	Eigen::VectorXd nowPos(3*ee_size);    // �{�bPos
	Eigen::VectorXd dp(3*ee_size);        // delta P
	Eigen::VectorXd dq(Active_DOF);       // delta q
	unsigned int J_rows = 3*ee_size;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(ee_size), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;				 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;							 // �~�tdp.norm()�p�󵥩�0.0001
	
	unsigned int iter = 0;               // ���N����
	

	for(int i=0; i < ee_size; i++)
	{	
		targetPos.block<3,1>(3*i, 0) = TargetPos[i];
		nowPos.block<3,1>(3*i, 0) = UserDefEE[i]->Get_EETFMat().block<3,1>(0,3);
	}
	
	dp = targetPos - nowPos;

	// �]���u�������v�T�ϥΪ̫��wEE�ܤƪ�Robot_q���ͧ�s
	// �G����Jacobian�O�D�x�}�άO�G�x�}�A�ҥέD�x�}��inverse�ӭp��
	while (iter < Max_Iter && dp.norm() > err)
	{
		// �����{�b���A�� Jacobian
		Get_JacobianNoOri(J, UserDefEE);

		ClampMag(dp, false);

		// ��Xdq
		dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;
			
		Robot_q += dq;
			
		#pragma region �p��FK
			
		// ����s�D��Frame cmd
		for(int i=0; i < Active_DOF; i++)
		{	
			// ����Joint limit�ˬd
			ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
			ActiveFrame_Store[i]->Set_Command(Robot_q(i));
		}
	
		// �A��s�Q��Frame cmd
		for(int i=0; i < Passive_DOF; i++)
			PassiveFrame_Store[i]->Set_Command();

		Root_DHFrame->Get_FK(Base_TFMat);
			
		#pragma endregion

		for(int i=0; i < ee_size; i++)
			nowPos.block<3,1>(3*i, 0) = UserDefEE[i]->Get_EETFMat().block<3,1>(0,3);
			
		dp = targetPos - nowPos;

		iter++;
	}

	//printf("IK iter = %d\n", iter);
	//printf("dp.norm() = %f\n", dp.norm());

	if(iter >= Max_Iter)
		return false;
	else	
		return true;
}

bool Robot::dlsIK(const VectorXdList& TargetPosOri, const std::vector<EndEffector*>& UserDefEE)
{
	unsigned int ee_size = UserDefEE.size();
	
	// �ˬd
	if(TargetPosOri.size() != ee_size || ee_size == 0)
	{	
		printf("Robot Error: TargetPosOri.size() = %d, UserDefEE.size() = %d\n", TargetPosOri.size(), ee_size);
		assert(false);
	}

	double  *targetTFMat   = new double[16*ee_size]; // �O���ؼ� PosOri ��TFMat�AColumn Major
	double* *nowTFMat_data = new double*[ee_size];   // �O���{�bPosOri data ��m

	Eigen::VectorXd dp(6*ee_size);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*ee_size;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(ee_size), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;				 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;							 // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();

	for(int i=0; i < ee_size; i++)
	{	
		if(TargetPosOri[i].rows() != 6)
		{
			printf("Robot Error�G TargetPosOri[%d].rows() != 6\n", i);
			delete[] targetTFMat, nowTFMat_data;
			assert(false);
		}
		
		Get_TFMatrix(targetTFMat + 16*i, TargetPosOri[i].data());
		nowTFMat_data[i] = const_cast<double*>(UserDefEE[i]->Get_EETFMat().data());
		Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);
	}
	

	// �]���u�������v�T�ϥΪ̫��wEE�ܤƪ�Robot_q���ͧ�s
	// �G����Jacobian�O�D�x�}�άO�G�x�}�A�ҥέD�x�}��inverse�ӭp��
	while (iter < Max_Iter && dp.norm() > err)
	{
		// �����{�b���A�� Jacobian
		Get_Jacobian(J, UserDefEE);

		ClampMag(dp, true);

		// ��Xdq
		dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

		Robot_q += dq;
			
		#pragma region �p��FK
			
		// ����s�D��Frame cmd
		for(int i=0; i < Active_DOF; i++)
		{	
			// ����Joint limit�ˬd
			ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
			ActiveFrame_Store[i]->Set_Command(Robot_q(i));
		}
	
		// �A��s�Q��Frame cmd
		for(int i=0; i < Passive_DOF; i++)
			PassiveFrame_Store[i]->Set_Command();

		Root_DHFrame->Get_FK(Base_TFMat);
			
		#pragma endregion

		for(int i=0; i < ee_size; i++)
			Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);

		iter++;
	}
	
	//printf("IK iter = %d\n", iter);
	//printf("dp.norm() = %f\n", dp.norm());

	delete[] targetTFMat, nowTFMat_data;

	if(iter >= Max_Iter)
		return false;
	else
		return true;
}

bool Robot::dlsIK(const Matrix4dList& TargetPosOri, const std::vector<EndEffector*>& UserDefEE)
{
	unsigned int ee_size = UserDefEE.size();
	
	// �ˬd
	if(TargetPosOri.size() != ee_size || ee_size == 0)
	{	
		printf("Robot Error: TargetPosOri.size() = %d, UserDefEE.size() = %d\n", TargetPosOri.size(), ee_size);
		assert(false);
	}

	double* *targetTFMat_data = new double*[ee_size]; // �O���ؼ� PosOri data ��m
	double* *nowTFMat_data    = new double*[ee_size]; // �O���{�b PosOri data ��m

	Eigen::VectorXd dp(6*ee_size);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*ee_size;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	double lambda = pow(Get_DLS_Damp(ee_size), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;                 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;                           // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();

	for(int i=0; i < ee_size; i++)
	{	
		targetTFMat_data[i] = const_cast<double*>(TargetPosOri[i].data());
		nowTFMat_data[i]    = const_cast<double*>(UserDefEE[i]->Get_EETFMat().data());
		Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
	}
	

	// �]���u�������v�T�ϥΪ̫��wEE�ܤƪ�Robot_q���ͧ�s
	// �G����Jacobian�O�D�x�}�άO�G�x�}�A�ҥέD�x�}��inverse�ӭp��
	while (iter < Max_Iter && dp.norm() > err)
	{
		// �����{�b���A�� Jacobian
		Get_Jacobian(J, UserDefEE);

		ClampMag(dp, true);

		// ��Xdq
		dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

		Robot_q += dq;
			
		#pragma region �p��FK
			
		// ����s�D��Frame cmd
		for(int i=0; i < Active_DOF; i++)
		{	
			// ����Joint limit�ˬd
			ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
			ActiveFrame_Store[i]->Set_Command(Robot_q(i));
		}
	
		// �A��s�Q��Frame cmd
		for(int i=0; i < Passive_DOF; i++)
			PassiveFrame_Store[i]->Set_Command();

		Root_DHFrame->Get_FK(Base_TFMat);
			
		#pragma endregion

		for(int i=0; i < ee_size; i++)
			Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
			
		iter++;
	}

	//printf("IK iter = %d\n", iter);
	//printf("dp.norm() = %f\n", dp.norm());

	delete[] targetTFMat_data, nowTFMat_data;

	if(iter >= Max_Iter)
		return false;
	else
		return true;
}

void Robot::dlsIK(const std::vector<Vector3dList>& EE_Trajectory, VectorXdList& q_Trajectory, const std::vector<EndEffector*>& UserDefEE)
{
	unsigned int ee_size = UserDefEE.size();
	if(ee_size == 0)
	{	printf("Robot Error: UserDefEE.size() = %d\n", ee_size);	return; }

	unsigned int Number = EE_Trajectory.size();
	
	q_Trajectory.clear();         // �M�ťH�e����
	q_Trajectory.reserve(Number); // �w���t�m�Ŷ�

	Eigen::VectorXd robot_q = Robot_q;

	Eigen::VectorXd targetPos(3*ee_size); // �ؼ�Pos
	Eigen::VectorXd nowPos(3*ee_size);    // �{�bPos
	Eigen::VectorXd dp(3*ee_size);        // delta P
	Eigen::VectorXd dq(Active_DOF);       // delta q
	unsigned int J_rows = 3*ee_size;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(ee_size), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;				 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;							 // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����

	// �ˬd
	for(int i=0; i < Number; i++)
	{
		if(EE_Trajectory[i].size() != ee_size)
		{	
			printf("Robot Error: EE_Trajectory[%d].size() != UserDefEE.size()\n", i);
			assert(false);
		}
	}

	for(int i=0; i < ee_size; i++)
		nowPos.block<3,1>(3*i, 0) = UserDefEE[i]->Get_EETFMat().block<3,1>(0,3);
	
	// �]���u�������v�T�ϥΪ̫��wEE�ܤƪ�Robot_q���ͧ�s
	// �G����Jacobian�O�D�x�}�άO�G�x�}�A�ҥέD�x�}��inverse�ӭp��
	for(int num = 0; num < Number; num++)
	{
		for(int i=0; i < ee_size; i++)
			targetPos.block<3,1>(3*i, 0) = EE_Trajectory[num][i];
			
		dp = targetPos - nowPos;
			
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_JacobianNoOri(J, UserDefEE);

			ClampMag(dp, false);

			// ��Xdq
			dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

			robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
				ActiveFrame_Store[i]->Set_Command(robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < ee_size; i++)
				nowPos.block<3,1>(3*i, 0) = UserDefEE[i]->Get_EETFMat().block<3,1>(0,3);
			
			dp = targetPos - nowPos;

			iter++;
		}

		iter = 0;
		q_Trajectory.push_back(robot_q);
	}

	#pragma region �^�_��l���A Robot_q
			
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		// ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
			
	#pragma endregion
}

void Robot::dlsIK(const std::vector<VectorXdList>& EE_Trajectory, VectorXdList& q_Trajectory, const std::vector<EndEffector*>& UserDefEE)
{
	unsigned int ee_size = UserDefEE.size();
	if(ee_size == 0)
	{	printf("Robot Error: UserDefEE.size() = %d\n", ee_size);	return; }

	unsigned int Number = EE_Trajectory.size();
	
	q_Trajectory.clear();         // �M�ťH�e����
	q_Trajectory.reserve(Number); // �w���t�m�Ŷ�

	Eigen::VectorXd robot_q = Robot_q;

	double  *targetTFMat   = new double[16*ee_size]; // �O���ؼ� PosOri ��TFMat�AColumn Major
	double* *nowTFMat_data = new double*[ee_size];   // �O���{�bPosOri data ��m

	Eigen::VectorXd dp(6*ee_size);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*ee_size;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(ee_size), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;				 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;							 // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();

	// �ˬd
	for(int i=0; i < Number; i++)
	{
		if(EE_Trajectory[i].size() != ee_size)
		{	
			printf("Robot Error: EE_Trajectory[%d].size() != UserDefEE.size()\n", i);
			delete[] targetTFMat, nowTFMat_data;
			assert(false);
		}

		for(int j=0; j < ee_size; j++)
		{	
			if(EE_Trajectory[i][j].rows() != 6)
			{	
				printf("Robot Error: EE_Trajectory[%d][%d].rows() != 6\n", i, j);
				delete[] targetTFMat, nowTFMat_data;
				assert(false);
			}
		}
	}

	for(int i=0; i < ee_size; i++)
		nowTFMat_data[i] = const_cast<double*>(UserDefEE[i]->Get_EETFMat().data());
	

	// �]���u�������v�T�ϥΪ̫��wEE�ܤƪ�Robot_q���ͧ�s
	// �G����Jacobian�O�D�x�}�άO�G�x�}�A�ҥέD�x�}��inverse�ӭp��
	for(int num = 0; num < Number; num++)
	{
		for(int i=0; i < ee_size; i++)
		{	
			Get_TFMatrix(targetTFMat + 16*i, EE_Trajectory[num][i].data());
			Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);
		}
			
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_Jacobian(J, UserDefEE);

			ClampMag(dp, true);

			// ��Xdq
			dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

			robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
				ActiveFrame_Store[i]->Set_Command(robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < ee_size; i++)
				Get_DiffTFMat(dp_data + 6*i, targetTFMat + 16*i,  nowTFMat_data[i]);

			iter++;
		}

		iter = 0;
		q_Trajectory.push_back(robot_q);
	}

	#pragma region �^�_��l���A Robot_q
			
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		// ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
			
	#pragma endregion

	delete[] targetTFMat, nowTFMat_data;
}

void Robot::dlsIK(const std::vector<Matrix4dList>& EE_Trajectory, VectorXdList& q_Trajectory, const std::vector<EndEffector*>& UserDefEE)
{
	unsigned int ee_size = UserDefEE.size();
	if(ee_size == 0)
	{	printf("Robot Error: UserDefEE.size() = %d\n", ee_size);	return; }

	unsigned int Number = EE_Trajectory.size();
	
	q_Trajectory.clear();         // �M�ťH�e����
	q_Trajectory.reserve(Number); // �w���t�m�Ŷ�

	Eigen::VectorXd robot_q = Robot_q;

	double* *targetTFMat_data = new double*[ee_size]; // �O���ؼ� PosOri data ��m
	double* *nowTFMat_data    = new double*[ee_size]; // �O���{�b PosOri data ��m

	Eigen::VectorXd dp(6*ee_size);           // delta P
	Eigen::VectorXd dq(Active_DOF);          // delta q
	unsigned int J_rows = 6*ee_size;
	unsigned int J_cols = Active_DOF;
	Eigen::MatrixXd J(J_rows, J_cols);

	const double lambda = pow(Get_DLS_Damp(ee_size), 2); // inverse ���� (�ϥΪ̦ۭq)
	const unsigned int Max_Iter = 10000;                 // IK�`���N���� (�ϥΪ̦ۭq)
	const double err = 0.0001;                           // �~�tdp.norm()�p�󵥩�0.0001

	unsigned int iter = 0;               // ���N����
	
	double* dp_data = dp.data();
	
	// �ˬd
	for(int i=0; i < Number; i++)
	{
		if(EE_Trajectory[i].size() != ee_size)
		{	
			printf("Robot Error: EE_Trajectory[%d].size() != UserDefEE.size()\n", i);
			delete[] targetTFMat_data, nowTFMat_data;
			assert(false);
		}
	}

	for(int i=0; i < ee_size; i++)
		nowTFMat_data[i] = const_cast<double*>(UserDefEE[i]->Get_EETFMat().data());

	
	// �]���u�������v�T�ϥΪ̫��wEE�ܤƪ�Robot_q���ͧ�s
	// �G����Jacobian�O�D�x�}�άO�G�x�}�A�ҥέD�x�}��inverse�ӭp��
	for(int num = 0; num < Number; num++)
	{
		for(int i=0; i < ee_size; i++)
		{
			targetTFMat_data[i] = const_cast<double*>(EE_Trajectory[num][i].data());
			Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
		}
				
		while (iter < Max_Iter && dp.norm() > err)
		{
			// �����{�b���A�� Jacobian
			Get_Jacobian(J, UserDefEE);

			ClampMag(dp, true);

			// ��Xdq
			dq = J.transpose() * ( J*J.transpose() + lambda * Eigen::MatrixXd::Identity(J_rows,J_rows) ).inverse() * dp;

			robot_q += dq;
			
			#pragma region �p��FK
			
			// ����s�D��Frame cmd
			for(int i=0; i < Active_DOF; i++)
			{	
				// ����Joint limit�ˬd
				ActiveFrame_Store[i]->Check_JointLimit(robot_q(i));
				ActiveFrame_Store[i]->Set_Command(robot_q(i));
			}
	
			// �A��s�Q��Frame cmd
			for(int i=0; i < Passive_DOF; i++)
				PassiveFrame_Store[i]->Set_Command();

			Root_DHFrame->Get_FK(Base_TFMat);
			
			#pragma endregion

			for(int i=0; i < ee_size; i++)
				Get_DiffTFMat(dp_data + 6*i, targetTFMat_data[i], nowTFMat_data[i]);
			
			iter++;
		}

		iter = 0;
		q_Trajectory.push_back(robot_q);
	}

	#pragma region �^�_��l���A Robot_q
			
	// ����s�D��Frame cmd
	for(int i=0; i < Active_DOF; i++)
	{	
		// ����Joint limit�ˬd
		// ActiveFrame_Store[i]->Check_JointLimit(Robot_q(i));
		ActiveFrame_Store[i]->Set_Command(Robot_q(i));
	}
	
	// �A��s�Q��Frame cmd
	for(int i=0; i < Passive_DOF; i++)
		PassiveFrame_Store[i]->Set_Command();

	Root_DHFrame->Get_FK(Base_TFMat);
			
	#pragma endregion

	delete[] targetTFMat_data, nowTFMat_data;
}


void Robot::Search_EE(DHFrame* Self_DHNode, std::vector<DHFrame*>& _Path)
{
	_Path.push_back(Self_DHNode);

	const std::vector<DHFrame*>& Child = Self_DHNode->Get_Child_DHNode();
	
	if(Child.size() == 0) // ���EndEffector
	{	
		EndEffector_Store.push_back(new EndEffector(Self_DHNode, _Path));
	}
	else
	{
		for(int i=0, n=Child.size(); i < n; i++)
		{
			Search_EE(Child[i], _Path);
		}
	}

	_Path.pop_back();
}

void Robot::Frame_Traversal(DHFrame* Self_DHNode)
{
	// �L�X Frame ��T
	Self_DHNode->Get_FrameInfo();
	const std::vector<DHFrame*>& Child = Self_DHNode->Get_Child_DHNode();

	for(int i=0, n=Child.size(); i < n; i++)
		Frame_Traversal(Child[i]);
}

inline void Robot::ClampMag(Eigen::VectorXd& dP, bool Is_IncludeOri)
{
	const static double Dmax = 0.1;  // ���ʶZ������
	const static double Omax = pi/4; // ��ʭ���
	static double* dP_data = NULL;

	dP_data = dP.data();

	if(Is_IncludeOri)
	{
		for(int i=0, n=dP.rows(); i < n; i+=6)
		{
			double dp_Norm = sqrt(dP_data[ i ]*dP_data[ i ] + dP_data[i+1]*dP_data[i+1] + dP_data[i+2]*dP_data[i+2]);
			double do_Norm = sqrt(dP_data[i+3]*dP_data[i+3] + dP_data[i+4]*dP_data[i+4] + dP_data[i+5]*dP_data[i+5]);

			if(dp_Norm > Dmax)
			{	
				dP_data[ i ] *= (Dmax / dp_Norm);
				dP_data[i+1] *= (Dmax / dp_Norm);
				dP_data[i+2] *= (Dmax / dp_Norm);
			}
			if(do_Norm > Omax)
			{
				dP_data[i+3] *= (Omax / do_Norm);
				dP_data[i+4] *= (Omax / do_Norm);
				dP_data[i+5] *= (Omax / do_Norm);
			}
		}
	}
	else
	{
		for(int i=0, n=dP.rows(); i < n; i+=3)
		{
			double dp_Norm = sqrt(dP_data[ i ]*dP_data[ i ] + dP_data[i+1]*dP_data[i+1] + dP_data[i+2]*dP_data[i+2]);;
			
			if(dp_Norm > Dmax)
			{
				dP_data[ i ] *= (Dmax / dp_Norm);
				dP_data[i+1] *= (Dmax / dp_Norm);
				dP_data[i+2] *= (Dmax / dp_Norm);
			}
		}
	}
}

#pragma endregion


#pragma region DynDHFrame ��@

// ���O�[�t�� (�H Base Frame ��ܡA�w�]��0) 
Eigen::Vector3d DynDHFrame::g = Eigen::Vector3d::Zero();

DynDHFrame::DynDHFrame()
	      : DHFrame()
{
	printf("DynDHFrame() Error�G��l�S�����w�~�Ӫ� DHFrame()\n");
	assert(false);
}

DynDHFrame::DynDHFrame(const DHFrame& _DHFrame, const double _Mass /*= 0*/, const double *_MC_P /*= NULL*/, const double* _Inertial /*= NULL*/)
	      : DHFrame(_DHFrame)
{
	// �Ѽƪ�l��
	cmd_d = 0; cmd_dd = 0;
	Mass = _Mass;

	// _MC_P = {MCx, MCy, MCz}
	if(_MC_P != NULL)
		MC_P << _MC_P[0], _MC_P[1], _MC_P[2];
	else
		MC_P.setZero();

	// _Inertial = {Ixx, Ixy, Ixz, Iyy, Iyz, Izz}
	if(_Inertial != NULL)
	{
		Inertial << _Inertial[0], _Inertial[1], _Inertial[2],
					_Inertial[1], _Inertial[3], _Inertial[4],
					_Inertial[2], _Inertial[4], _Inertial[5];
	}
	else
		Inertial.setZero();

	W.setZero(); Wd.setZero(); Ac.setZero(); Ae.setZero();
	Force.setZero(); Torque.setZero();
	tau = 0;

	External_Force.setZero();
	External_Torque.setZero();

	R_Parent.setZero();
	r_Parent_MC.setZero();
	r_Parent_Origin.setZero();

	if( FrameMode != CONST_FRAME )
	{	
		Forward_Recur_ptr = &DynDHFrame::Forward_Recur1;
		Backward_Recur_ptr = &DynDHFrame::Backward_Recur1;
	}
	else
	{	
		Forward_Recur_ptr = &DynDHFrame::Forward_Recur2;
		Backward_Recur_ptr = &DynDHFrame::Backward_Recur2;
	}
}

DynDHFrame::~DynDHFrame()
{
	Forward_Recur_ptr = NULL;
	Backward_Recur_ptr = NULL;
}

void DynDHFrame::Get_FrameInfo()
{
	printf("FrameInfo�G ");
	
	if(FrameMode == ACTIVE_FRAME)
	{	
		printf("Parent_ID=%d, Self_ID=%d, ACTIVE_FRAME\n", Parent_ID, Self_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, cmd=%f, cmd_d=%f, cmd_dd=%f\n", a, alpha, d, theta, cmd, cmd_d, cmd_dd);
		
		if(HaveJointLimit)
			printf("joint_min = %f, joint_max = %f\n", joint_min, joint_max);
		
		printf("Mass = %f, MC_P = [%f; %f; %f]\n", Mass, MC_P(0), MC_P(1), MC_P(2));
		printf("Inertial�G\nIxx = %f\nIxy = %f\nIxz = %f\nIyy = %f\nIyz = %f\nIzz = %f\n", Inertial(0,0), Inertial(0,1), Inertial(0,2), Inertial(1,1), Inertial(1,2), Inertial(2,2));
	}
	else if(FrameMode == PASSIVE_FRAME)
	{	
		printf("Parent_ID=%d, Self_ID=%d, DependActive_ID=%d, PASSIVE_FRAME\n", Parent_ID, Self_ID, DependActive_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, ratio=%f, cmd=%f, cmd_d=%f, cmd_dd=%f\n", a, alpha, d, theta, ratio, cmd, cmd_d, cmd_dd);
		printf("Mass = %f, MC_P = [%f; %f; %f]\n", Mass, MC_P(0), MC_P(1), MC_P(2));
		printf("Inertial�G\nIxx = %f\nIxy = %f\nIxz = %f\nIyy = %f\nIyz = %f\nIzz = %f\n", Inertial(0,0), Inertial(0,1), Inertial(0,2), Inertial(1,1), Inertial(1,2), Inertial(2,2));
	}
	else // FrameMode == CONST_FRAME
	{	
		printf("Parent_ID=%d, Self_ID=%d, CONST_FRAME\n", Parent_ID, Self_ID);	
	}
	printf("qList_Index = %d\n", qList_Index);

	printf("Child_DHNode�G");
	for(int i=0; i < Child_DHNode.size(); i++)
		printf("%d ", Child_DHNode[i]->Self_ID);
	printf("\n");

	std::cout << "TFMat = \n" << TFMat << std::endl << std::endl;
}

void DynDHFrame::Set_Mass(double New_Mass){ Mass = New_Mass; }
void DynDHFrame::Set_MC_Position(const double* New_MC_Position) 
{
	// MC_P = {MCx, MCy, MCz}
	double* MC_P_data = MC_P.data(); // Column Major
	MC_P_data[0] = New_MC_Position[0];
	MC_P_data[1] = New_MC_Position[1];
	MC_P_data[2] = New_MC_Position[2];
}
void DynDHFrame::Set_MC_Position(const Eigen::Vector3d& New_MC_Position){ MC_P = New_MC_Position; }
void DynDHFrame::Set_Inertial(const double* New_Inertial)
{
	// Inertial = {Ixx, Ixy, Ixz, Iyy, Iyz, Izz}
	double* Inertial_data = Inertial.data(); // Column Major
	Inertial_data[0] = New_Inertial[0];
	Inertial_data[1] = New_Inertial[1];
	Inertial_data[2] = New_Inertial[2];
	Inertial_data[3] = New_Inertial[1];
	Inertial_data[4] = New_Inertial[3];
	Inertial_data[5] = New_Inertial[4];
	Inertial_data[6] = New_Inertial[2];
	Inertial_data[7] = New_Inertial[4];
	Inertial_data[8] = New_Inertial[5];
}
void DynDHFrame::Set_Inertial(const Eigen::Matrix3d& New_Inertial){ Inertial = New_Inertial; }

double DynDHFrame::Get_Mass() const { return Mass; }
const Eigen::Vector3d& DynDHFrame::Get_MC_Position() const { return MC_P; }
const Eigen::Matrix3d& DynDHFrame::Get_Inertial() const { return Inertial; }

const Eigen::Vector3d& DynDHFrame::Get_W() const { return W; }           // ���t��          ���G(rad/s)
const Eigen::Vector3d& DynDHFrame::Get_Wd() const { return Wd; }         // ���[�t��        ���G(rad/s^2)
const Eigen::Vector3d& DynDHFrame::Get_Ac() const { return Ac; }         // ��ߥ[�t��      ���G(m/s^2)
const Eigen::Vector3d& DynDHFrame::Get_Ae() const { return Ae; }         // Link ���ݥ[�t�� ���G(m/s^2)
const Eigen::Vector3d& DynDHFrame::Get_Force() const { return Force; }   // �O              ���G(N)
const Eigen::Vector3d& DynDHFrame::Get_Torque() const { return Torque; } // �O�x            ���G(Nm)
double DynDHFrame::Get_tau() const { return tau; }

void DynDHFrame::Set_External_Value(const Eigen::Vector3d& Ex_Force, const Eigen::Vector3d& Ex_Torque)
{ 
	External_Force = Ex_Force;	External_Torque = Ex_Torque; 
}
void DynDHFrame::Set_External_Zero()
{
	External_Force.setZero();	External_Torque.setZero();
}
void DynDHFrame::Set_External_Force(const Eigen::Vector3d& Ex_Force)
{
	External_Force = Ex_Force;
}
void DynDHFrame::Set_External_Torque(const Eigen::Vector3d& Ex_Torque)
{
	External_Torque = Ex_Torque; 
}
void DynDHFrame::Get_External_Value(Eigen::Vector3d& Dest_Force, Eigen::Vector3d& Dest_Torque) const
{
	Dest_Force = External_Force;	Dest_Torque = External_Torque;
}
void DynDHFrame::Get_External_Force(Eigen::Vector3d& Dest_Force) const
{
	Dest_Force = External_Force;
}
const Eigen::Vector3d& DynDHFrame::Get_External_Force() const
{
	return External_Force;
}
void DynDHFrame::Get_External_Torque(Eigen::Vector3d& Dest_Torque) const
{
	Dest_Torque = External_Torque;
}
const Eigen::Vector3d& DynDHFrame::Get_External_Torque() const
{
	return External_Torque;
}


void DynDHFrame::Set_Command(double New_q, double New_qd, double New_qdd)
{
	if(FrameMode == ACTIVE_FRAME)
	{	
		cmd = New_q;
		cmd_d = New_qd;
		cmd_dd = New_qdd;
	}
}
void DynDHFrame::Set_q(double New_q)
{
	if(FrameMode == ACTIVE_FRAME)
		cmd = New_q;
}
void DynDHFrame::Set_qd(double New_qd)
{
	if(FrameMode == ACTIVE_FRAME)
		cmd_d = New_qd;
}
void DynDHFrame::Set_qdd(double New_qdd)
{
	if(FrameMode == ACTIVE_FRAME)
		cmd_dd = New_qdd;
}

void DynDHFrame::Set_Command()
{
	if(FrameMode == PASSIVE_FRAME)
	{
		cmd = ratio * ((DynDHFrame*)DependActive_DHNode)->cmd;
		cmd_d = ratio * ((DynDHFrame*)DependActive_DHNode)->cmd_d;
		cmd_dd = ratio * ((DynDHFrame*)DependActive_DHNode)->cmd_dd;
	}
}
void DynDHFrame::Set_q()
{
	if(FrameMode == PASSIVE_FRAME)
		cmd = ratio * ((DynDHFrame*)DependActive_DHNode)->cmd;
}
void DynDHFrame::Set_qd()
{
	if(FrameMode == PASSIVE_FRAME)
		cmd_d = ratio * ((DynDHFrame*)DependActive_DHNode)->cmd_d;
}
void DynDHFrame::Set_qdd()
{
	if(FrameMode == PASSIVE_FRAME)
		cmd_dd = ratio * ((DynDHFrame*)DependActive_DHNode)->cmd_dd;
}

void DynDHFrame::Get_Command(double& Dest_q, double& Dest_qd, double& Dest_qdd) const
{
	Dest_q = cmd;
	Dest_qd = cmd_d;
	Dest_qd = cmd_dd;
}

double DynDHFrame::Get_q() const { return cmd; };
double DynDHFrame::Get_qd() const { return cmd_d; };
double DynDHFrame::Get_qdd() const { return cmd_dd; };

void DynDHFrame::Get_FK_InvDyn(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae)
{
	#pragma region ��s FK
	
	// ���o�̷s�� TFMat ==> T[0, i]
	(this->*UpdateMat_ptr)(Parent_T);
	
	#pragma endregion

	// ���]���ɬ� Frame[i]�A�h Frame[i-1] ��Parent Frame
	#pragma region Forward Recursion �p��
	
	(this->*Forward_Recur_ptr)(Parent_T, Parent_W, Parent_Wd, Parent_Ae);

	int Child_Size = Child_DHNode.size();
	if(Child_Size == 0) 
	{
		// �w��FEnd-Effector�A�i�� Inverse Recursion
		if(FrameMode == CONST_FRAME)
		{
			Force = -External_Force;
			Torque = -External_Torque;
		}
		else
		{
			Force = Mass*(Ac - TFMat.block<3,3>(0,0).transpose() * DynDHFrame::g) - External_Force;
			Torque = r_Parent_MC.cross(Force) + W.cross( Inertial*W ) + Inertial*Wd - External_Torque + MC_P.cross(External_Force);
			tau = Torque.transpose() * R_Parent.col(2);
		}

		return;
	}	
	else
	{
		// �~�򭡥N
		for(int i=0; i < Child_Size; i++)
			((DynDHFrame*)Child_DHNode[i])->Get_FK_InvDyn(TFMat, W, Wd, Ae);
	}

	#pragma endregion

	#pragma region Backward Recursion �p��
	
	(this->*Backward_Recur_ptr)();
	
	#pragma endregion
}

void DynDHFrame::Get_InvDyn(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae)
{
	#pragma region ����s FK
	
	//// ���o�̷s�� TFMat ==> T[0, i]
	//(this->*UpdateMat_ptr)(Parent_T);
	
	#pragma endregion

	// ���]���ɬ� Frame[i]�A�h Frame[i-1] ��Parent Frame
	#pragma region Forward Recursion �p��
	
	(this->*Forward_Recur_ptr)(Parent_T, Parent_W, Parent_Wd, Parent_Ae);

	int Child_Size = Child_DHNode.size();
	if(Child_Size == 0) 
	{
		// �w��FEnd-Effector�A�i�� Inverse Recursion
		if(FrameMode == CONST_FRAME)
		{
			Force = -External_Force;
			Torque = -External_Torque;
		}
		else
		{
			Force = Mass*(Ac - TFMat.block<3,3>(0,0).transpose() * DynDHFrame::g) - External_Force;
			Torque = r_Parent_MC.cross(Force) + W.cross( Inertial*W ) + Inertial*Wd - External_Torque + MC_P.cross(External_Force);
			tau = Torque.transpose() * R_Parent.col(2);
		}

		return;
	}	
	else
	{
		// �~�򭡥N
		for(int i=0; i < Child_Size; i++)
			((DynDHFrame*)Child_DHNode[i])->Get_InvDyn(TFMat, W, Wd, Ae);
	}

	#pragma endregion

	#pragma region Backward Recursion �p��
	
	(this->*Backward_Recur_ptr)();
	
	#pragma endregion
}

// Active or Passive Frame �ϥ�
void DynDHFrame::Forward_Recur1(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae)
{
	// R[i, i-1] = R[0, i].transpose * R[0, i-1]
	R_Parent = TFMat.block<3,3>(0,0).transpose() * Parent_T.block<3,3>(0,0);

	// r[i-1, i] ==> Frame[i-1]���I �� Frame[i]���I �V�q(�HFrame[i]���)
	r_Parent_Origin = TFMat.block<3,3>(0,0).transpose() * (TFMat.block<3,1>(0,3) - Parent_T.block<3,1>(0,3));

	// r[i-1, MCi] ==> Frame[i-1]���I �� ���[i] �V�q = r[i-1, i] + c[i]
	r_Parent_MC = r_Parent_Origin + MC_P;

	// Forward Compute
	W = R_Parent * Parent_W + R_Parent.col(2) * cmd_d;
	Wd = R_Parent * Parent_Wd + R_Parent.col(2) * cmd_dd + W.cross( R_Parent.col(2) * cmd_d );
	Ac = R_Parent * Parent_Ae + Wd.cross(r_Parent_MC) + W.cross( W.cross(r_Parent_MC) );
	Ae = R_Parent * Parent_Ae + Wd.cross(r_Parent_Origin) + W.cross( W.cross(r_Parent_Origin) );
}

// Const Frame �ϥ�
void DynDHFrame::Forward_Recur2(const Eigen::Matrix4d& Parent_T, const Eigen::Vector3d& Parent_W, const Eigen::Vector3d& Parent_Wd, const Eigen::Vector3d& Parent_Ae)
{
	// R[i, i-1] = R[0, i].transpose * R[0, i-1]
	R_Parent = TFMat.block<3,3>(0,0).transpose() * Parent_T.block<3,3>(0,0);
	
	// r[i-1, i] ==> Frame[i-1]���I �� Frame[i]���I �V�q(�HFrame[i]���)
	r_Parent_Origin = TFMat.block<3,3>(0,0).transpose() * (TFMat.block<3,1>(0,3) - Parent_T.block<3,1>(0,3));

	// Forward Compute
	W = R_Parent * Parent_W;
	Wd = R_Parent * Parent_Wd;
	// Ac = R_Parent * Parent_Ae; // ���έp��
	Ae = R_Parent * Parent_Ae + Wd.cross(r_Parent_Origin) + W.cross( W.cross(r_Parent_Origin) );
}

// Active / Passive Frame �ϥ�
void DynDHFrame::Backward_Recur1()
{
	Force = Mass*(Ac - TFMat.block<3,3>(0,0).transpose() * DynDHFrame::g) - External_Force;
	Torque = W.cross( Inertial*W ) + Inertial*Wd - External_Torque + MC_P.cross(External_Force);

	// �N�Ҧ� Child Frame �� Force �M Troque �����֥[�_��
	for(int i=0, n=Child_DHNode.size(); i < n; i++)
	{
			DynDHFrame* Child_Node = (DynDHFrame*)Child_DHNode[i];
			
			// ���]����Frame�� Frame[i]
			// R_Child �� Child_Node->R_Parent.transpose() = R[i+1, i].transpose() = R[i, i+1]
			Eigen::Matrix3d R_Child = (Child_Node->R_Parent).transpose();

			Force += ( R_Child * Child_Node->Force );
			Torque += ( R_Child * Child_Node->Torque );
			
			if(Child_Node->FrameMode != CONST_FRAME)
				Torque += ( (R_Child * Child_Node->Force).cross(MC_P) );
			else
				Torque += ( (R_Child * Child_Node->Force).cross( TFMat.block<3,3>(0,0).transpose() * (TFMat.block<3,1>(0,3) - Child_Node->TFMat.block<3,1>(0,3)) + MC_P ) );
	}

	Torque += r_Parent_MC.cross(Force);

	tau = Torque.transpose() * R_Parent.col(2);
}

// Const Frame �ϥ�
void DynDHFrame::Backward_Recur2()
{
	Force = -External_Force;
	Torque = -External_Torque;

	// �N�Ҧ� Child Frame �� Force �M Troque �����֥[�_��
	for(int i=0, n=Child_DHNode.size(); i < n; i++)
	{
			DynDHFrame* Child_Node = (DynDHFrame*)Child_DHNode[i];
			
			// ���]����Frame�� Frame[i]
			// R_Child �� Child_Node->R_Parent.transpose() = R[i+1, i].transpose() = R[i, i+1]
			Eigen::Matrix3d R_Child = (Child_Node->R_Parent).transpose();

			Force += ( R_Child * Child_Node->Force );
			Torque += ( R_Child * Child_Node->Torque );
	}
}

#pragma endregion


#pragma region DynRobot ��@

DynRobot::DynRobot()
	    : Robot()
{

}
		
DynRobot::~DynRobot()
{
	// ����Ŷ�
	for(int i=0, n=ActiveFrame_Store.size(); i < n ; i++)
		delete ((DynDHFrame*)ActiveFrame_Store[i]);
	
	for(int i=0, n=PassiveFrame_Store.size(); i < n ; i++)
		delete ((DynDHFrame*)PassiveFrame_Store[i]);

	for(int i=0, n=ConstFrame_Store.size(); i < n ; i++)
		delete ((DynDHFrame*)ConstFrame_Store[i]);

	for(int i=0, n=EndEffector_Store.size(); i < n ; i++)
		delete EndEffector_Store[i];

	ActiveFrame_Store.clear();
	PassiveFrame_Store.clear();
	ConstFrame_Store.clear();
	EndEffector_Store.clear();
}

void DynRobot::AddActiveFrame(DynDHFrame* ActiveFrame)
{
	// �ˬdFrame Mode
	if(ActiveFrame->FrameMode != ACTIVE_FRAME)
	{	
		printf("Robot Error�GAddActiveFrame()�ϥο��~\n");
		assert(false);
	}
	
	ActiveFrame_Store.push_back(ActiveFrame);
	Active_DOF++;
}

void DynRobot::AddActiveFrame(DynDHFrame* ActiveFrame, double joint_min, double joint_max)
{
	// �ˬdFrame Mode
	if(ActiveFrame->FrameMode != ACTIVE_FRAME)
	{	
		printf("Robot Error�GAddActiveFrame()�ϥο��~\n");
		assert(false);
	}
	
	ActiveFrame->Set_JointLimit(joint_min, joint_max);
	ActiveFrame_Store.push_back(ActiveFrame);
	Active_DOF++;
}

void DynRobot::AddPassiveFrame(DynDHFrame* PassiveFrame)
{
	// �ˬdFrame Mode
	if(PassiveFrame->FrameMode != PASSIVE_FRAME)
	{	
		printf("Robot Error�GAddPassiveFrame()�ϥο��~\n");
		assert(false);
	}
	
	PassiveFrame_Store.push_back(PassiveFrame);
	Passive_DOF++;
}

void DynRobot::AddConstFrame(DynDHFrame* ConstFrame)
{
	// �ˬdFrame Mode
	if(ConstFrame->FrameMode != CONST_FRAME)
	{	
		printf("Robot Error�GAddConstFrame()�ϥο��~\n");
		assert(false);
	}
	
	ConstFrame_Store.push_back(ConstFrame);
}

bool DynRobot::Build_DynRobot()
{
	// Robot_q, Robot_qd, Robot_qdd ���Ǩ̷� ActiveFrame �� SelfID �Ѥp��j
	Robot_qd = Eigen::VectorXd::Zero(Active_DOF);
	Robot_qdd = Eigen::VectorXd::Zero(Active_DOF);

	return Robot::Build_Robot();
}

void DynRobot::Get_RobotInfo()
{
	printf("Robot Dynamic Frame Info�G\n");
	printf("*****************************************\n");
	Frame_Traversal((DynDHFrame*)Root_DHFrame);
	printf("*****************************************\n");
}

void DynRobot::Set_qd(Eigen::VectorXd& Src_qd)
{
	if(Src_qd.rows() != Active_DOF)
		return;
	
	// ����s�D��Frame cmd_d
	for(int i=0; i < Active_DOF; i++)	
		((DynDHFrame*)ActiveFrame_Store[i])->Set_qd(Src_qd(i));
		
	// �A��s�Q��Frame cmd_d
	for(int i=0; i < Passive_DOF; i++)
		((DynDHFrame*)PassiveFrame_Store[i])->Set_qd();

	Robot_qd = Src_qd;
}

void DynRobot::Set_qdd(Eigen::VectorXd& Src_qdd)
{
	if(Src_qdd.rows() != Active_DOF)
		return;
	
	// ����s�D��Frame cmd_dd
	for(int i=0; i < Active_DOF; i++)	
		((DynDHFrame*)ActiveFrame_Store[i])->Set_qdd(Src_qdd(i));
		
	// �A��s�Q��Frame cmd_dd
	for(int i=0; i < Passive_DOF; i++)
		((DynDHFrame*)PassiveFrame_Store[i])->Set_qdd();

	Robot_qdd = Src_qdd;
}

void DynRobot::Reset_qd()
{
	Robot_qd.setZero();

	// ����s�D��Frame cmd_d
	for(int i=0; i < Active_DOF; i++)	
		((DynDHFrame*)ActiveFrame_Store[i])->Set_qd(Robot_qd(i));
		
	// �A��s�Q��Frame cmd_d
	for(int i=0; i < Passive_DOF; i++)
		((DynDHFrame*)PassiveFrame_Store[i])->Set_qd();
}

void DynRobot::Reset_qdd()
{
	Robot_qdd.setZero();

	// ����s�D��Frame cmd_dd
	for(int i=0; i < Active_DOF; i++)	
		((DynDHFrame*)ActiveFrame_Store[i])->Set_qdd(Robot_qdd(i));
		
	// �A��s�Q��Frame cmd_dd
	for(int i=0; i < Passive_DOF; i++)
		((DynDHFrame*)PassiveFrame_Store[i])->Set_qdd();
}

void DynRobot::Get_qd(Eigen::VectorXd& Dest_qd) const
{
	Dest_qd = Robot_qd;
}

const Eigen::VectorXd& DynRobot::Get_qd() const
{
	return Robot_qd;
}

void DynRobot::Get_qdd(Eigen::VectorXd& Dest_qdd) const
{
	Dest_qdd = Robot_qdd;
}

const Eigen::VectorXd& DynRobot::Get_qdd() const
{
	return Robot_qdd;
}

bool DynRobot::InvDyn(Eigen::VectorXd& Dest_Torque, const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd)
{
	if( !(q.rows() == Active_DOF && qd.rows() == Active_DOF && qdd.rows() == Active_DOF) )
		return false;

	Dest_Torque = Eigen::VectorXd::Zero(Active_DOF);

	// ��s DynDHFrame List cmd, cmd_d, cmd_dd
	Robot_q = q; Robot_qd = qd; Robot_qdd = qdd;


	// ����s�D��Frame cmd, cmd_d, cmd_dd
	for(int i=0; i < Active_DOF; i++)
		((DynDHFrame*)ActiveFrame_Store[i])->Set_Command(Robot_q(i), Robot_qd(i), Robot_qdd(i));
		
	// �A��s�Q��Frame cmd, cmd_d, cmd_dd
	for(int i=0; i < Passive_DOF; i++)
		((DynDHFrame*)PassiveFrame_Store[i])->Set_Command();

	// �H���j���覡�h�P�ɧ��� FK ��s�M Dynamic Torgue �p��A�ðO���b�C�� Frame ��
	((DynDHFrame*)Root_DHFrame)->Get_FK_InvDyn(Base_TFMat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

	// ��o Torque
	for(int i=0; i < Active_DOF; i++)
		Dest_Torque(i) = ((DynDHFrame*)ActiveFrame_Store[i])->Get_tau();

	return true;
}

bool DynRobot::InvDyn(VectorXdList& Dest_Torque, const VectorXdList& q, const VectorXdList& qd, const VectorXdList& qdd)
{
	// �M�Ůe��
	Dest_Torque.clear();
	
	// �ˬd VectorXdList size
	if( !(q.size() == qd.size() && qd.size() == qdd.size()) )
		return false;

	const int Size = q.size();

	// �ˬd�C�� VectorXdList[i].rows() == Active_DOF
	for(int i=0; i < Size; i++)
	{
		if( !(q[i].rows() == Active_DOF && qd[i].rows() == Active_DOF && qdd[i].rows() == Active_DOF) )
			return false;
	}

	// �w���t�m�Ŷ�
	Dest_Torque.reserve(Size+1);

	for(int index = 0; index < Size; index++)
	{
		// ��s DynDHFrame List cmd, cmd_d, cmd_dd

		// ����s�D��Frame cmd, cmd_d, cmd_dd
		for(int i=0; i < Active_DOF; i++)
			((DynDHFrame*)ActiveFrame_Store[i])->Set_Command(q[index](i), qd[index](i), qdd[index](i));
		
		// �A��s�Q��Frame cmd, cmd_d, cmd_dd
		for(int i=0; i < Passive_DOF; i++)
			((DynDHFrame*)PassiveFrame_Store[i])->Set_Command();

		// �H���j���覡�h�P�ɧ��� FK ��s�M Dynamic Torgue �p��A�ðO���b�C�� Frame ��
		((DynDHFrame*)Root_DHFrame)->Get_FK_InvDyn(Base_TFMat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

		// ��o Torque
		Dest_Torque.push_back(Eigen::VectorXd(Active_DOF));

		for(int i=0; i < Active_DOF; i++)
			Dest_Torque[index](i) = ((DynDHFrame*)ActiveFrame_Store[i])->Get_tau();
	}

	if(Size != 0)
	{
		Robot_q = q[Size-1];
		Robot_qd = qd[Size-1];
		Robot_qdd = qdd[Size-1];
	}
}

bool DynRobot::ForDyn(const Eigen::VectorXd& Torque, const Eigen::VectorXd& Ini_q, const Eigen::VectorXd& Ini_qd)
{
	/******** �p��y�{ ********
	// Dynamic Equation : M(q)*qdd + C(q, qd)*qd + G(q) + Tau_ex = Tau(q, qd, qdd)
	// M(q)*qdd = Tau - ( C(q, qd)*qd + G(q) + Tau_ex )
	// qdd = M.inv() * [ Tau - ( C(q, qd)*qd + G(q) + Tau_ex ) ] = M.inv() * [ Tau - Tau(q, qd, 0)]

	1. ���p��[��դO,�V�ߤO,���O,�~�O]�ҳy���� Torgue �o�� Tau(q, qd, 0)
	2. �DM(q)�A�Q�� M(q)*test_qdd = M.col(i) = Tau(q, qd, test_qdd) - Tau(q, qd, 0),  test_qdd = {test_qdd(i)�Ȭ�1�A�䥦��0}
	3. �p�� qdd = M.inv() * [ Tau - Tau(q, qd, 0)]
	***************************/
	if( !(Torque.rows() == Active_DOF && Ini_q.rows() == Active_DOF && Ini_qd.rows() == Active_DOF) )
		return false;

	Eigen::VectorXd Tau_q_qd_0(Active_DOF);
	Eigen::MatrixXd M(Active_DOF, Active_DOF);
	
	double* test_qdd = new double[Active_DOF];
	for(int i=0; i < Active_DOF; i++)
		test_qdd[i] = 0;

	
	#pragma region InvDyn ==> ��oTau(q, qd, 0)

	// ��s DynDHFrame List cmd, cmd_d, cmd_dd
	Robot_q = Ini_q;
	Robot_qd = Ini_qd;

	// ����s�D��Frame cmd, cmd_d, cmd_dd
	for(int i=0; i < Active_DOF; i++)
		((DynDHFrame*)ActiveFrame_Store[i])->Set_Command(Robot_q(i), Robot_qd(i), 0);
		
	// �A��s�Q��Frame cmd, cmd_d, cmd_dd
	for(int i=0; i < Passive_DOF; i++)
		((DynDHFrame*)PassiveFrame_Store[i])->Set_Command();

	// �H���j���覡�h�P�ɧ��� FK ��s�M Dynamic Torgue �p��A�ðO���b�C�� Frame ��
	((DynDHFrame*)Root_DHFrame)->Get_FK_InvDyn(Base_TFMat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

	// ��o Torque
	for(int i=0; i < Active_DOF; i++)
		Tau_q_qd_0(i) = ((DynDHFrame*)ActiveFrame_Store[i])->Get_tau();

	#pragma endregion

	#pragma region ��oM(q)

	for(int index=0; index < Active_DOF; index++)
	{
		if(index == 0)
			test_qdd[index] = 1;
		else
		{
			test_qdd[index-1] = 0;
			test_qdd[index] = 1;
		}

		// ����s�D��Frame cmd_dd (���Gcmd, cmd_d ����)
		for(int i=0; i < Active_DOF; i++)
			((DynDHFrame*)ActiveFrame_Store[i])->Set_qdd( test_qdd[i] );
		
		// �A��s�Q��Frame cmd_dd (���Gcmd, cmd_d ����)
		for(int i=0; i < Passive_DOF; i++)
			((DynDHFrame*)PassiveFrame_Store[i])->Set_qdd();

		// �H���j���覡�h���� Dynamic Torgue �p��A�ðO���b�C�� Frame ��
		((DynDHFrame*)Root_DHFrame)->Get_InvDyn(Base_TFMat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

		// ��o M.col(index)
		for(int i=0; i < Active_DOF; i++)
			M(i, index) = ((DynDHFrame*)ActiveFrame_Store[i])->Get_tau() - Tau_q_qd_0(i);
	}

	#pragma endregion

	#pragma region �p�� Robot_qdd

	Robot_qdd = M.inverse() * (Torque - Tau_q_qd_0);
	
	#pragma endregion

	delete[] test_qdd;

	return true;
}

bool DynRobot::ForDyn(VectorXdList& Dest_q, VectorXdList& Dest_qd, VectorXdList& Dest_qdd, const VectorXdList& Torque, const Eigen::VectorXd& Ini_q, const Eigen::VectorXd& Ini_qd, const double TimeInterval)
{
	/******** �p��y�{ ********
	// Dynamic Equation : M(q)*qdd + C(q, qd)*qd + G(q) + Tau_ex = Tau(q, qd, qdd)
	// M(q)*qdd = Tau - ( C(q, qd)*qd + G(q) + Tau_ex )
	// qdd = M.inv() * [ Tau - ( C(q, qd)*qd + G(q) + Tau_ex ) ] = M.inv() * [ Tau - Tau(q, qd, 0)]

	1. ���p��[��դO,�V�ߤO,���O,�~�O]�ҳy���� Torgue �o�� Tau(q, qd, 0)
	2. �DM(q)�A�Q�� M(q)*test_qdd = M.col(i) = Tau(q, qd, test_qdd) - Tau(q, qd, 0),  test_qdd = {test_qdd(i)�Ȭ�1�A�䥦��0}
	3. �p�� qdd = M.inv() * [ Tau - Tau(q, qd, 0)]
	4. �p��U�@�誺q(t+T), qd(t+T) ==> 
	   qd(t+T) = qd(t) + T * qdd(t)
	   q(t+T) = q(t) + T * qd(t) + 0.5*(T^2) * qdd(t)
	5. ���ƥH�W�B�J���쵲��
	***************************/

	// ���G�̧ǱN [q qd qdd] �H�@�� Matrix �x�s�A�C�@�ɨ誺 Matrix �A�x�s�� q_traj 
	const int Size = Torque.size();

	// �M�Ůe��
	Dest_q.clear(); Dest_qd.clear(); Dest_qdd.clear();

	// �ˬd
	if( !(Size != 0 && Ini_q.rows() == Active_DOF && Ini_qd.rows() == Active_DOF) )
		return false;
	for(int i=0; i < Size; i++)
	{
		if(Torque[i].rows() != Active_DOF)
			return false;
	}

	// �w���t�m�Ŷ�
	Dest_q.reserve(Size+1);    Dest_qd.reserve(Size+1);    Dest_qdd.reserve(Size+1);

	Eigen::VectorXd Tau_q_qd_0(Active_DOF);
	Eigen::MatrixXd M(Active_DOF, Active_DOF);
	
	double* test_qdd = new double[Active_DOF];
	for(int i=0; i < Active_DOF; i++)
		test_qdd[i] = 0;

	// �}�l�i�歡�N�B��
	for(int Num = 0; Num < Size; Num++)
	{
		if(Num == 0)
		{
			Dest_q.push_back(Ini_q);    Dest_qd.push_back(Ini_qd);
		}
		else
		{
			Dest_q.push_back(Eigen::VectorXd(Active_DOF));    Dest_qd.push_back(Eigen::VectorXd(Active_DOF));
			
			// qd(t+T) = qd(t) + T * qdd(t)
			// q(t+T) = q(t) + T * qd(t) + 0.5*(T^2) * qdd(t)
			Dest_qd[Num] = Dest_qd[Num-1] + TimeInterval * Dest_qdd[Num-1];
			Dest_q[Num]  = Dest_q[Num-1] + TimeInterval * Dest_qd[Num-1] +  (0.5*TimeInterval*TimeInterval) * Dest_qdd[Num-1];
		}

		#pragma region InvDyn ==> ��oTau(q, qd, 0)

		// ��s DynDHFrame List cmd, cmd_d, cmd_dd

		// ����s�D��Frame cmd, cmd_d, cmd_dd
		for(int i=0; i < Active_DOF; i++)
			((DynDHFrame*)ActiveFrame_Store[i])->Set_Command(Dest_q[Num](i), Dest_qd[Num](i), 0);
		
		// �A��s�Q��Frame cmd, cmd_d, cmd_dd
		for(int i=0; i < Passive_DOF; i++)
			((DynDHFrame*)PassiveFrame_Store[i])->Set_Command();

		// �H���j���覡�h�P�ɧ��� FK ��s�M Dynamic Torgue �p��A�ðO���b�C�� Frame ��
		((DynDHFrame*)Root_DHFrame)->Get_FK_InvDyn(Base_TFMat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

		// ��o Torque
		for(int i=0; i < Active_DOF; i++)
			Tau_q_qd_0(i) = ((DynDHFrame*)ActiveFrame_Store[i])->Get_tau();

		#pragma endregion

		#pragma region ��oM(q)

		for(int index=0; index < Active_DOF; index++)
		{
			if(index == 0)
				test_qdd[index] = 1;
			else
			{
				test_qdd[index-1] = 0;
				test_qdd[index] = 1;
			}

			// ����s�D��Frame cmd_dd (���Gcmd, cmd_d ����)
			for(int i=0; i < Active_DOF; i++)
				((DynDHFrame*)ActiveFrame_Store[i])->Set_qdd( test_qdd[i] );
		
			// �A��s�Q��Frame cmd_dd (���Gcmd, cmd_d ����)
			for(int i=0; i < Passive_DOF; i++)
				((DynDHFrame*)PassiveFrame_Store[i])->Set_qdd();

			// �H���j���覡�h���� Dynamic Torgue �p��A�ðO���b�C�� Frame ��
			((DynDHFrame*)Root_DHFrame)->Get_InvDyn(Base_TFMat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

			// ��o M.col(index)
			for(int i=0; i < Active_DOF; i++)
				M(i, index) = ((DynDHFrame*)ActiveFrame_Store[i])->Get_tau() - Tau_q_qd_0(i);
		}

		test_qdd[Active_DOF-1] = 0;

		#pragma endregion

		#pragma region �p�� Robot_qdd

		Dest_qdd.push_back(Eigen::VectorXd(Active_DOF));
		Dest_qdd[Num] = M.inverse() * (Torque[Num] - Tau_q_qd_0);
	
		#pragma endregion
	}

	Robot_q = Dest_q[Size-1];
	Robot_qd = Dest_qd[Size-1];
	Robot_qdd = Dest_qdd[Size-1];
	delete[] test_qdd;

	return true;
}

void DynRobot::Frame_Traversal(DynDHFrame* Self_DHNode)
{
	// �L�X Frame ��T
	Self_DHNode->Get_FrameInfo();
	const std::vector<DHFrame*>& Child = Self_DHNode->Get_Child_DHNode();

	for(int i=0, n=Child.size(); i < n; i++)
		Frame_Traversal((DynDHFrame*)Child[i]);
}

#pragma endregion


#pragma region Robot �����ƾǹB��禡

/*******************************
�إ߼зǪ� DH Model�A�Ѽƻ����G
[   a   ] �� Z(i-1)�PZ(i)���@���u�Z��
[ alpha ] ��Z(i-1)�u��X(i)��V�����Z(i)������
[   d   ] ��frame(i-1)�����I��@���u�PZ(i-1)���I���Z��(�u��Z(i-1)��V����)
[ theta ] ��X(i-1)�u��Z(i-1)��V�����X(i)������

T = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),   a*cos(theta),]
    [ sin(theta),  cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),  a*sin(theta),]
    [ 0,           sin(alpha),             cos(alpha),              d,			 ]
    [ 0,           0,                      0,                       1            ]
*******************************/
Eigen::Matrix4d& Rbt::DH2TFMat(double a, double alpha, double d, double theta)
{
	static Eigen::Matrix4d T;
	static double* T_data = T.data(); // Column Major

	T_data[0] = cos(theta);
	T_data[1] = sin(theta);
	T_data[2] = 0;
	T_data[3] = 0;

	T_data[4] = -sin(theta)*cos(alpha);
	T_data[5] = cos(theta)*cos(alpha);
	T_data[6] = sin(alpha);
	T_data[7] = 0;

	T_data[8] = sin(theta)*sin(alpha);
	T_data[9] = -cos(theta)*sin(alpha);
	T_data[10] = cos(alpha);
	T_data[11] = 0;

	T_data[12] = a*cos(theta);
	T_data[13] = a*sin(theta);
	T_data[14] = d;
	T_data[15] = 1;

	return T;
}

void Rbt::DH2TFMat(Eigen::Matrix4d& Dest_T, double a, double alpha, double d, double theta)
{
	double* T_data = Dest_T.data(); // Column Major

	T_data[0] = cos(theta);
	T_data[1] = sin(theta);
	T_data[2] = 0;
	T_data[3] = 0;

	T_data[4] = -sin(theta)*cos(alpha);
	T_data[5] = cos(theta)*cos(alpha);
	T_data[6] = sin(alpha);
	T_data[7] = 0;

	T_data[8] = sin(theta)*sin(alpha);
	T_data[9] = -cos(theta)*sin(alpha);
	T_data[10] = cos(alpha);
	T_data[11] = 0;

	T_data[12] = a*cos(theta);
	T_data[13] = a*sin(theta);
	T_data[14] = d;
	T_data[15] = 1;
}

Eigen::Vector3d& Rbt::Get_Orientation(const Eigen::Matrix3d& Src_R)
{
	static Eigen::Vector3d k; // [kx,ky,kz]

	Get_Orientation(k.data(), Src_R.data());

	return k;
};

void Rbt::Get_Orientation(Eigen::Vector3d& Dest_Ori, const Eigen::Matrix3d& Src_R)
{
	Get_Orientation(Dest_Ori.data(), Src_R.data());
}

void Rbt::Get_Orientation(double* Dest_Ori, const double* Src_R) 
{
	/*****************************
	Column Major
	>> Dest_Ori array size: 3
	>> Src_R    array size: 9

	        [0]       [0 3 6]
	   Ori: [1]    R: [1 4 7]
	        [2]       [2 5 8]
	*****************************/

	double theta = acos( (Src_R[0]+Src_R[4]+Src_R[8]-1)/2.0 );

	// theta = 0 (deg) ==> singular
	if(abs(theta) <= 1e-8)
	{
		//Dest_Ori << 0, 0, 0;
		Dest_Ori[0] = 0;
		Dest_Ori[1] = 0;
		Dest_Ori[2] = 0;
		return;
	}
	else if(abs(theta-pi) <= 1e-8) // theta = 180 (deg) ==> singular
	{
		// k*k.transpose() = (Src_R + Eigen::Matrix3d::Identity()) / 2;
		double temp[9] = {(Src_R[0]+1.0)/2.0, Src_R[1]/2.0, Src_R[2]/2.0, Src_R[3]/2.0, (Src_R[4]+1.0)/2.0, Src_R[5]/2.0, Src_R[6]/2.0, Src_R[7]/2.0, (Src_R[8]+1.0)/2.0};

		// �O kx >= 0
		Dest_Ori[0] = sqrt(temp[0]);

		if(Dest_Ori[0] == 0)
		{
			if(temp[5] > 0) // ky*kz > 0
			{
				Dest_Ori[1] = sqrt(temp[4]); //ky
				Dest_Ori[2] = sqrt(temp[8]); //kz
			}
			else
			{
				Dest_Ori[1] = sqrt(temp[4]); //ky
				Dest_Ori[2] = -sqrt(temp[8]); //kz
			}
		}
		else  // kx > 0
		{
			if(temp[1] > 0) // kx*ky > 0
				Dest_Ori[1] = sqrt(temp[4]); //ky
			else
				Dest_Ori[1] = -sqrt(temp[4]); //ky


			if(temp[2] > 0) // kx*kz > 0
				Dest_Ori[2] = sqrt(temp[8]); //kz
			else
				Dest_Ori[2] = -sqrt(temp[8]); //kz
		}
	}
	else
	{
		// Dest_Ori << Src_R(2,1)-Src_R(1,2), Src_R(0,2)-Src_R(2,0), Src_R(1,0)-Src_R(0,1);
		// Dest_Ori = Dest_Ori / (2*sin(theta));
		double Den = 2*sin(theta);
		Dest_Ori[0] = (Src_R[5]-Src_R[7]) / Den;
		Dest_Ori[1] = (Src_R[6]-Src_R[2]) / Den;
		Dest_Ori[2] = (Src_R[1]-Src_R[3]) / Den;
	}

	Dest_Ori[0] *= theta;
	Dest_Ori[1] *= theta;
	Dest_Ori[2] *= theta;
}

void Rbt::Get_PosOri(Eigen::VectorXd& Dest_P, const Eigen::Matrix4d& Src_T)
{
	if(Dest_P.rows() != 6)
		Dest_P.resize(6);

	// Dest_P << Src_T.block<3,1>(0,3), Get_Orientation(Src_T.block<3,3>(0,0));
	Get_PosOri(Dest_P.data(), Src_T.data());
}

void Rbt::Get_PosOri(double* Dest_P, const double* Src_T)// Column Major�A�����T�O Array size ���T
{
	/*****************************
	Column Major
	>> Dest_P array size: 6
	>> Src_T  array size: 16
	      [0]   [Px]
	      [1]   [Py]       [0 4  8 12]   [     |   ]
	   P: [2] = [Pz]    T: [1 5  9 13] = [  R  | P ]
          [3]   [Ox]       [2 6 10 14]   [-----|---]
   	      [4]   [Oy]       [3 7 11 15]   [  0  | 1 ]
		  [5]   [Oz]
	*****************************/
	
	#pragma region Position
	
	Dest_P[0] = Src_T[12];
	Dest_P[1] = Src_T[13];
	Dest_P[2] = Src_T[14];
	
	#pragma endregion

	#pragma region Orientation

	double theta = acos( (Src_T[0]+Src_T[5]+Src_T[10]-1)/2.0 );

	// theta = 0 (deg) ==> singular
	if(abs(theta) <= 1e-8)
	{
		Dest_P[3] = 0;
		Dest_P[4] = 0;
		Dest_P[5] = 0;
		return;
	}
	else if(abs(theta-pi) <= 1e-8) // theta = 180 (deg) ==> singular
	{
		// k*k.transpose() = (Src_R + Eigen::Matrix3d::Identity()) / 2;
		double temp[9] = {(Src_T[0]+1.0)/2.0, Src_T[1]/2.0, Src_T[2]/2.0, Src_T[4]/2.0, (Src_T[5]+1.0)/2.0, Src_T[6]/2.0, Src_T[8]/2.0, Src_T[9]/2.0, (Src_T[10]+1.0)/2.0};

		// �O kx >= 0
		Dest_P[3] = sqrt(temp[0]);

		if(Dest_P[3] == 0)
		{
			if(temp[5] > 0) // ky*kz > 0
			{
				Dest_P[4] = sqrt(temp[4]); //ky
				Dest_P[5] = sqrt(temp[8]); //kz
			}
			else
			{
				Dest_P[4] = sqrt(temp[4]); //ky
				Dest_P[5] = -sqrt(temp[8]); //kz
			}
		}
		else  // kx > 0
		{
			if(temp[1] > 0) // kx*ky > 0
				Dest_P[4] = sqrt(temp[4]); //ky
			else
				Dest_P[4] = -sqrt(temp[4]); //ky


			if(temp[2] > 0) // kx*kz > 0
				Dest_P[5] = sqrt(temp[8]); //kz
			else
				Dest_P[5] = -sqrt(temp[8]); //kz
		}
	}
	else
	{
		// Dest_Ori << Src_R(2,1)-Src_R(1,2), Src_R(0,2)-Src_R(2,0), Src_R(1,0)-Src_R(0,1);
		// Dest_Ori = Dest_Ori / (2*sin(theta));
		double Den = 2*sin(theta);
		Dest_P[3] = (Src_T[6]-Src_T[9]) / Den;
		Dest_P[4] = (Src_T[8]-Src_T[2]) / Den;
		Dest_P[5] = (Src_T[1]-Src_T[4]) / Den;
	}

	Dest_P[3] *= theta;
	Dest_P[4] *= theta;
	Dest_P[5] *= theta;

	#pragma endregion

}

void Rbt::Get_TFMatrix(Eigen::Matrix4d& Dest_T, const Eigen::VectorXd& Src_P)
{
	if(Src_P.rows() != 6)
		return;

	Get_TFMatrix(Dest_T.data(), Src_P.data());
}

void Rbt::Get_TFMatrix(double* Dest_T, const double* Src_P)
{
	/*****************************
	Column Major
	>> Dest_T array size: 16
	>> Src_P  array size: 6

	                                      [0]   [Px]
	      [0 4  8 12]   [     |   ]       [1]   [Py]       
	   T: [1 5  9 13] = [  R  | P ]    P: [2] = [Pz]
		  [2 6 10 14]   [-----|---]       [3]   [Ox]
		  [3 7 11 15]   [  0  | 1 ]       [4]   [Oy]
		                                  [5]   [Oz]
	*****************************/

	#pragma region Position
	
	Dest_T[12] = Src_P[0];
	Dest_T[13] = Src_P[1];
	Dest_T[14] = Src_P[2];
	
	Dest_T[3] = 0; Dest_T[7] = 0; Dest_T[11] = 0; Dest_T[15] = 1;

	#pragma endregion

	#pragma region Orientation

	double theta = sqrt(Src_P[3]*Src_P[3] + Src_P[4]*Src_P[4] + Src_P[5]*Src_P[5]);
	
	if(abs(theta) <= 1e-8)
	{
		Dest_T[0] = 1;
		Dest_T[1] = 0;
		Dest_T[2] = 0;
		
		Dest_T[4] = 0;
		Dest_T[5] = 1;
		Dest_T[6] = 0;

		Dest_T[8] = 0;
		Dest_T[9] = 0;
		Dest_T[10] = 1;
	}
	else
	{	
		// �ܳ��V�q
		double k[3] = {Src_P[3]/theta, Src_P[4]/theta, Src_P[5]/theta};
		double Cos = cos(theta);
		double Sin = sin(theta);

		//Eigen::Matrix3d R = cos(theta)*Eigen::Matrix3d::Identity() + (1-cos(theta))*k*k.transpose() + sin(theta)*k_cross;
		Dest_T[0] = Cos + (1-Cos)*k[0]*k[0];
		Dest_T[1] = (1-Cos)*k[0]*k[1] + Sin*k[2];
		Dest_T[2] = (1-Cos)*k[0]*k[2] - Sin*k[1];
		
		Dest_T[4] = (1-Cos)*k[0]*k[1] - Sin*k[2];
		Dest_T[5] = Cos + (1-Cos)*k[1]*k[1];
		Dest_T[6] = (1-Cos)*k[1]*k[2] + Sin*k[0];

		Dest_T[8] = (1-Cos)*k[0]*k[2] + Sin*k[1];
		Dest_T[9] = (1-Cos)*k[1]*k[2] - Sin*k[0];
		Dest_T[10] = Cos + (1-Cos)*k[2]*k[2];
	}

	#pragma endregion
}

void Rbt::Get_RMatrix(Eigen::Matrix3d& Dest_R, const Eigen::Vector3d& Src_O)
{
	Get_RMatrix(Dest_R.data(), Src_O.data());
}

void Rbt::Get_RMatrix(double* Dest_R, const double* Src_O)
{
	/*****************************
	Column Major
	>> Dest_R array size: 9
	>> Src_O  array size: 3

	      [0 3 6]         [0]
	   R: [1 4 7]    Ori: [1]
	      [2 5 8]         [2]
	*****************************/

	double theta = sqrt(Src_O[0]*Src_O[0] + Src_O[1]*Src_O[1] + Src_O[2]*Src_O[2]);
	
	if(abs(theta) <= 1e-8)
	{
		Dest_R[0] = 1;
		Dest_R[1] = 0;
		Dest_R[2] = 0;
		
		Dest_R[3] = 0;
		Dest_R[4] = 1;
		Dest_R[5] = 0;

		Dest_R[6] = 0;
		Dest_R[7] = 0;
		Dest_R[8] = 1;
	}
	else
	{	
		// �ܳ��V�q
		double k[3] = {Src_O[0]/theta, Src_O[1]/theta, Src_O[2]/theta};
		double Cos = cos(theta);
		double Sin = sin(theta);

		//Eigen::Matrix3d R = cos(theta)*Eigen::Matrix3d::Identity() + (1-cos(theta))*k*k.transpose() + sin(theta)*k_cross;
		Dest_R[0] = Cos + (1-Cos)*k[0]*k[0];
		Dest_R[1] = (1-Cos)*k[0]*k[1] + Sin*k[2];
		Dest_R[2] = (1-Cos)*k[0]*k[2] - Sin*k[1];
		
		Dest_R[3] = (1-Cos)*k[0]*k[1] - Sin*k[2];
		Dest_R[4] = Cos + (1-Cos)*k[1]*k[1];
		Dest_R[5] = (1-Cos)*k[1]*k[2] + Sin*k[0];

		Dest_R[6] = (1-Cos)*k[0]*k[2] + Sin*k[1];
		Dest_R[7] = (1-Cos)*k[1]*k[2] - Sin*k[0];
		Dest_R[8] = Cos + (1-Cos)*k[2]*k[2];
	}
}

void Rbt::Get_DiffTFMat(double* Dest_dp, const double* T_target, const double* T_ref)
{
	/*****************************
	Column Major
	>> Dest_dp         array size: 6
	>> T_target, T_ref array size: 16
	       [0]   [ dx]
	       [1]   [ dy]       [0 4  8 12]   [     |   ]
	   dp: [2] = [ dz]    T: [1 5  9 13] = [  R  | P ]
           [3]   [dRx]       [2 6 10 14]   [-----|---]
   	       [4]   [dRy]       [3 7 11 15]   [  0  | 1 ]
		   [5]   [dRz]
								 [dx]
	   dp = Pnew - Pold  ==Get=> [dy]
								 [dz]
	--------------------------------------------------
	   Rd = Skew(w)*R ==> dR = Rnew-Rold = Skew(dR)*Rold ==> (S(dR)+I) = Rnew * Rold.transpose();
	   
	   [  1 -dRz dRy ]								      [dRx]
	   [ dRz  1 -dRx ] = Rnew * Rold.transpose()  ==Get=> [dRy]
	   [-dRy dRx  1  ]								      [dRz]
	*****************************/
	
	// Diff_Pos = P_target - P_ref
	Dest_dp[0] = T_target[12] - T_ref[12]; // dx
	Dest_dp[1] = T_target[13] - T_ref[13]; // dy
	Dest_dp[2] = T_target[14] - T_ref[14]; // dz

	// Diff_R = R_target * R_ref.transpose()
	Dest_dp[3] = T_target[2]*T_ref[1] + T_target[6]*T_ref[5] + T_target[10]*T_ref[9]; // dRx
	Dest_dp[4] = T_target[0]*T_ref[2] + T_target[4]*T_ref[6] + T_target[8]*T_ref[10]; // dRy
	Dest_dp[5] = T_target[1]*T_ref[0] + T_target[5]*T_ref[4] + T_target[9]*T_ref[8];  // dRz
}

void Rbt::Get_RotX(double* Dest_R, double Theta)
{
	/*****************************
	Column Major
	>> Dest_R array size: 9

	      [0 3 6]   [1  0  0]
	   R: [1 4 7] = [0  C -S]
	      [2 5 8]   [0  S  C]
	*****************************/
	Dest_R[0] = 1.0;
	Dest_R[1] = 0.0;
	Dest_R[2] = 0.0;
	Dest_R[3] = 0.0;
	Dest_R[4] = cos(Theta);
	Dest_R[5] = sin(Theta);
	Dest_R[6] = 0.0;
	Dest_R[7] = -sin(Theta);
	Dest_R[8] = cos(Theta);
}
void Rbt::Get_RotY(double* Dest_R, double Theta) // Column Major�A�����T�O Array size ���T
{
	/*****************************
	Column Major
	>> Dest_R array size: 9

	      [0 3 6]   [C  0  S]
	   R: [1 4 7] = [0  1  0]
	      [2 5 8]   [-S 0  C]
	*****************************/
	Dest_R[0] = cos(Theta);
	Dest_R[1] = 0.0;
	Dest_R[2] = -sin(Theta);
	Dest_R[3] = 0.0;
	Dest_R[4] = 1.0;
	Dest_R[5] = 0.0;
	Dest_R[6] = sin(Theta);
	Dest_R[7] = 0.0;
	Dest_R[8] = cos(Theta);
}
void Rbt::Get_RotZ(double* Dest_R, double Theta) // Column Major�A�����T�O Array size ���T
{
	/*****************************
	Column Major
	>> Dest_R array size: 9

	      [0 3 6]   [C -S  0]
	   R: [1 4 7] = [S  C  0]
	      [2 5 8]   [0  0  1]
	*****************************/
	Dest_R[0] = cos(Theta);
	Dest_R[1] = sin(Theta);
	Dest_R[2] = 0.0;
	Dest_R[3] = -sin(Theta);
	Dest_R[4] = cos(Theta);
	Dest_R[5] = 0.0;
	Dest_R[6] = 0.0;
	Dest_R[7] = 0.0;
	Dest_R[8] = 1.0;
}

#pragma endregion