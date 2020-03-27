//#include "stdafx.h"
#include "NTU_ArmHand_Parameter.h"

using namespace Rbt;

void NTU_Arm::Build_KArm(Rbt::Robot& robot)
{
	// 黑金剛手臂
	for(int i=0; i < 6; i++)
		robot.AddActiveFrame(new DHFrame(i, i+1, KArm_a[i], KArm_alpha[i], KArm_d[i], KArm_theta[i]), KArm_JointMin[i], KArm_JointMax[i]);
	
	robot.Set_DLS_Parameter(1, 0.00175, 2, 0.00175);
	robot.Build_Robot();
}

void NTU_Arm::Build_KArm_Dyn(Rbt::DynRobot& robot)
{
	// 黑金剛手臂
	for(int i=0; i < 6; i++)
		robot.AddActiveFrame(new DynDHFrame(DHFrame(i, i+1, KArm_a[i], KArm_alpha[i], KArm_d[i], KArm_theta[i]), KArm_M[i], KArm_MC[i], KArm_Inertial[i]), KArm_JointMin[i], KArm_JointMax[i]);
	
	robot.Set_DLS_Parameter(1, 0.00175, 2, 0.00175);
	robot.Build_Robot();

	// 設定重力方向
	Rbt::DynDHFrame::g = Eigen::Vector3d(0, 0, -1) * Gravity;
}

void NTU_Arm::Build_8AxisArm(Rbt::Robot& robot)
{
	// 八軸手臂
	for(int i=0; i < 8; i++)
		robot.AddActiveFrame(new DHFrame(i, i+1, _8AxisArm_a[i], _8AxisArm_alpha[i], _8AxisArm_d[i], _8AxisArm_theta[i]), _8AxisArm_JointMin[i], _8AxisArm_JointMax[i]);
	
	robot.Set_DLS_Parameter(1, 1e-5, 2, 1e-5);

	// 八軸擺放的位置
	Eigen::Matrix4d BaseTFMat;
	BaseTFMat << -1,  0,  0,  1.5,
				  0, -1,  0,  0,
				  0,  0,  1,  0.1,
				  0,  0,  0,  1;
	robot.Set_BaseTFMat(BaseTFMat);

	robot.Build_Robot();
}

void NTU_Arm::Build_8AxisArm_Dyn(Rbt::DynRobot& robot)
{
	// 八軸手臂
	for(int i=0; i < 8; i++)
		robot.AddActiveFrame(new DynDHFrame(DHFrame(i, i+1, _8AxisArm_a[i], _8AxisArm_alpha[i], _8AxisArm_d[i], _8AxisArm_theta[i]), _8AxisArm_M[i], _8AxisArm_MC[i], _8AxisArm_Inertial[i]), 
							 _8AxisArm_JointMin[i], _8AxisArm_JointMax[i]);
	
	robot.Set_DLS_Parameter(1, 1e-5, 2, 1e-5);
	robot.Build_Robot();

	// 設定重力方向
	Rbt::DynDHFrame::g = Eigen::Vector3d(0, 0, -1) * Gravity;
}

void NTU_ArmHand::Build_ArmHand(Rbt::Robot& robot)
{
	#pragma region 黑金剛手臂

	for(int i=0; i < 5; i++)
		robot.AddActiveFrame(new DHFrame(i, i+1, NTU_Arm::KArm_a[i], NTU_Arm::KArm_alpha[i], NTU_Arm::KArm_d[i], NTU_Arm::KArm_theta[i]), NTU_Arm::KArm_JointMin[i], NTU_Arm::KArm_JointMax[i]);
	
	// Frame 6 移至新手掌中心 ==> 新黑金剛EE
	robot.AddActiveFrame(new DHFrame(5, 6, NTU_Arm::KArm_a[5], NTU_Arm::KArm_alpha[5], NTU_Arm::KArm_d[5], NTU_Arm::KArm_theta[5], NULL, &DH2TFMat(-0.00118, 0, -0.026, 0)), Deg2Rad(-78), Deg2Rad(78)); // Joint 6 如果使用手臂原本基板 joint limit = -85 ~ 85 (deg)
																																															             // Joint 6 如果使用手掌紅色基板 joint limit = -78 ~ 78 (deg)
	#pragma endregion

	#pragma region 新手掌

	Eigen::Matrix4d Pseudo_T1, Pseudo_T2;
	Pseudo_T1 << 0, 0, 1, -0.10382,
				 0, 1, 0, 0,
				-1, 0, 0, 0.026,
				 0, 0, 0, 1;

	// 大拇指
	Pseudo_T2 << sin(Deg2Rad(20)), cos(Deg2Rad(20)),   0,  0.02626,
				 cos(Deg2Rad(20)), -sin(Deg2Rad(20)),  0,  0.04793,
				 0,				   0,				  -1,  0.01762,
				 0,				   0,				   0,  1;
	
	
	robot.AddConstFrame  (new DHFrame(6, 7, Pseudo_T1 * Pseudo_T2));
	robot.AddActiveFrame (new DHFrame(7, 8, 0.02904, Deg2Rad(90), -0.05932, Deg2Rad(45)), Deg2Rad(0), Deg2Rad(125));
	robot.AddActiveFrame (new DHFrame(8, 9, 0, Deg2Rad(-90), 0.04573, Deg2Rad(-90)), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DHFrame(9, 10, 0.0532, 0, 0, 0), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DHFrame(10, 11, 0.0215, 0, 0, Deg2Rad(-6.68)), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DHFrame(11, 12, 11, 0.0218, 0, 0, Deg2Rad(6.68), 1));

	
	// 食指
	Pseudo_T2 << sin(Deg2Rad(20)), 0,	-cos(Deg2Rad(20)),	0.011788,
				 cos(Deg2Rad(20)), 0,	sin(Deg2Rad(20)),	0.05202,
				 0,				  -1,	     0,				0.11494,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DHFrame(6, 13, Pseudo_T1 * Pseudo_T2));
	robot.AddActiveFrame (new DHFrame(13, 14, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DHFrame(14, 15, 0.0532, 0, 0, 0), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DHFrame(15, 16, 0.0215, 0, 0, Deg2Rad(-6.68)), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DHFrame(16, 17, 16, 0.0218, 0, 0, Deg2Rad(6.68), 1));

	
	// 中指
	Pseudo_T2 << sin(Deg2Rad(10)), 0,	-cos(Deg2Rad(10)),	0.00297,
				 cos(Deg2Rad(10)), 0,	sin(Deg2Rad(10)),	0.019114,
				 0,				  -1,	     0,				0.12494,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DHFrame(6, 18, Pseudo_T1 * Pseudo_T2));
	robot.AddActiveFrame (new DHFrame(18, 19, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DHFrame(19, 20, 0.0532, 0, 0, 0), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DHFrame(20, 21, 0.0215, 0, 0, Deg2Rad(-6.68)), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DHFrame(21, 22, 21, 0.0218, 0, 0, Deg2Rad(6.68), 1));


	// 無名指
	Pseudo_T2 << -sin(Deg2Rad(2)), 0,	-cos(Deg2Rad(2)),	0.000105,
				 cos(Deg2Rad(2)),  0,	-sin(Deg2Rad(2)),	-0.01483,
				 0,				  -1,	     0,				0.11934,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DHFrame(6, 23, Pseudo_T1 * Pseudo_T2));
	robot.AddActiveFrame (new DHFrame(23, 24, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DHFrame(24, 25, 0.0532, 0, 0, 0), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DHFrame(25, 26, 0.0215, 0, 0, Deg2Rad(-6.68)), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DHFrame(26, 27, 26, 0.0218, 0, 0, Deg2Rad(6.68), 1));


	// 小拇指
	Pseudo_T2 << -sin(Deg2Rad(10)), 0,	-cos(Deg2Rad(10)),	0.00296,
				 cos(Deg2Rad(10)),  0,	-sin(Deg2Rad(10)),	-0.04882,
				 0,				  -1,	     0,				0.09994,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DHFrame(6, 28, Pseudo_T1 * Pseudo_T2));
	robot.AddActiveFrame (new DHFrame(28, 29, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DHFrame(29, 30, 0.0532, 0, 0, 0), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DHFrame(30, 31, 0.0215, 0, 0, Deg2Rad(-6.68)), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DHFrame(31, 32, 31, 0.0218, 0, 0, Deg2Rad(6.68), 1));

	#pragma endregion

	robot.Set_DLS_Parameter(1, 0.00175, 5, 0.005);
	robot.Build_Robot();
}

void NTU_ArmHand::Build_ArmHand_Dyn(Rbt::DynRobot& robot)
{
	#pragma region 黑金剛手臂

	for(int i=0; i < 5; i++)
		robot.AddActiveFrame(new DynDHFrame(DHFrame(i, i+1, NTU_Arm::KArm_a[i], NTU_Arm::KArm_alpha[i], NTU_Arm::KArm_d[i], NTU_Arm::KArm_theta[i]), ArmHand_M[i], ArmHand_MC[i], ArmHand_Inertial[i]), NTU_Arm::KArm_JointMin[i], NTU_Arm::KArm_JointMax[i]);
	
	// Frame 6 移至新手掌中心 ==> 新黑金剛EE
	robot.AddActiveFrame(new DynDHFrame(DHFrame(5, 6, NTU_Arm::KArm_a[5], NTU_Arm::KArm_alpha[5], NTU_Arm::KArm_d[5], NTU_Arm::KArm_theta[5], NULL, &DH2TFMat(-0.00118, 0, -0.026, 0)), ArmHand_M[5], ArmHand_MC[5], ArmHand_Inertial[5]), Deg2Rad(-78), Deg2Rad(78)); // Joint 6 如果使用手臂原本基板 joint limit = -85 ~ 85 (deg)
																																																	                                                                   // Joint 6 如果使用手掌紅色基板 joint limit = -78 ~ 78 (deg)
	#pragma endregion

	#pragma region 新手掌

	Eigen::Matrix4d Pseudo_T1, Pseudo_T2;
	Pseudo_T1 << 0, 0, 1, -0.10382,
				 0, 1, 0, 0,
				-1, 0, 0, 0.026,
				 0, 0, 0, 1;

	// 大拇指
	Pseudo_T2 << sin(Deg2Rad(20)), cos(Deg2Rad(20)),   0,  0.02626,
				 cos(Deg2Rad(20)), -sin(Deg2Rad(20)),  0,  0.04793,
				 0,				   0,				  -1,  0.01762,
				 0,				   0,				   0,  1;
	
	
	robot.AddConstFrame  (new DynDHFrame(DHFrame(6, 7, Pseudo_T1 * Pseudo_T2)));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(7, 8, 0.02904, Deg2Rad(90), -0.05932, Deg2Rad(45)), ArmHand_M[6], ArmHand_MC[6], ArmHand_Inertial[6]), Deg2Rad(0), Deg2Rad(125));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(8, 9, 0, Deg2Rad(-90), 0.04573, Deg2Rad(-90)), ArmHand_M[7], ArmHand_MC[7], ArmHand_Inertial[7]), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(9, 10, 0.0532, 0, 0, 0), ArmHand_M[8], ArmHand_MC[8], ArmHand_Inertial[8]), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(10, 11, 0.0215, 0, 0, Deg2Rad(-6.68)), ArmHand_M[9], ArmHand_MC[9], ArmHand_Inertial[9]), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DynDHFrame(DHFrame(11, 12, 11, 0.0218, 0, 0, Deg2Rad(6.68), 1), ArmHand_M[10], ArmHand_MC[10], ArmHand_Inertial[10]));

	
	// 食指
	Pseudo_T2 << sin(Deg2Rad(20)), 0,	-cos(Deg2Rad(20)),	0.011788,
				 cos(Deg2Rad(20)), 0,	sin(Deg2Rad(20)),	0.05202,
				 0,				  -1,	     0,				0.11494,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DynDHFrame(DHFrame(6, 13, Pseudo_T1 * Pseudo_T2)));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(13, 14, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), ArmHand_M[11], ArmHand_MC[11], ArmHand_Inertial[11]), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(14, 15, 0.0532, 0, 0, 0), ArmHand_M[12], ArmHand_MC[12], ArmHand_Inertial[12]), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(15, 16, 0.0215, 0, 0, Deg2Rad(-6.68)), ArmHand_M[13], ArmHand_MC[13], ArmHand_Inertial[13]), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DynDHFrame(DHFrame(16, 17, 16, 0.0218, 0, 0, Deg2Rad(6.68), 1), ArmHand_M[14], ArmHand_MC[14], ArmHand_Inertial[14]));

	
	// 中指
	Pseudo_T2 << sin(Deg2Rad(10)), 0,	-cos(Deg2Rad(10)),	0.00297,
				 cos(Deg2Rad(10)), 0,	sin(Deg2Rad(10)),	0.019114,
				 0,				  -1,	     0,				0.12494,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DynDHFrame(DHFrame(6, 18, Pseudo_T1 * Pseudo_T2)));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(18, 19, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), ArmHand_M[15], ArmHand_MC[15], ArmHand_Inertial[15]), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(19, 20, 0.0532, 0, 0, 0), ArmHand_M[16], ArmHand_MC[16], ArmHand_Inertial[16]), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(20, 21, 0.0215, 0, 0, Deg2Rad(-6.68)), ArmHand_M[17], ArmHand_MC[17], ArmHand_Inertial[17]), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DynDHFrame(DHFrame(21, 22, 21, 0.0218, 0, 0, Deg2Rad(6.68), 1), ArmHand_M[18], ArmHand_MC[18], ArmHand_Inertial[18]));


	// 無名指
	Pseudo_T2 << -sin(Deg2Rad(2)), 0,	-cos(Deg2Rad(2)),	0.000105,
				 cos(Deg2Rad(2)),  0,	-sin(Deg2Rad(2)),	-0.01483,
				 0,				  -1,	     0,				0.11934,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DynDHFrame(DHFrame(6, 23, Pseudo_T1 * Pseudo_T2)));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(23, 24, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), ArmHand_M[19], ArmHand_MC[19], ArmHand_Inertial[19]), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(24, 25, 0.0532, 0, 0, 0), ArmHand_M[20], ArmHand_MC[20], ArmHand_Inertial[20]), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(25, 26, 0.0215, 0, 0, Deg2Rad(-6.68)), ArmHand_M[21], ArmHand_MC[21], ArmHand_Inertial[21]), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DynDHFrame(DHFrame(26, 27, 26, 0.0218, 0, 0, Deg2Rad(6.68), 1), ArmHand_M[22], ArmHand_MC[22], ArmHand_Inertial[22]));


	// 小拇指
	Pseudo_T2 << -sin(Deg2Rad(10)), 0,	-cos(Deg2Rad(10)),	0.00296,
				 cos(Deg2Rad(10)),  0,	-sin(Deg2Rad(10)),	-0.04882,
				 0,				  -1,	     0,				0.09994,
				 0,				   0,		 0,				1;

	robot.AddConstFrame  (new DynDHFrame(DHFrame(6, 28, Pseudo_T1 * Pseudo_T2)));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(28, 29, 0, Deg2Rad(-90), 0, Deg2Rad(-90)), ArmHand_M[23], ArmHand_MC[23], ArmHand_Inertial[23]), Deg2Rad(-20), Deg2Rad(20));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(29, 30, 0.0532, 0, 0, 0), ArmHand_M[24], ArmHand_MC[24], ArmHand_Inertial[24]), Deg2Rad(-60), Deg2Rad(60));
	robot.AddActiveFrame (new DynDHFrame(DHFrame(30, 31, 0.0215, 0, 0, Deg2Rad(-6.68)), ArmHand_M[25], ArmHand_MC[25], ArmHand_Inertial[25]), Deg2Rad(0), Deg2Rad(90));
	robot.AddPassiveFrame(new DynDHFrame(DHFrame(31, 32, 31, 0.0218, 0, 0, Deg2Rad(6.68), 1), ArmHand_M[26], ArmHand_MC[26], ArmHand_Inertial[26]));

	#pragma endregion

	robot.Set_DLS_Parameter(1, 0.00175, 5, 0.005);
	robot.Build_Robot();

	// 設定重力方向
	Rbt::DynDHFrame::g = Eigen::Vector3d(0, 0, -1) * Gravity;
}