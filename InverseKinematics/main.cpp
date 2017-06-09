#include "m_robot.h"

int main()
{
	M_Robot::Robot mit;
	M_Robot::Vector6 v;
	v.SetValues(1);
	M_Robot::Matrix4x4 mat;
	mat = mit.ForwardKinematic(v);
	std::cout << "Legjobb: " << std::endl << mit.InverseKinematic(mat) << std::endl;
	std::cout << "Osszesen: " << mit.ValidSolutionNumber() << std::endl;
	for (int i = 0; i < 8; i++)
		if (mit.IsValidSolution(i))
			std::cout << mit.GetIkineSolution(i) << std::endl;
	system("pause");
	return 0;
}