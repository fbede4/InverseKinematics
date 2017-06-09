#include "m_robot.h"

using namespace std;

namespace M_Robot
{

#pragma region Mátrixok és vektorok	  

#define NUMWIDTH 10	//mátrix és vektor elemeinek kiírásához

	Matrix4x4::Matrix4x4()
	{
		for (int i = 0; i < 16; i++)
			matrix[i] = 0.0;
	}

	void Matrix4x4::Transpose()
	{
		double tmp;
		tmp = matrix[1]; matrix[1] = matrix[4]; matrix[4] = tmp;
		tmp = matrix[2]; matrix[2] = matrix[8]; matrix[8] = tmp;
		tmp = matrix[3]; matrix[3] = matrix[12]; matrix[12] = tmp;
		tmp = matrix[6]; matrix[6] = matrix[9]; matrix[9] = tmp;
		tmp = matrix[7]; matrix[7] = matrix[13]; matrix[13] = tmp;
		tmp = matrix[11]; matrix[11] = matrix[14]; matrix[14] = tmp;
	}

	Matrix4x4 Matrix4x4::Transposed() const
	{
		Matrix4x4 mat = *this;
		mat.Transpose();
		return mat;
	}

	Matrix4x4 Robot::eul2r(Vector6& posandrot)
	{
		Matrix4x4 transformMatrix;
		double c1 = cos(posandrot(3)), c2 = cos(posandrot(4)), c3 = cos(posandrot(5));
		double s1 = sin(posandrot(3)), s2 = sin(posandrot(4)), s3 = sin(posandrot(5));
		transformMatrix(0, 0) = c1*c2*c3 - s1*s3;
		transformMatrix(0, 1) = -c1*c2*s3 - s1*c3;
		transformMatrix(0, 2) = c1*s2;
		transformMatrix(0, 3) = posandrot(0);
		transformMatrix(1, 0) = s1*c2*c3 + c1*s3;
		transformMatrix(1, 1) = -s1*c2*s3 + c1*c3;
		transformMatrix(1, 2) = s1*s2;
		transformMatrix(1, 3) = posandrot(1);
		transformMatrix(2, 0) = -s2*c3;
		transformMatrix(2, 1) = s2*s3;
		transformMatrix(2, 2) = c2;
		transformMatrix(2, 3) = posandrot(2);
		transformMatrix(3, 0) = 0;
		transformMatrix(3, 1) = 0;
		transformMatrix(3, 2) = 0;
		transformMatrix(3, 3) = 1;
		return transformMatrix;
	}

	double& Matrix4x4::operator()(int row, int col)
	{
		return matrix[4 * row + col];
	}

	Vector4 Matrix4x4::operator*(const Vector4& other) const
	{
		Vector4 vec;
		for (int i = 0; i < 4; i++)
			for (int ii = 0; ii < 4; ii++)
				vec.vector[i] += matrix[i * 4 + ii] * other.vector[ii];
		return vec;
	}

	Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const
	{
		Matrix4x4 mat;
		for (int i = 0; i < 4; i++)
			for (int ii = 0; ii < 4; ii++)
				for (int iii = 0; iii < 4; iii++)
					mat.matrix[4 * i + ii] += matrix[4 * i + iii] * other.matrix[4 * iii + ii];
		return mat;
	}

	bool Matrix4x4::operator==(Matrix4x4& other)
	{
		for (int i = 0; i < 16; i++)
			if (!NEARaTOb(matrix[i], other.matrix[i]))
				return false;
		return true;
	}

	Vector4::Vector4()
	{
		vector[0] = 0.0;
		vector[1] = 0.0;
		vector[2] = 0.0;
		vector[3] = 0.0;
	}

	double& Vector4::operator()(int index)
	{
		return vector[index];
	}

	bool Vector4::operator==(Vector4 & other)
	{
		for (int i = 0; i < 4; i++)
			if (!NEARaTOb(vector[i], other.vector[i]))
				return false;
		return true;
	}

	ostream& operator<<(ostream& os, Matrix4x4& matrix)
	{
		os << setw(NUMWIDTH) << matrix(0, 0) << setw(NUMWIDTH) << matrix(0, 1) << setw(NUMWIDTH) << matrix(0, 2) << setw(NUMWIDTH) << matrix(0, 3) << endl
			<< setw(NUMWIDTH) << matrix(1, 0) << setw(NUMWIDTH) << matrix(1, 1) << setw(NUMWIDTH) << matrix(1, 2) << setw(NUMWIDTH) << matrix(1, 3) << endl
			<< setw(NUMWIDTH) << matrix(2, 0) << setw(NUMWIDTH) << matrix(2, 1) << setw(NUMWIDTH) << matrix(2, 2) << setw(NUMWIDTH) << matrix(2, 3) << endl
			<< setw(NUMWIDTH) << matrix(3, 0) << setw(NUMWIDTH) << matrix(3, 1) << setw(NUMWIDTH) << matrix(3, 2) << setw(NUMWIDTH) << matrix(3, 3) << endl;
		return os;
	}

	ostream& operator<<(ostream& os, Vector4& vector)
	{
		os << setw(NUMWIDTH) << vector(0) << endl
			<< setw(NUMWIDTH) << vector(1) << endl
			<< setw(NUMWIDTH) << vector(2) << endl
			<< setw(NUMWIDTH) << vector(3) << endl;
		return os;
	}

	ostream & operator<<(ostream & os, Vector6 & vector)
	{
		os << setw(NUMWIDTH) << vector(0) << endl
			<< setw(NUMWIDTH) << vector(1) << endl
			<< setw(NUMWIDTH) << vector(2) << endl
			<< setw(NUMWIDTH) << vector(3) << endl
			<< setw(NUMWIDTH) << vector(4) << endl
			<< setw(NUMWIDTH) << vector(5) << endl;
		return os;
	}

	Vector6::Vector6()
	{
		SetValues(0.0);
	}

	double& Vector6::operator()(int index)
	{
		return vector[index];
	}

	double Vector6::DistanceSquare(Vector6& other)
	{
		double lensq = 0.0;
		double d;
		for (int i = 0; i < 6; i++)
		{
			d = vector[i] - other.vector[i];
			lensq += d*d;
		}
		return lensq;
	}

	void Vector6::SetValues(double value)
	{
		vector[0] = value;
		vector[1] = value;
		vector[2] = value;
		vector[3] = value;
		vector[4] = value;
		vector[5] = value;
	}

	void Vector6::SetValues(double value0, double value1, double value2, double value3, double value4, double value5)
	{
		vector[0] = value0;
		vector[1] = value1;
		vector[2] = value2;
		vector[3] = value3;
		vector[4] = value4;
		vector[5] = value5;
	}

	void Vector6::RadToDeg()
	{
		double deg = 180 / M_PI;
		vector[0] *= deg;
		vector[1] *= deg;
		vector[2] *= deg;
		vector[3] *= deg;
		vector[4] *= deg;
		vector[5] *= deg;
	}

	void Vector6::DegToRad()
	{
		double deg = M_PI / 180;
		vector[0] *= deg;
		vector[1] *= deg;
		vector[2] *= deg;
		vector[3] *= deg;
		vector[4] *= deg;
		vector[5] *= deg;
	}

	bool Vector6::operator==(Vector6 & other)
	{
		for (int i = 0; i < 6; i++)
			if (!NEARaTOb(vector[i], other.vector[i]))
				return false;
		return true;
	}
#pragma endregion

#pragma region Robotkar irányítása

	Robot::Robot()
	{
		double deg = M_PI / 180.0;	//fok és radián között váltás
		joints[0].SetParams(0.350, 0.095, -170 * deg, 170 * deg);
		joints[1].SetParams(0.0, 0.245, -90 * deg, 135 * deg);
		joints[2].SetParams(0.0, 0.135, -20 * deg, 171 * deg);
		joints[3].SetParams(0.270, 0.0, -160 * deg, 160 * deg);
		joints[4].SetParams(0.0, 0.0, -120 * deg, 120 * deg);
		joints[5].SetParams(0.085, 0.0, -360 * deg, 360 * deg);
		//kezdõállapot beállítás
		currentSolutionIndex = 0;
		ForwardKinematic(currentState);
	}

	Robot::Robot(bool is_KR5)
	{
		double deg = M_PI / 180.0;	//fok és radián között váltás
		if (is_KR5)
		{
			joints[0].SetParams(0.350, -0.095, -170 * deg, 170 * deg);
			joints[1].SetParams(0.0, 0.245, -90 * deg, 135 * deg);
			joints[2].SetParams(0.0, -0.135, -20 * deg, 171 * deg);
			joints[3].SetParams(0.270, 0.0, -160 * deg, 160 * deg);
			joints[4].SetParams(0.0, 0.0, -120 * deg, 120 * deg);
			joints[5].SetParams(0.085, 0.0, -360 * deg, 360 * deg);
		}
		else
		{
			joints[0].SetParams(0.350, 0.095, -170 * deg, 170 * deg);
			joints[1].SetParams(0.0, 0.245, -90 * deg, 135 * deg);
			joints[2].SetParams(0.0, 0.135, -20 * deg, 171 * deg);
			joints[3].SetParams(0.270, 0.0, -160 * deg, 160 * deg);
			joints[4].SetParams(0.0, 0.0, -120 * deg, 120 * deg);
			joints[5].SetParams(0.085, 0.0, -360 * deg, 360 * deg);
		}
		//kezdõállapot beállítás
		currentSolutionIndex = 0;
		ForwardKinematic(currentState);
	}

	Matrix4x4 eul2r(Vector6& posandrot)
	{
		Matrix4x4 transformMatrix;
		double c1 = cos(posandrot(3)), c2 = cos(posandrot(4)), c3 = cos(posandrot(5));
		double s1 = sin(posandrot(3)), s2 = sin(posandrot(4)), s3 = sin(posandrot(5));
		transformMatrix(0, 0) = c1*c2*c3 - s1*s3;
		transformMatrix(0, 1) = -c1*c2*s3 - s1*c3;
		transformMatrix(0, 2) = c1*s2;
		transformMatrix(0, 3) = posandrot(0);
		transformMatrix(1, 0) = s1*c2*c3 + c1*s3;
		transformMatrix(1, 1) = -s1*c2*s3 + c1*c3;
		transformMatrix(1, 2) = s1*s2;
		transformMatrix(1, 3) = posandrot(1);
		transformMatrix(2, 0) = -s2*c3;
		transformMatrix(2, 1) = s2*s3;
		transformMatrix(2, 2) = c2;
		transformMatrix(2, 3) = posandrot(2);
		transformMatrix(3, 0) = 0;
		transformMatrix(3, 1) = 0;
		transformMatrix(3, 2) = 0;
		transformMatrix(3, 3) = 1;
		return transformMatrix;
	}

	Vector6 r2eul(Matrix4x4& transformMatrix)
	{
		Vector6 posandrot;
		posandrot(0) = transformMatrix(0, 3);
		posandrot(1) = transformMatrix(1, 3);
		posandrot(2) = transformMatrix(2, 3);
		posandrot(3) = atan2(transformMatrix(1, 2), transformMatrix(0, 2));
		posandrot(4) = atan2(cos(posandrot(3))*transformMatrix(0, 2) + sin(posandrot(3))*transformMatrix(1, 2), transformMatrix(2, 2));
		posandrot(5) = atan2(transformMatrix(2, 1), -transformMatrix(2, 0));
		return posandrot;
	}

	Matrix4x4 Robot::TransformMatrix03(Vector6& request)
	{
		Matrix4x4 mat03;
		double cos1, cos2, cos23, sin1, sin2, sin23;
		cos1 = cos(request(0));
		cos2 = cos(request(1));
		cos23 = cos(request(1) + request(2));
		sin1 = sin(request(0));
		sin2 = sin(request(1));
		sin23 = sin(request(1) + request(2));

		mat03(0, 0) = -cos1*cos23;
		mat03(0, 1) = sin1;
		mat03(0, 2) = cos1*sin23;
		mat03(0, 3) = -joints[2].a*cos1*cos23 + joints[1].a*cos1*sin2 + joints[0].a*cos1;
		mat03(1, 0) = -sin1*cos23;
		mat03(1, 1) = -cos1;
		mat03(1, 2) = sin1*sin23;
		mat03(1, 3) = -joints[2].a*sin1*cos23 + joints[1].a*sin1*sin2 + joints[0].a*sin1;
		mat03(2, 0) = sin23;
		mat03(2, 1) = 0;
		mat03(2, 2) = cos23;
		mat03(2, 3) = joints[2].a*sin23 + joints[1].a*cos2 + joints[0].d;
		mat03(3, 0) = 0;
		mat03(3, 1) = 0;
		mat03(3, 2) = 0;
		mat03(3, 3) = 1;
		return mat03;
	}


	Vector6 Robot::ikine6s(Matrix4x4& request) {

		//elõkészületek
		//ha nem jön ki megoldás, átállítja magát
		for (int i = 0; i < 2; i++)
			validSolution[i] = true;
		solutionNumber = 4;
		Vector6 K = r2eul(request);
		double x = request(0, 3) - 0.085*cos(K(3))*sin(K(4)), y = request(1, 3) - 0.085*sin(K(3))*sin(K(4)), z = request(2, 3) + 0.085*cos(K(4));
		//double x = request(0, 3), y = request(1, 3), z = request(2, 3);
		double ox = request(0, 1), oy = request(1, 1), oz = request(2, 1);
		double ax = request(0, 2), ay = request(1, 2), az = transformMatrix(2, 2);
		/*------------------elsõ megoldás----------------------*/
		//elsõ
		double r = sqrt(x*x + y*y);
		jointState[0](0) = atan2(y, x) + M_PI;
		//második
		double tmp_x = x*cos(jointState[0](0)) + y*sin(jointState[0](0)) - joints[0].a;
		r = sqrt(tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d));
		double psi = acos((joints[1].a*joints[1].a - joints[3].d*joints[3].d - joints[2].a*joints[2].a + tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d)) / (2.0*joints[1].a*r));
		jointState[0](1) = atan2((z - joints[0].d), tmp_x) - psi;
		//harmadik
		double nu = cos(jointState[0](1))*tmp_x + sin(jointState[0](1))*(z - joints[0].d) - joints[1].a;
		double du = sin(jointState[0](1))*tmp_x - cos(jointState[0](1))*(z - joints[0].d);
		jointState[0](2) = atan2(joints[2].a, joints[3].d) - atan2(nu, du);
		//negyedik
		double cos4, sin4, m3, cos1, sin1, y1, m2, m1;
		cos1 = cos(jointState[0](0));
		sin1 = sin(jointState[0](0));
		y1 = cos1*request(0, 2) + sin1*request(1, 2);
		m2 = sin1*request(0, 2) - cos1*request(1, 2);
		m1 = (cos(jointState[0](1) - jointState[0](2)))*y1 + (sin(jointState[0](1) - jointState[0](2)))*request(2, 2);
		jointState[0](3) = atan2(-m2, -m1);
		//ötödik
		cos4 = cos(jointState[0](3));
		sin4 = sin(jointState[0](3));
		nu = -cos4*m1 - m2*sin4;
		m3 = -request(2, 2)*(cos(jointState[0](1) - jointState[0](2))) + y1*(sin(jointState[0](1) - jointState[0](2)));
		jointState[0](4) = -atan2(nu, m3);
		//hatodik
		double z1, l2, l1, l3, a1, a3;
		z1 = cos1*request(0, 1) + sin1*request(1, 1);
		l2 = sin1*request(0, 1) - cos1*request(1, 1);
		l1 = z1*(cos(jointState[0](1) - jointState[0](2))) + request(2, 1)*(sin(jointState[0](1) - jointState[0](2)));
		l3 = z1*(sin(jointState[0](1) - jointState[0](2))) - request(2, 1)*(cos(jointState[0](1) - jointState[0](2)));
		a1 = l1*cos4 + l2*sin4;
		a3 = l1*sin4 - l2*cos4;
		nu = -a1*cos(jointState[0](4)) - l3*sin(jointState[0](4));
		du = -a3;
		jointState[0](5) = atan2(nu, du);


		/*------------------második megoldás----------------------*/
		//elsõ
		jointState[1](0) = jointState[0](0);
		//második
		tmp_x = x*cos(jointState[1](0)) + y*sin(jointState[1](0)) - joints[0].a;
		r = sqrt(tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d));
		psi = acos((joints[1].a*joints[1].a - joints[3].d*joints[3].d - joints[2].a*joints[2].a + tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d)) / (2.0*joints[1].a*r));
		jointState[1](1) = atan2((z - joints[0].d), tmp_x) + psi;
		//harmadik
		nu = cos(jointState[1](1))*tmp_x + sin(jointState[1](1))*(z - joints[0].d) - joints[1].a;
		du = sin(jointState[1](1))*tmp_x - cos(jointState[1](1))*(z - joints[0].d);
		jointState[1](2) = atan2(joints[2].a, joints[3].d) - atan2(nu, du);
		//negyedik
		cos1 = cos(jointState[1](0));
		sin1 = sin(jointState[1](0));
		y1 = cos1*request(0, 2) + sin1*request(1, 2);
		m2 = sin1*request(0, 2) - cos1*request(1, 2);
		m1 = (cos(jointState[1](1) - jointState[1](2)))*y1 + (sin(jointState[1](1) - jointState[1](2)))*request(2, 2);
		jointState[1](3) = atan2(-m2, -m1);
		//ötödik
		cos4 = cos(jointState[1](3));
		sin4 = sin(jointState[1](3));
		nu = -cos4*m1 - m2*sin4;
		m3 = -request(2, 2)*(cos(jointState[1](1) - jointState[1](2))) + y1*(sin(jointState[1](1) - jointState[1](2)));
		jointState[1](4) = atan2(nu, m3);
		//hatodik
		z1 = cos1*request(0, 1) + sin1*request(1, 1);
		l2 = sin1*request(0, 1) - cos1*request(1, 1);
		l1 = z1*(cos(jointState[1](1) - jointState[1](2))) + request(2, 1)*(sin(jointState[1](1) - jointState[1](2)));
		l3 = z1*(sin(jointState[1](1) - jointState[1](2))) - request(2, 1)*(cos(jointState[1](1) - jointState[1](2)));
		a1 = l1*cos4 + l2*sin4;
		a3 = l1*sin4 - l2*cos4;
		nu = -a1*cos(jointState[1](4)) - l3*sin(jointState[1](4));
		du = -a3;
		jointState[1](5) = atan2(nu, du);


		/*------------------harmadik megoldás----------------------*/
		//elsõ
		jointState[2](0) = jointState[0](0)-M_PI;
		//második
		tmp_x = x*cos(jointState[2](0)) + y*sin(jointState[2](0)) - joints[0].a;
		r = sqrt(tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d));
		psi = acos((joints[1].a*joints[1].a - joints[3].d*joints[3].d - joints[2].a*joints[2].a + tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d)) / (2.0*joints[1].a*r));
		jointState[2](1) = atan2((z - joints[0].d), tmp_x) - psi;
		//harmadik
		nu = cos(jointState[2](1))*tmp_x + sin(jointState[2](1))*(z - joints[0].d) - joints[1].a;
		du = sin(jointState[2](1))*tmp_x - cos(jointState[2](1))*(z - joints[0].d);
		jointState[2](2) = atan2(joints[2].a, joints[3].d) - atan2(nu, du);
		//negyedik
		cos1 = cos(jointState[2](0));
		sin1 = sin(jointState[2](0));
		y1 = cos1*request(0, 2) + sin1*request(1, 2);
		m2 = sin1*request(0, 2) - cos1*request(1, 2);
		m1 = (cos(jointState[2](1) - jointState[2](2)))*y1 + (sin(jointState[2](1) - jointState[2](2)))*request(2, 2);
		jointState[2](3) = atan2(-m2, -m1);
		//ötödik
		cos4 = cos(jointState[2](3));
		sin4 = sin(jointState[2](3));
		nu = -cos4*m1 - m2*sin4;
		m3 = -request(2, 2)*(cos(jointState[2](1) - jointState[2](2))) + y1*(sin(jointState[2](1) - jointState[2](2)));
		jointState[2](4) = -atan2(nu, m3);
		//hatodik
		z1 = cos1*request(0, 1) + sin1*request(1, 1);
		l2 = sin1*request(0, 1) - cos1*request(1, 1);
		l1 = z1*(cos(jointState[2](1) - jointState[2](2))) + request(2, 1)*(sin(jointState[2](1) - jointState[2](2)));
		l3 = z1*(sin(jointState[2](1) - jointState[2](2))) - request(2, 1)*(cos(jointState[2](1) - jointState[2](2)));
		a1 = l1*cos4 + l2*sin4;
		a3 = l1*sin4 - l2*cos4;
		nu = -a1*cos(jointState[2](4)) - l3*sin(jointState[2](4));
		du = -a3;
		jointState[2](5) = atan2(nu, du);


		/*------------------negyedik megoldás----------------------*/
		//elsõ
		jointState[3](0) = jointState[0](0) - M_PI;
		//második
		tmp_x = x*cos(jointState[3](0)) + y*sin(jointState[3](0)) - joints[0].a;
		r = sqrt(tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d));
		psi = acos((joints[1].a*joints[1].a - joints[3].d*joints[3].d - joints[2].a*joints[2].a + tmp_x*tmp_x + (z - joints[0].d)*(z - joints[0].d)) / (2.0*joints[1].a*r));
		jointState[3](1) = atan2((z - joints[0].d), tmp_x) + psi;
		//harmadik
		nu = cos(jointState[3](1))*tmp_x + sin(jointState[3](1))*(z - joints[0].d) - joints[1].a;
		du = sin(jointState[3](1))*tmp_x - cos(jointState[3](1))*(z - joints[0].d);
		jointState[3](2) = atan2(joints[2].a, joints[3].d) - atan2(nu, du);
		//negyedik
		cos1 = cos(jointState[3](0));
		sin1 = sin(jointState[3](0));
		y1 = cos1*request(0, 2) + sin1*request(1, 2);
		m2 = sin1*request(0, 2) - cos1*request(1, 2);
		m1 = (cos(jointState[3](1) - jointState[3](2)))*y1 + (sin(jointState[3](1) - jointState[3](2)))*request(2, 2);
		jointState[3](3) = atan2(-m2, -m1);
		//ötödik
		cos4 = cos(jointState[3](3));
		sin4 = sin(jointState[3](3));
		nu = -cos4*m1 - m2*sin4;
		m3 = -request(2, 2)*(cos(jointState[3](1) - jointState[3](2))) + y1*(sin(jointState[3](1) - jointState[3](2)));
		jointState[3](4) = -atan2(nu, m3);
		//hatodik
		z1 = cos1*request(0, 1) + sin1*request(1, 1);
		l2 = sin1*request(0, 1) - cos1*request(1, 1);
		l1 = z1*(cos(jointState[3](1) - jointState[3](2))) + request(2, 1)*(sin(jointState[3](1) - jointState[3](2)));
		l3 = z1*(sin(jointState[3](1) - jointState[3](2))) - request(2, 1)*(cos(jointState[3](1) - jointState[3](2)));
		a1 = l1*cos4 + l2*sin4;
		a3 = l1*sin4 - l2*cos4;
		nu = -a1*cos(jointState[3](4)) - l3*sin(jointState[3](4));
		du = -a3;
		jointState[3](5) = atan2(nu, du);

		double offsets[6] = { M_PI, M_PI / 2, -M_PI / 2, M_PI, 0, 0 };
		for (int i = 0; i < 4; i++)
		{
			for (int k = 0; k < 4; k++)
			{
				jointState[k](i) = jointState[k](i) - offsets[i];
			}
		}
		jointState[0](2) = -jointState[0](2);
		jointState[1](2) = -jointState[1](2);
		jointState[0](3) = jointState[0](3) + M_PI;
		jointState[0](4) = -(jointState[0](4) + M_PI);
		jointState[1](4) = jointState[1](4) + M_PI;
		for (int k = 0; k < 4; k++)
		{
			ValidateVector6(jointState[k]);
		}
		/*bool chooseSolution[2];
		chooseSolution[0] = true;
		chooseSolution[1] = true;
		for (int i = 0; i < 2; i++)
		{
			for (int ii = 0; ii < 6; ii++)
			{
				ROUNDaTOb(jointState[i](ii), 0);
				if (!joints[ii].StateInBoundaries(jointState[i](ii)))
				{
					chooseSolution[i]=false;
					break;
				}
			}
		}
		if(chooseSolution[0]==true)
			return jointState[0];
		if (chooseSolution[1] == true && chooseSolution[0]==false)
			return jointState[1];
		else
		{
			throw "Cannot reach desired position";
		}*/
		CheckSolutionValidity();
		SearchBestSolution();
		return currentState;
	}


	Matrix4x4 Robot::TransformMatrix36(Vector6& request)
	{
		Matrix4x4 mat36;
		double cos4, cos5, cos6, sin4, sin5, sin6;
		cos4 = cos(request(3));
		cos5 = cos(request(4));
		cos6 = cos(request(5));
		sin4 = sin(request(3));
		sin5 = sin(request(4));
		sin6 = sin(request(5));

		mat36(0, 0) = cos4*cos5*cos6 - sin4*sin6;
		mat36(0, 1) = -cos4*cos5*sin6 - sin4*cos6;
		mat36(0, 2) = -cos4*sin5;
		mat36(0, 3) = -joints[5].d*cos4*sin5;
		mat36(1, 0) = sin4*cos5*cos6 + cos4*sin6;
		mat36(1, 1) = -sin4*cos5*sin6 + cos4*cos6;
		mat36(1, 2) = -sin4*sin5;
		mat36(1, 3) = -joints[5].d*sin4*sin5;
		mat36(2, 0) = sin5*cos6;
		mat36(2, 1) = -sin5*sin6;
		mat36(2, 2) = cos5;
		mat36(2, 3) = joints[3].d + joints[5].d*cos5;
		mat36(3, 0) = 0;
		mat36(3, 1) = 0;
		mat36(3, 2) = 0;
		mat36(3, 3) = 1;
		return mat36;
	}

	Matrix4x4 Robot::ForwardKinematic(Vector6& request)
	{
		currentState = request;
		bool reachable = true;
		//a kért csuklóállások határon belül kell legyenek
		for (int i = 0; i < 6; i++)
			reachable &= joints[i].StateInBoundaries(request(i));
		if (!reachable)
		{
			currentState = jointState[currentSolutionIndex];
			throw "Cannot turn all joints that much";
		}

		//transzformációs mátrix kiszámítása
		transformMatrix = TransformMatrix03(currentState) * TransformMatrix36(currentState);
		//transformMatrix = TransformMatrix06(currentState);
		for (int i = 0; i < 4; i++)for (int ii = 0; ii < 4; ii++)
			ROUNDaTOb(transformMatrix(i, ii), 0);

		//mostantól egy megoldás ismert, a többi nem ezt a pozíciót mutatja
		jointState[currentSolutionIndex] = currentState;
		solutionNumber = 1;
		for (int i = 0; i < 8; i++)
			validSolution[i] = i == currentSolutionIndex;
		CalculatePosAndRotFromTransformMatrix();
		return transformMatrix;
	}

	void Robot::CalculatePosAndRotFromTransformMatrix()
	{
		posAndRot(0) = transformMatrix(0, 3);
		posAndRot(1) = transformMatrix(1, 3);
		posAndRot(2) = transformMatrix(2, 3);
		posAndRot(3) = atan2(transformMatrix(1, 2), transformMatrix(0, 2));
		posAndRot(4) = atan2(cos(posAndRot(3))*transformMatrix(0, 2) + sin(posAndRot(3))*transformMatrix(1, 2), transformMatrix(2, 2));
		posAndRot(5) = atan2(transformMatrix(2, 1), -transformMatrix(2, 0));
	}

	void Robot::CalculateTransformMatrixFromPosAndRot()
	{
		double c1 = cos(posAndRot(3)), c2 = cos(posAndRot(4)), c3 = cos(posAndRot(5));
		double s1 = sin(posAndRot(3)), s2 = sin(posAndRot(4)), s3 = sin(posAndRot(5));
		transformMatrix(0, 0) = c1*c2*c3 - s1*s3;
		transformMatrix(0, 1) = -c1*c2*s3 - s1*c3;
		transformMatrix(0, 2) = c1*s2;
		transformMatrix(0, 3) = posAndRot(0);
		transformMatrix(1, 0) = s1*c2*c3 + c1*s3;
		transformMatrix(1, 1) = -s1*c2*s3 + c1*c3;
		transformMatrix(1, 2) = s1*s2;
		transformMatrix(1, 3) = posAndRot(1);
		transformMatrix(2, 0) = -s2*c3;
		transformMatrix(2, 1) = s2*s3;
		transformMatrix(2, 2) = c2;
		transformMatrix(2, 3) = posAndRot(2);
		transformMatrix(3, 0) = 0;
		transformMatrix(3, 1) = 0;
		transformMatrix(3, 2) = 0;
		transformMatrix(3, 3) = 1;
	}

	void Robot::MobileDataToMatrix(M_Robot::Vector6& vector)
	{
		M_Robot::Matrix4x4 mat;
		double c1, c2, c3, s1, s2, s3;
		c1 = cos(vector(3));
		c2 = cos(vector(4));
		c3 = cos(vector(5));
		s1 = sin(vector(3));
		s2 = sin(vector(4));
		s3 = sin(vector(5));
		//s3 = sin(-vector(5));	   //arra az esetre, ha nem mûködik a másik
		transformMatrix(0, 0) = c1*c3 - s1*s2*s3;
		transformMatrix(0, 1) = -c2*s1;
		transformMatrix(0, 2) = c1*s3 + c3*s1*s2;
		transformMatrix(0, 3) = vector(0);
		transformMatrix(1, 0) = c3*s1 + c1*s2*s3;
		transformMatrix(1, 1) = c1*c2;
		transformMatrix(1, 2) = s1*s3 - c1*c3*s2;
		transformMatrix(1, 3) = vector(1);
		transformMatrix(2, 0) = -c2*s3;
		transformMatrix(2, 1) = s2;
		transformMatrix(2, 2) = c2*c3;
		transformMatrix(2, 3) = vector(2);
		transformMatrix(3, 0) = 0;
		transformMatrix(3, 1) = 0;
		transformMatrix(3, 2) = 0;
		transformMatrix(3, 3) = 1;
	}

	void Robot::CalculateJoint1(double x, double y)
	{
		jointState[0](0) = atan2(y, x);
		jointState[1](0) = jointState[0](0);
		jointState[2](0) = jointState[0](0);
		jointState[3](0) = jointState[0](0);
		//második két megoldás ha ellenkezõ irányba fordul az elsõ csukló
		jointState[4](0) = jointState[0](0) < 0 ? jointState[0](0) + M_PI : jointState[0](0) - M_PI;
		jointState[5](0) = jointState[4](0);
		jointState[6](0) = jointState[4](0);
		jointState[7](0) = jointState[4](0);
		//ha a csukló állása határon kívül van, azzal ne számoljon	 
		if (!joints[0].StateInBoundaries(jointState[0](0)))
		{
			validSolution[0] = false;
			validSolution[1] = false;
			validSolution[2] = false;
			validSolution[3] = false;
		}
		if (!joints[0].StateInBoundaries(jointState[4](0)))
		{
			validSolution[4] = false;
			validSolution[5] = false;
			validSolution[6] = false;
			validSolution[7] = false;
		}
	}

	void Robot::CalculateJoint23(double r, double z, int solnum)
	{
		double s11, s12, s13, s14;	//új változók kevesebb számoláshoz
		s11 = (r - joints[0].a) / joints[1].a;
		s12 = joints[2].a / joints[1].a;
		s13 = joints[3].d / joints[1].a;
		s14 = (z - joints[0].d) / joints[1].a;
		double D, A, B;	//új változók megint
		D = 1 - s11*s11 - s12*s12 - s13*s13 - s14*s14;
		A = 2 * (s11*s12 - s13*s14);
		B = 2 * (-s11*s13 - s12*s14);
		double deg23_1, deg23_2;	//szögek
		double mem_sqrt;	//gyakran szükséges tag
		mem_sqrt = A*A + B*B - D*D;
		if (mem_sqrt < 0)
		{
			validSolution[solnum] = false;
			validSolution[solnum + 1] = false;
			validSolution[solnum + 2] = false;
			validSolution[solnum + 3] = false;
			return;
		}
		mem_sqrt = sqrt(mem_sqrt);
		deg23_1 = atan2(B*D + A*mem_sqrt, A*D - B*mem_sqrt);	//egyik megoldás
		deg23_2 = atan2(B*D - A*mem_sqrt, A*D + B*mem_sqrt);	//másik megoldás
		double A2B2 = A*A + B*B;	//gyakran szükséges tag
		double cos23, sin23;	//2., 3. szögek összegének szinusza, koszinusza
								//egyik megoldás
		cos23 = (D*A - B*mem_sqrt) / A2B2;
		sin23 = (D*B + A*mem_sqrt) / A2B2;
		jointState[solnum](1) = atan2((s11 + s12*cos23 - s13*sin23), (s14 - s12*sin23 - s13*cos23));
		jointState[solnum](2) = deg23_1 - jointState[solnum](1);
		while (jointState[solnum](2) < -M_PI)
			jointState[solnum](2) += M_PI;
		while (jointState[solnum](2) > M_PI)
			jointState[solnum](2) -= M_PI;
		jointState[solnum + 1](1) = jointState[solnum](1);
		jointState[solnum + 1](2) = jointState[solnum](2);
		//másik megoldás
		cos23 = (D*A + B*mem_sqrt) / A2B2;
		sin23 = (D*B - A*mem_sqrt) / A2B2;
		jointState[solnum + 2](1) = atan2((s11 + s12*cos23 - s13*sin23), (s14 - s12*sin23 - s13*cos23));
		jointState[solnum + 2](2) = deg23_2 - jointState[solnum + 2](1);
		while (jointState[solnum + 2](2) < -M_PI)
			jointState[solnum + 2](2) += 2 * M_PI;
		while (jointState[solnum + 2](2) > M_PI)
			jointState[solnum + 2](2) -= 2 * M_PI;
		jointState[solnum + 3](1) = jointState[solnum + 2](1);
		jointState[solnum + 3](2) = jointState[solnum + 2](2);
	}

	void Robot::CalculateJoint456(Matrix4x4& request)
	{
		Matrix4x4 matrix;
		for (int i = 0; i < 8; i+=2)
		{
			if (validSolution[i])
			{
				matrix = TransformMatrix03(jointState[i]);
				matrix.Transpose();
				matrix = matrix * request;
				if (NEARaTOb(matrix(2, 2), 1))
				{
					double sumdeg = atan2(matrix(1, 0), matrix(0, 0));
					jointState[i](4) = 0;
					double prevsumdeg = currentState(3) + currentState(5);
					while (prevsumdeg > M_PI)
						prevsumdeg -= 2 * M_PI;
					while (prevsumdeg < -M_PI)
						prevsumdeg += 2 * M_PI;
					double diff = (sumdeg - prevsumdeg) / 2;
					jointState[i](3) += diff;
					if (jointState[i](3) > joints[3].limit_pos || jointState[i](3) < joints[3].limit_neg)
						diff *= 2;
					jointState[i](5) += diff;
					if (jointState[i](5) > joints[5].limit_pos)
						jointState[i](5) -= 2 / M_PI;
					if (jointState[i](5) < joints[5].limit_neg)
						jointState[i](5) += 2 / M_PI;
					validSolution[i + 1] = false;	//a következõ megoldás ugyanez lenne, így kerüljük a doblázást
				}
				else
				{
					double cos4, sin4, sin5, shift;
					jointState[i](3) = atan2(matrix(1, 2), matrix(0, 2));
					cos4 = cos(jointState[i](3));
					sin4 = sin(jointState[i](3));
					jointState[i](4) = atan2(-cos4*matrix(0, 2) - sin4*matrix(1, 2), matrix(2, 2));
					if (jointState[i](4) > 0)
						jointState[i](5) = atan2(-matrix(2, 1), matrix(2, 0));
					else
						jointState[i](5) = atan2(matrix(2, 1), -matrix(2, 0));
					shift = jointState[i](5) > currentState(5) ? -2 * M_PI : 2 * M_PI;
					if (abs(jointState[i](5) - currentState(5)) > M_PI)
						jointState[i](5) += shift;

					jointState[i + 1](3) = atan2(-matrix(1, 2), -matrix(0, 2));
					cos4 = cos(jointState[i + 1](3));
					sin4 = sin(jointState[i + 1](3));
					jointState[i + 1](4) = atan2(-cos4*matrix(0, 2) - sin4*matrix(1, 2), matrix(2, 2));
					if (jointState[i + 1](4) > 0)
						jointState[i + 1](5) = atan2(-matrix(2, 1), matrix(2, 0));
					else
						jointState[i + 1](5) = atan2(matrix(2, 1), -matrix(2, 0));
					shift = jointState[i + 1](5) > currentState(5) ? -2 * M_PI : 2 * M_PI;
					if (abs(jointState[i + 1](5) - currentState(5)) > M_PI)
						jointState[i + 1](5) += shift;
				}
			}
		}
	}

	Vector6 Robot::InverseKinematic(Matrix4x4& request)
	{
		//elõkészületek
		//ha nem jön ki megoldás, átállítja magát
		for (int i = 0; i < 8; i++)
			validSolution[i] = true;
		solutionNumber = 8;
		//végpont koordináták lekérése d6 tag nélkül
		double x, y, z;
		x = request(0, 3) - request(0, 2) * joints[5].d;
		y = request(1, 3) - request(1, 2) * joints[5].d;
		z = request(2, 3) - request(2, 2) * joints[5].d;
		//így x, y koordináta helyettesíthetõ sugárral
		double r = sqrt(x*x + y*y);

		//elsõ szög
		CalculateJoint1(x, y);

		//második és harmadik szög		   
		//ha az elsõ csukló határon belül van, akkor nézze
		if (validSolution[0])
			CalculateJoint23(r, z, 0);
		if (validSolution[4])
			CalculateJoint23(-r, z, 4);

		//utolsó három	
		CalculateJoint456(request);

		//végsõ ellenõrzés
		CheckSolutionValidity();
		SearchBestSolution();

		CalculatePosAndRotFromTransformMatrix();
		return currentState;
	}

	void Robot::CheckSolutionValidity()
	{
		for (int i = 0; i < 4; i++)
		{
			if (validSolution[i])
				for (int ii = 0; ii < 6; ii++)
				{
					ROUNDaTOb(jointState[i](ii), 0);
					if (!joints[ii].StateInBoundaries(jointState[i](ii)))
					{
						validSolution[i] = false;
						solutionNumber--;
						break;
					}
				}
			else
				solutionNumber--;
		}
		if (solutionNumber == 0)
			throw "Cannot reach desired position";
	}

	void Robot::SearchBestSolution()
	{
		double bestlen = 1e10;
		double len;
		for (int i = 0; i < 8; i++)
		{
			if (validSolution[i])
			{
				len = currentState.DistanceSquare(jointState[i]);
				if (len < bestlen)
				{
					bestlen = len;
					currentSolutionIndex = i;
				}
			}
		}
		currentState = jointState[currentSolutionIndex];
	}

	Vector6 Robot::InverseKinematic(Vector6& request)
	{
		posAndRot = request;
		MobileDataToMatrix(request);
		//CalculateTransformMatrixFromPosAndRot();
		//cout << transformMatrix << endl;
		return InverseKinematic(transformMatrix);
	}


	int Robot::ValidSolutionNumber()
	{
		return solutionNumber;
	}

	bool Robot::IsValidSolution(int index)
	{
		return validSolution[index];
	}

	int Robot::GetCurrentSolutionIndex()
	{
		return currentSolutionIndex;
	}

	Vector6 Robot::GetIkineSolution()
	{
		return currentState;
	}

	Vector6 Robot::GetIkineSolution(int index)
	{
		return jointState[index];
	}

	Matrix4x4 Robot::GetTransformMatrix()
	{
		return transformMatrix;
	}

	Vector6 Robot::GetPositionAndRotation()
	{
		return posAndRot;
	}

	void Robot::ChangeState(int index)
	{
		currentState = jointState[index];
		currentSolutionIndex = index;
	}

	void Joint::SetParams(double d, double a, double limit_neg, double limit_pos)
	{
		this->d = d;
		this->a = a;
		this->limit_neg = limit_neg;
		this->limit_pos = limit_pos;
	}

	bool Joint::StateInBoundaries(double state)
	{
		return state >= limit_neg && state <= limit_pos;
	}


	void ValidateVector6(Vector6& vector)
	{
		for (int i = 0; i < 6; i++)
		{
			while (vector(i) < -M_PI)
				vector(i) += 2*M_PI;
			while (vector(i) > M_PI)
				vector(i) -= 2*M_PI;
		}
	}

#pragma endregion	   

}