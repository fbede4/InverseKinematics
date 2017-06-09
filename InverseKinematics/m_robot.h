#pragma once
#ifndef M_ROBOT_H
#define	M_ROBOT_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstdlib>
  
namespace M_Robot
{

#pragma region M�trixok �s vektorok	 

#define THRESHOLD 1e-5
#define ROUNDaTOb(a, b) ((a-b < THRESHOLD && a-b > -THRESHOLD)	? a = b : a)
#define NEARaTOb(a, b) (a-b < THRESHOLD && a-b > -THRESHOLD)


	class Matrix4x4;
	class Vector4;
	class Vector6;

	//rot�ci�s m�trixb�l euler sz�gek sz�m�t�sa
	Vector6 r2eul(Matrix4x4& transformMatrix);
	//rot�ci�s m�trixb�l euler sz�gek sz�m�t�sa
	Matrix4x4 eul2r(Vector6& posandrot);
	//-PI �s PI k�z� teszi a vektor elemeit	k*PI eltol�ssal ahol k eg�sz
	void ValidateVector6(Vector6& vector);

	class Matrix4x4
	{
		friend Vector4;
		double matrix[16];

	public:
		Matrix4x4();
		void Transpose();	//transzpon�lja a m�trixot
		Matrix4x4 Transposed() const;	//a m�trix transzpon�ltj�t adja vissza
		double& operator()(const int row, const int col);
		Vector4 operator*(const Vector4& other) const;
		Matrix4x4 operator*(const Matrix4x4& other) const;
		bool operator==(Matrix4x4& other);
	};

	class Vector4
	{
		friend Matrix4x4;
		double vector[4];

	public:
		Vector4();
		double& operator()(int index);
		bool operator==(Vector4& other);
	};

	class Vector6
	{
		double vector[6];

	public:
		Vector6();
		double& operator()(int index);
		double DistanceSquare(Vector6& other);
		void SetValues(double value);
		void SetValues(double value0, double value1, double value2, double value3, double value4, double value5);
		void RadToDeg();
		void DegToRad();
		bool operator==(Vector6& other);
	};

	std::ostream& operator<<(std::ostream& os, Matrix4x4& matrix);
	std::ostream& operator<<(std::ostream& os, Vector4& vector);
	std::ostream& operator<<(std::ostream& os, Vector6& vector);

#pragma endregion

#pragma region Robotkar ir�ny�t�sa

	class Robot;
	class Joint;

	//egy csukl� adatainak strukt�r�ja
	struct Joint
	{
		double limit_neg;	//negat�v limit
		double limit_pos;	//pozit�v limit
		double a;			//a eltol�s
		double d;			//d eltol�s

	public:
		//be�ll�tja a param�tereket
		void SetParams(double d, double a, double limit_neg, double limit_pos);

		//ha az �llapot a hat�rokon bel�l van igaz, egy�bk�nt hamis
		bool StateInBoundaries(double state);
	};

	//megval�s�tott mitsubishi robotkar
	//inverse �s forward kinematika sz�mol�s
	class Robot
	{
		Joint joints[6];			//csukl�k (hat�rok, a �s d eltol�sok)
		Matrix4x4 transformMatrix;	//adott csukl��ll�s mellett a transzform�ci�s m�trix
		Vector6 posAndRot;			//adott csukl��ll�s mellett a v�gpont poz�ci�ja �s euler sz�gei
		Vector6 jointState[8];		//a robot v�gpontj�nak helyzet�t eredm�nyez� n�gy �ll�s
		bool validSolution[8];		//megmondja, hogy melyik index� megold�s �rv�nyes
		int solutionNumber;			//h�nyf�lek�ppen �rhet� el a k�rt v�gpont, amennyi jointState �rv�nyes
		Vector6 currentState;		//a robot jelenlegi �ll�sa, k�vetkez� �ll�s meghat�roz�s�hoz ehhez keres k�zelit
		int currentSolutionIndex;	//a v�lasztott megold�s indexe a megold�sok k�z�tt


	private:
		//a mobilr�l k�ld�tt adatok alapj�n sz�mol transfromMatrix-ot
		void MobileDataToMatrix(Vector6& vector);

		//kisz�molja a transzform�ci�s m�trixb�l a v�gpont helyzet�t �s euler sz�geit
		void CalculatePosAndRotFromTransformMatrix();

		//kisz�molja a transzform�ci�s m�trixot a v�gpont helyzet�b�l �s euler sz�geib�l
		void CalculateTransformMatrixFromPosAndRot();

		//els� csukl� mind a n�gy megold�sra
		void CalculateJoint1(double x, double y);

		//2. �s 3. csukl�k solnum �s solnum+1 megold�sainak kisz�mol�sa
		void CalculateJoint23(double r, double z, int solNum);

		//utols� h�rom csukl� az eddig lehets�ges megold�sokra
		void CalculateJoint456(Matrix4x4& request);

		//megn�zi, hogy �rv�nyes-e a megold�s
		//ha nincs �rv�nyes megold�s, hib�t dob
		void CheckSolutionValidity();

		//megkeresi a legjobb megold�st
		void SearchBestSolution();

		Matrix4x4 TransformMatrix03(Vector6& request);
		Matrix4x4 TransformMatrix36(Vector6& request);
		Matrix4x4 TransformMatrix03_KR5like(Vector6& request);

	public:
		//csukl�k param�tereinek be�ll�t�sa
		Robot();

		Robot(bool is_KR5);

		//adott csukl��ll�sok mellett kisz�molja a v�gpot helyzet�t
		Matrix4x4 ForwardKinematic(Vector6& request);

		Matrix4x4 eul2r(Vector6& posandrot);

		//kisz�molja a csukl��ll�sokat a transzform�ci�s m�trixb�l, amik a k�rt v�gpontot eredm�nyezik
		//Az el�z� �ll�shoz legk�zelebbi megold�st adja vissza
		Vector6 InverseKinematic(Matrix4x4& request);

		//kisz�molja a csukl��ll�sokat a v�gpontokb�l �s euler sz�gekb�l, amik a k�rt v�gpontot eredm�nyezik
		//Az el�z� �ll�shoz legk�zelebbi megold�st adja vissza
		Vector6 InverseKinematic(Vector6& request);

		//matlab toolboxb�l implement�lt algoritmussal sz�mol inverz kinematik�t
		Vector6 ikine6s(Matrix4x4& request);

		//megmondja mennyi �rv�nyes megold�sa van
		int ValidSolutionNumber();

		//megmondja, hogy az adott index� megold�s �rv�nyes-e
		bool IsValidSolution(int index);

		//visszaadja a jelenlegi �llapot index�t a megold�sok k�z�tt
		int GetCurrentSolutionIndex();

		//visszaadja az el�z� �llapothoz legk�zelebbi megold�st
		Vector6 GetIkineSolution();

		//visszaadja a k�rt index� megold�st 
		Vector6 GetIkineSolution(int index);

		//visszaadja a transzform�ci�s m�trixot
		Matrix4x4 GetTransformMatrix();

		//visszaadja a poz�ci�t �s euler sz�geket tartalmaz� vektort
		Vector6 GetPositionAndRotation();

		//�t�ll�tja a csukl�k �ll�s�t a k�rt index� megold�sra
		void ChangeState(int index);
	};	
#pragma endregion  

}	

#endif // !M_ROBOT_H   